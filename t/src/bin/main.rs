#![no_std]
#![no_main]
#![feature(mem_copy_fn)]
extern crate alloc;

use core::mem::MaybeUninit;
// use core::ptr::Thin;

use alloc::vec;
use alloc::{string::String, vec::Vec};
use blocking_network_stack::ipv4::RouterConfiguration;
use blocking_network_stack::Stack;
use embedded_io::{Read, Write};
use esp_alloc as _;
use esp_backtrace as _;
use esp_hal::delay;
use esp_hal::timer::timg::Timer;
use esp_hal::{
    main,
    peripherals::{RADIO_CLK, RNG, TIMG0},
    psram,
    rng::Rng,
    time,
    timer::timg::TimerGroup,
};

use esp_hal::time::Duration;
use esp_println::println;
use esp_wifi::{
    init,
    wifi::{new_with_mode, AccessPointConfiguration, Configuration, WifiApDevice, WifiDeviceMode},
    EspWifiController,
};
use smoltcp::iface::{SocketSet, SocketStorage};
use smoltcp::socket;
use smoltcp::wire::IpAddress;

mod mem;
use mem::*;

#[main]
fn main() -> ! {
    esp_println::logger::init_logger_from_env();
    let _config = esp_hal::Config::default().with_cpu_clock(esp_hal::clock::CpuClock::max());

    let peripherals = esp_hal::init(esp_hal::Config::default());

    let (start, size) = psram::psram_raw_parts(&peripherals.PSRAM);
    init_psram_heap(start, size);
    init_heap();

    println!("Going to access PSRAM");
    let mut large_vec = Vec::<u32>::with_capacity(500 * 1024 / 4);

    for i in 0..(500 * 1024 / 4) {
        large_vec.push((i & 0xff) as u32);
    }

    println!("vec size = {} bytes", large_vec.len() * 4);
    println!("vec address = {:p}", large_vec.as_ptr());
    println!("vec[..100] = {:?}", &large_vec[..100]);

    let string = String::from("A string allocated in PSRAM");
    println!("'{}' allocated at {:p}", &string, string.as_ptr());

    println!("{}", esp_alloc::HEAP.stats());

    println!("done");
    let timer: TimerGroup<TIMG0> = TimerGroup::new(peripherals.TIMG0);
    // let rng: RNG = unsafe { core::mem::transmute(&_)};
    let wifi = new_wifi(
        timer.timer0,
        unsafe { core::mem::transmute_copy(&peripherals.RNG) },
        peripherals.RADIO_CLK,
    );
    let (mut device, mut controller) =
        new_with_mode(&wifi, peripherals.WIFI, WifiApDevice).unwrap();
    let iface = create_interface(&mut device);
    let now = || time::now().duration_since_epoch().to_millis();

    let mut socket_set_entries: [SocketStorage; 3] = Default::default();
    let socket_set = SocketSet::new(&mut socket_set_entries[..]);
    let mut rng = Rng::new(peripherals.RNG);

    let mut stack = Stack::new(iface, device, socket_set, now, rng.random());
    let client_config = Configuration::AccessPoint(AccessPointConfiguration {
        ssid: "esp-wifi".try_into().unwrap(),
        ..Default::default()
    });
    let _ = controller.set_configuration(&client_config);
    _ = controller.start();

    println!("{}", esp_alloc::HEAP.stats());
    println!("is wifi started: {:?}", controller.is_started());

    println!("{:?}", controller.capabilities());
    use blocking_network_stack::ipv4::{
        ClientConfiguration::Fixed,
        ClientSettings,
        Configuration::{Client, Router},
        Ipv4Addr, Mask, Subnet,
    };

    stack
        .set_iface_configuration(&Client(Fixed(ClientSettings {
            ip: Ipv4Addr::from(parse_ip("192.168.2.1")),
            subnet: Subnet {
                gateway: Ipv4Addr::from(parse_ip("192.168.2.1")),
                mask: Mask(24),
            },
            dns: None,
            secondary_dns: None,
        })))
        .unwrap();
    println!("{:?}", stack.get_iface_configuration());

    println!("Start busy loop on main. Connect to the AP `esp-wifi` and point your browser to http://192.168.2.1:8080/");
    println!("Use a static IP in the range 192.168.2.2 .. 192.168.2.255, use gateway 192.168.2.1");

    let mut rx_buffer = [0u8; 1536];
    let mut tx_buffer = [0u8; 1536];
    let mut rx_meta = [smoltcp::socket::udp::PacketMetadata::EMPTY];
    let mut tx_meta = [smoltcp::socket::udp::PacketMetadata::EMPTY];
    let mut socket =
        stack.get_udp_socket(&mut rx_meta, &mut rx_buffer, &mut tx_meta, &mut tx_buffer);
    if let Err(fuck) = socket.bind(5000) {
        println!("fuck: {fuck:?}")
    }
    let p = delay::Delay::new();
    for _ in 1..10 {
        println!("i face is up: {}", stack.is_iface_up());
        stack
            .update_iface_configuration(&Router(RouterConfiguration::default()))
            .unwrap();
        stack.reset();

        p.delay_millis(1000);
    }
    loop {
        let mut data = Vec::new();
        let thing = socket.receive(&mut data);
        if let Ok((val, addr, num)) = thing {
            println!("val = {val}\n addr= {addr:?}\n num = {num}");
            println!("data = {data:?}");
        } else if let Err(shit) = thing {
            println!("shit = {shit:?}");
        }
        let _ = socket.send(
            IpAddress::v4(192, 168, 2, 3),
            1200,
            &stack.get_ip_info().unwrap().ip.octets(),
        );

        p.delay_millis(1000);
    }
}

fn new_wifi(timer: Timer, rng: RNG, radio: RADIO_CLK) -> EspWifiController<'static> {
    let esp_wifi: EspWifiController<'static> = init(timer, Rng::new(rng), radio).unwrap();
    esp_wifi
}
pub fn create_interface<Dm>(
    device: &mut esp_wifi::wifi::WifiDevice<Dm>,
) -> smoltcp::iface::Interface
where
    Dm: WifiDeviceMode,
{
    // users could create multiple instances but since they only have one WifiDevice
    // they probably can't do anything bad with that
    smoltcp::iface::Interface::new(
        smoltcp::iface::Config::new(smoltcp::wire::HardwareAddress::Ethernet(
            smoltcp::wire::EthernetAddress::from_bytes(&device.mac_address()),
        )),
        device,
        timestamp(),
    )
}
fn timestamp() -> smoltcp::time::Instant {
    smoltcp::time::Instant::from_micros(
        esp_hal::time::now().duration_since_epoch().to_micros() as i64
    )
}
fn parse_ip(ip: &str) -> [u8; 4] {
    let mut result = [0u8; 4];
    for (idx, octet) in ip.split(".").enumerate() {
        result[idx] = octet.parse::<u8>().unwrap();
    }
    result
}
