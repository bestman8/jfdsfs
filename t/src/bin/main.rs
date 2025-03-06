#![no_std]
#![no_main]
#![feature(mem_copy_fn)]
extern crate alloc;

use core::{marker::PhantomData, mem::MaybeUninit};

use alloc::{string::String, vec::Vec};
use blocking_network_stack::Stack;
use embedded_io::{Read, Write};
use esp_alloc as _;
use esp_backtrace as _;
use esp_hal::{ main, peripherals::{RADIO_CLK, RNG, TIMG0}, psram, rng::{self, Rng}, time, timer::timg::TimerGroup};
use esp_hal::{timer::timg::Timer};

use esp_println::println;
use esp_wifi::{
    esp_now::{PeerInfo, BROADCAST_ADDRESS},
    init, wifi::{event::{self, EventExt}, new_with_mode, AccessPointConfiguration, Configuration, WifiApDevice, WifiDeviceMode}, EspWifiController,
};
use esp_hal::time::Duration;
use smoltcp::iface::{SocketSet, SocketStorage};


fn init_psram_heap(start: *mut u8, size: usize) {
    unsafe {
        esp_alloc::HEAP.add_region(esp_alloc::HeapRegion::new(
            start,
            size,
            esp_alloc::MemoryCapability::External.into(),
        ));
    }
}
fn init_heap() {
    const HEAP_SIZE: usize = 64 * 1024;
    static mut HEAP: MaybeUninit<[u8; HEAP_SIZE]> = MaybeUninit::uninit();

    unsafe {
        esp_alloc::HEAP.add_region(esp_alloc::HeapRegion::new(
            HEAP.as_mut_ptr() as *mut u8,
            HEAP_SIZE,
            esp_alloc::MemoryCapability::Internal.into(),
        ));
    }
}

#[main]
fn main() -> ! {
    esp_println::logger::init_logger_from_env();
    let config = esp_hal::Config::default().with_cpu_clock(esp_hal::clock::CpuClock::max());

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
    let wifi = new_wifi(timer.timer0, unsafe { core::mem::transmute_copy(&peripherals.RNG )} , peripherals.RADIO_CLK);
    let (mut device, mut controller) = new_with_mode(&wifi, peripherals.WIFI, WifiApDevice).unwrap();
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
   let _ =controller.set_configuration(&client_config);
   _=controller.start();
    
    println!("{}", esp_alloc::HEAP.stats());
    println!("is wifi started: {:?}", controller.is_started());

    println!("{:?}", controller.capabilities());

    stack
        .set_iface_configuration(&blocking_network_stack::ipv4::Configuration::Client(
            blocking_network_stack::ipv4::ClientConfiguration::Fixed(
                blocking_network_stack::ipv4::ClientSettings {
                    ip: blocking_network_stack::ipv4::Ipv4Addr::from(parse_ip("192.168.2.1")),
                    subnet: blocking_network_stack::ipv4::Subnet {
                        gateway: blocking_network_stack::ipv4::Ipv4Addr::from(parse_ip(
                            "192.168.2.1",
                        )),
                        mask: blocking_network_stack::ipv4::Mask(24),
                    },
                    dns: None,
                    secondary_dns: None,
                },
            ),
        ))
        .unwrap();

    println!("Start busy loop on main. Connect to the AP `esp-wifi` and point your browser to http://192.168.2.1:8080/");
    println!("Use a static IP in the range 192.168.2.2 .. 192.168.2.255, use gateway 192.168.2.1");

    let mut rx_buffer = [0u8; 1536];
    let mut tx_buffer = [0u8; 1536];
    let mut socket = stack.get_socket(&mut rx_buffer, &mut tx_buffer);

    socket.listen(8080).unwrap();

    loop {
        socket.work();

        if !socket.is_open() {
            socket.listen(8080).unwrap();
        }

        if socket.is_connected() {
            println!("Connected");

            let mut time_out = false;
            let deadline = time::now() + Duration::secs(20);
            let mut buffer = [0u8; 1024];
            let mut pos = 0;
            while let Ok(len) = socket.read(&mut buffer[pos..]) {
                let to_print = unsafe { core::str::from_utf8_unchecked(&buffer[..(pos + len)]) };

                if to_print.contains("\r\n\r\n") {
                    println!("{}", to_print);
                    println!();
                    break;
                }

                pos += len;

                if time::now() > deadline {
                    println!("Timeout");
                    time_out = true;
                    break;
                }
            }

            if !time_out {
                socket
                    .write_all(
                        b"HTTP/1.0 200 OK\r\n\r\n\
                    <html>\
                        <body>\
                            <h1>Hello Rust! Hello esp-wifi!</h1>\
                        </body>\
                    </html>\r\n\
                    ",
                    )
                    .unwrap();

                socket.flush().unwrap();
            }

            socket.close();

            println!("Done\n");
            println!();
        }

        let start = time::now();
        while start.duration_since_epoch() < Duration::secs(5) {
            socket.work();
        }
    }
}

fn new_wifi(timer: Timer, rng: RNG, radio: RADIO_CLK) -> EspWifiController<'static> {

    let esp_wifi: EspWifiController<'static> =  init(
        timer, Rng::new(rng), 
        radio
    ).unwrap();
    esp_wifi

    
}
pub fn create_interface<Dm>(device: &mut esp_wifi::wifi::WifiDevice<Dm>) -> smoltcp::iface::Interface 
where
    Dm: WifiDeviceMode,{
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
        esp_hal::time::now()
            .duration_since_epoch()
            .to_micros() as i64,
    )
}
fn parse_ip(ip: &str) -> [u8; 4] {
    let mut result = [0u8; 4];
    for (idx, octet) in ip.split(".").into_iter().enumerate() {
        result[idx] = u8::from_str_radix(octet, 10).unwrap();
    }
    result
}