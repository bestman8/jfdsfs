//! Embassy access point
//!
//! - creates an open access-point with SSID `esp-wifi`
//! - you can connect to it using a static IP in range 192.168.2.2 .. 192.168.2.255, gateway 192.168.2.1
//! - open http://192.168.2.1:8080/ in your browser - the example will perform an HTTP get request to some "random" server
//!
//! On Android you might need to choose _Keep Accesspoint_ when it tells you the WiFi has no internet connection, Chrome might not want to load the URL - you can use a shell and try `curl` and `ping`
//!
//! Because of the huge task-arena size configured this won't work on ESP32-S2
//!

//% FEATURES: embassy esp-wifi esp-wifi/wifi esp-wifi/utils esp-wifi/sniffer esp-hal/unstable
//% CHIPS: esp32 esp32s2 esp32s3 esp32c2 esp32c3 esp32c6

#![no_std]
#![no_main]

use core::{net::Ipv4Addr, str::FromStr};
extern crate alloc;

use alloc::{vec, vec::Vec};
use embassy_executor::Spawner;
use embassy_net::{
    tcp::TcpSocket,
    udp::{PacketMetadata, UdpMetadata},
    IpListenEndpoint, Ipv4Cidr, Runner, Stack, StackResources, StaticConfigV4,
};
use embassy_time::{Duration, Timer};
use esp_alloc as _;
use esp_backtrace as _;
use esp_hal::{clock::CpuClock, psram, rng::Rng, timer::timg::TimerGroup};
use esp_println::{print, println};
use esp_wifi::{
    init,
    wifi::{
        AccessPointConfiguration, Configuration, WifiApDevice, WifiController, WifiDevice,
        WifiEvent, WifiState,
    },
    EspWifiController,
};

use core::mem::MaybeUninit;
///# Safety
/// there is no safety guaranteed so if you wish feel free to call it 50x in a row
pub unsafe fn init_psram_heap(start: *mut u8, size: usize) {
    unsafe {
        esp_alloc::HEAP.add_region(esp_alloc::HeapRegion::new(
            start,
            size,
            esp_alloc::MemoryCapability::External.into(),
        ));
    }
}
#[allow(static_mut_refs)]
pub fn init_heap() {
    const HEAP_SIZE: usize = 72 * 1024;
    static mut HEAP: MaybeUninit<[u8; HEAP_SIZE]> = MaybeUninit::uninit();

    unsafe {
        esp_alloc::HEAP.add_region(esp_alloc::HeapRegion::new(
            HEAP.as_mut_ptr() as *mut u8,
            HEAP_SIZE,
            esp_alloc::MemoryCapability::Internal.into(),
        ));
    }
}

// use heapless::Vec;

// When you are okay with using a nightly compiler it's better to use https://docs.rs/static_cell/2.1.0/static_cell/macro.make_static.html
macro_rules! mk_static {
    ($t:ty,$val:expr) => {{
        static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        #[deny(unused_attributes)]
        let x = STATIC_CELL.uninit().write(($val));
        x
    }};
}

const GW_IP_ADDR_ENV: Option<&'static str> = option_env!("GATEWAY_IP");

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) -> ! {
    esp_println::logger::init_logger_from_env();
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    // esp_alloc::heap_allocator!(72 * 1024);
    let (start, size) = psram::psram_raw_parts(&peripherals.PSRAM);

    unsafe { init_psram_heap(start, size) };
    init_heap();

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let mut rng = Rng::new(peripherals.RNG);

    let init = &*mk_static!(
        EspWifiController<'static>,
        init(timg0.timer0, rng, peripherals.RADIO_CLK).unwrap()
    );

    let wifi = peripherals.WIFI;
    let (wifi_interface, controller) =
        esp_wifi::wifi::new_with_mode(init, wifi, WifiApDevice).unwrap();

    let timg1 = TimerGroup::new(peripherals.TIMG1);
    esp_hal_embassy::init(timg1.timer0);

    let gw_ip_addr_str = GW_IP_ADDR_ENV.unwrap_or("192.168.2.1");
    let gw_ip_addr = Ipv4Addr::from_str(gw_ip_addr_str).expect("failed to parse gateway ip");

    let config = embassy_net::Config::ipv4_static(StaticConfigV4 {
        address: Ipv4Cidr::new(gw_ip_addr, 24),
        gateway: Some(gw_ip_addr),
        dns_servers: Default::default(),
    });

    let seed = (rng.random() as u64) << 32 | rng.random() as u64;

    // Init network stack
    let (stack, runner) = embassy_net::new(
        wifi_interface,
        config,
        mk_static!(StackResources<3>, StackResources::<3>::new()),
        seed,
    );

    spawner.spawn(connection(controller)).ok();
    spawner.spawn(net_task(runner)).ok();
    spawner.spawn(run_dhcp(stack, gw_ip_addr_str)).ok();

    let mut rx_buffer = [0; 1536];
    let mut tx_buffer = [0; 1536];

    loop {
        if stack.is_link_up() {
            break;
        }
        Timer::after(Duration::from_millis(500)).await;
    }
    println!(
        "Connect to the AP `esp-wifi` and point your browser to http://{gw_ip_addr_str}:8080/"
    );
    println!("DHCP is enabled so there's no need to configure a static IP, just in case:");
    while !stack.is_config_up() {
        Timer::after(Duration::from_millis(100)).await
    }
    stack
        .config_v4()
        .inspect(|c| println!("ipv4 config: {c:?}"));
    let mut tx_metadata = vec![PacketMetadata::EMPTY];
    let mut rx_metadata = vec![PacketMetadata::EMPTY];
    // let mut socket = TcpSocket::new(stack, &mut rx_buffer, &mut tx_buffer);
    let mut socket = embassy_net::udp::UdpSocket::new(
        stack,
        &mut rx_metadata,
        &mut rx_buffer,
        &mut tx_metadata,
        &mut tx_buffer,
    );
    // socket.wait_recv_ready().await;
    socket.bind(5000).unwrap_or_else(|err| println!("{err:?}"));

    loop {
        socket.wait_recv_ready().await;
        let mut buff = [0; 500];
        let (size, meta) = socket.recv_from(&mut buff).await.unwrap();
        println!("buffer = {buff:?} size={size} meta = {meta:?} ");
    }

    // socket.set_timeout(Some(embassy_time::Duration::from_secs(10)));
    // loop {
    //     println!("Wait for connection...");
    //     let r = socket
    //         .accept(IpListenEndpoint {
    //             addr: None,
    //             port: 8080,
    //         })
    //         .await;
    //     println!("Connected...");

    //     if let Err(e) = r {
    //         println!("connect error: {:?}", e);
    //         continue;
    //     }

    //     use embedded_io_async::Write;

    //     let mut buffer = [0u8; 1024];
    //     let mut pos = 0;
    //     loop {
    //         match socket.read(&mut buffer).await {
    //             Ok(0) => {
    //                 println!("read EOF");
    //                 break;
    //             }
    //             Ok(len) => {
    //                 let to_print =
    //                     unsafe { core::str::from_utf8_unchecked(&buffer[..(pos + len)]) };

    //                 if to_print.contains("\r\n\r\n") {
    //                     print!("{}", to_print);
    //                     println!();
    //                     break;
    //                 }

    //                 pos += len;
    //             }
    //             Err(e) => {
    //                 println!("read error: {:?}", e);
    //                 break;
    //             }
    //         };
    //     }

    //     let r = socket
    //         .write_all(
    //             b"HTTP/1.0 200 OK\r\n\r\n\
    //         <html>\
    //             <body>\
    //                 <h1>Hello Rust! Hello esp-wifi!</h1>\
    //             </body>\
    //         </html>\r\n\
    //         ",
    //         )
    //         .await;
    //     if let Err(e) = r {
    //         println!("write error: {:?}", e);
    //     }

    //     let r = socket.flush().await;
    //     if let Err(e) = r {
    //         println!("flush error: {:?}", e);
    //     }
    //     Timer::after(Duration::from_millis(1000)).await;

    //     socket.close();
    //     Timer::after(Duration::from_millis(1000)).await;

    //     socket.abort();
    // }
}

#[embassy_executor::task]
async fn run_dhcp(stack: Stack<'static>, gw_ip_addr: &'static str) {
    use core::net::{Ipv4Addr, SocketAddrV4};

    use edge_dhcp::{
        io::{self, DEFAULT_SERVER_PORT},
        server::{Server, ServerOptions},
    };
    use edge_nal::UdpBind;
    use edge_nal_embassy::{Udp, UdpBuffers};

    let ip = Ipv4Addr::from_str(gw_ip_addr).expect("dhcp task failed to parse gw ip");

    let mut buf = [0u8; 1500];

    let mut gw_buf = [Ipv4Addr::UNSPECIFIED];

    let buffers = UdpBuffers::<3, 1024, 1024, 10>::new();
    let unbound_socket = Udp::new(stack, &buffers);
    let mut bound_socket = unbound_socket
        .bind(core::net::SocketAddr::V4(SocketAddrV4::new(
            Ipv4Addr::UNSPECIFIED,
            DEFAULT_SERVER_PORT,
        )))
        .await
        .unwrap();

    loop {
        _ = io::server::run(
            &mut Server::<_, 64>::new_with_et(ip),
            &ServerOptions::new(ip, Some(&mut gw_buf)),
            &mut bound_socket,
            &mut buf,
        )
        .await
        .inspect_err(|e| log::warn!("DHCP server error: {e:?}"));
        Timer::after(Duration::from_millis(500)).await;
    }
}

#[embassy_executor::task]
async fn connection(mut controller: WifiController<'static>) {
    println!("start connection task");
    println!("Device capabilities: {:?}", controller.capabilities());
    loop {
        if esp_wifi::wifi::wifi_state() == WifiState::ApStarted {
            // wait until we're no longer connected
            controller.wait_for_event(WifiEvent::ApStop).await;
            Timer::after(Duration::from_millis(5000)).await
        }
        if !matches!(controller.is_started(), Ok(true)) {
            let client_config = Configuration::AccessPoint(AccessPointConfiguration {
                ssid: "esp-wifi".try_into().unwrap(),
                ..Default::default()
            });
            controller.set_configuration(&client_config).unwrap();
            println!("Starting wifi");
            controller.start_async().await.unwrap();
            println!("Wifi started!");
        }
    }
}

#[embassy_executor::task]
async fn net_task(mut runner: Runner<'static, WifiDevice<'static, WifiApDevice>>) {
    runner.run().await
}
