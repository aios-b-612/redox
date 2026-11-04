mod breadcrumb;
mod genet;
mod led;

use std::fs;
use std::io::Read;
use std::thread;
use std::time::Duration;

use common;
use daemon::Daemon;
use driver_network::NetworkScheme;
use fdt::Fdt;
use genet::Genet;
use syscall;

const CPRMAN_PHYS: usize = 0xfe1_01000;
const GPIO_PHYS: usize = 0xfe20_0000;
const GENET_PHYS: usize = 0xfd58_0000;

const PASSWORD: u32 = 0x5A00_0000;
const ENAB: u32 = 1 << 4;
const GATE: u32 = 1 << 6;
const BUSY: u32 = 1 << 7;
const SRC_MASK: u32 = 0xF;
const DIVI_MASK: u32 = 0xFFF << 12;
const SRC_PLLD_PER: u32 = 6;

fn rd(base: usize, off: usize) -> u32 {
    unsafe { core::ptr::read_volatile((base + off) as *const u32) }
}

fn wr(base: usize, off: usize, val: u32) {
    unsafe { core::ptr::write_volatile((base + off) as *mut u32, val) }
}

/// Copia o log do kernel (o que está no monitor) para um arquivo no cartão.
fn dump_kernel_log(tag: &str) {
    let mut buf = Vec::new();
    if let Ok(mut f) = fs::File::open("/scheme/sys/log") {
        let _ = f.read_to_end(&mut buf);
    }
    if buf.is_empty() {
        return;
    }
    let path = format!("/var/lib/ai/boot-{}.log", tag);
    if fs::write(&path, &buf).is_ok() {
        if let Ok(f) = fs::OpenOptions::new().write(true).open(&path) {
            let _ = f.sync_all();
        }
    }
}

fn mark(led: &Option<led::Led>, n: u32, path: &str, msg: &str) {
    breadcrumb::write_at(path, n, msg);
    dump_kernel_log(&format!("{:02}", n));
    println!("AIOS genet: etapa {} — {}", n, msg);
    if let Some(l) = led {
        l.stage((n + 1).min(6));
    }
}

/// NetworkAdapter implementation for BCM2711 GENET.
struct GenetAdapter {
    genet: Genet,
}

impl driver_network::NetworkAdapter for GenetAdapter {
    fn mac_address(&mut self) -> [u8; 6] {
        self.genet.mac()
    }

    fn available_for_read(&mut self) -> usize {
        self.genet.poll_rx();
        self.genet.rx_pending()
    }

    fn read_packet(&mut self, buf: &mut [u8]) -> Result<Option<usize>, syscall::Error> {
        self.genet.poll_rx();
        match self.genet.pop_rx() {
            Some(pkt) => {
                let n = pkt.len().min(buf.len());
                buf[..n].copy_from_slice(&pkt[..n]);
                Ok(Some(n))
            }
            None => Ok(None),
        }
    }

    fn write_packet(&mut self, buf: &[u8]) -> Result<usize, syscall::Error> {
        self.genet
            .tx_frame(buf)
            .map_err(|_| syscall::Error::new(syscall::EIO))
    }
}

fn main() {
    daemon::Daemon::new(daemon);
}

fn daemon(daemon: Daemon) -> ! {
    common::init();
    common::setup_logging(
        "net",
        "platform",
        "bcm2711-genetd",
        common::output_level(),
        common::file_level(),
    );

    // LED (GPIO 42, active low). Map it early; tolerate failure.
    let led_base = unsafe { common::physmap(GPIO_PHYS, 0x1000, common::Prot::RW, common::MemoryType::DeviceMemory) };
    let led = match led_base {
        Ok(p) => {
            let l = led::Led::new(p as usize);
            l.init();
            Some(l)
        }
        Err(_) => None,
    };
    if let Some(l) = &led {
        l.blink(3, 150, 150); // hello
    }

    // ---------- ETAPA 0 ----------
    mark(&led, 0, "/var/lib/ai/net-00", "daemon iniciou + log salvo");
    thread::sleep(Duration::from_millis(200));

    // ---------- ETAPA 1 ----------
    mark(&led, 1, "/var/lib/ai/net-01", "vivo apos 10s");
    thread::sleep(Duration::from_millis(100));

    // ---------- ETAPA 2: leitura CPRMAN ----------
    let mut info = String::new();
    let mut c250 = (0u32, 0u32);
    let mut c125 = (0u32, 0u32);
    let mut have_cpr = false;

    if let Ok(p) = unsafe { common::physmap(CPRMAN_PHYS, 0x2000, common::Prot::RW, common::MemoryType::DeviceMemory) } {
        let b = p as usize;
        have_cpr = true;
        c250 = (rd(b, 0x1e8), rd(b, 0x1ec));
        c125 = (rd(b, 0x210), rd(b, 0x214));
        info = format!(
            "GENET250={:08x}/{:08x} GENET125={:08x}/{:08x}",
            c250.0, c250.1, c125.0, c125.1
        );
    } else {
        info = "physmap CPRMAN falhou".to_string();
    }
    mark(&led, 2, "/var/lib/ai/net-02", &info);
    thread::sleep(Duration::from_millis(100));

    // ---------- ETAPA 3: escrita CPRMAN ----------
    let need250 = have_cpr && (c250.0 & ENAB == 0 || c250.0 & GATE == 0);
    let need125 = have_cpr && (c125.0 & ENAB == 0 || c125.0 & GATE == 0);
    let mut after;

    if have_cpr && (need250 || need125) {
        let b = unsafe {
            common::physmap(CPRMAN_PHYS, 0x2000, common::Prot::RW, common::MemoryType::DeviceMemory).unwrap() as usize
        };
        if need250 {
            if rd(b, 0x1ec) & DIVI_MASK == 0 {
                wr(b, 0x1ec, PASSWORD | (3u32 << 12));
            }
            let src = if rd(b, 0x1e8) & SRC_MASK == 0 {
                SRC_PLLD_PER
            } else {
                rd(b, 0x1e8) & SRC_MASK
            };
            wr(b, 0x1e8, PASSWORD | src | ENAB | GATE);
            for _ in 0..200 {
                if rd(b, 0x1e8) & BUSY == 0 {
                    break;
                }
                thread::sleep(Duration::from_millis(1));
            }
        }
        if need125 {
            if rd(b, 0x214) & DIVI_MASK == 0 {
                wr(b, 0x214, PASSWORD | (6u32 << 12));
            }
            let src = if rd(b, 0x210) & SRC_MASK == 0 {
                SRC_PLLD_PER
            } else {
                rd(b, 0x210) & SRC_MASK
            };
            wr(b, 0x210, PASSWORD | src | ENAB | GATE);
            for _ in 0..200 {
                if rd(b, 0x210) & BUSY == 0 {
                    break;
                }
                thread::sleep(Duration::from_millis(1));
            }
        }
        after = format!(
            "DEPOIS GENET250={:08x}/{:08x} GENET125={:08x}/{:08x}",
            rd(b, 0x1e8),
            rd(b, 0x1ec),
            rd(b, 0x210),
            rd(b, 0x214)
        );
    } else {
        after = "nada a ligar (ja tinham ENAB+GATE ou CPRMAN falhou)".to_string();
    }
    mark(&led, 3, "/var/lib/ai/net-03", &after);
    thread::sleep(Duration::from_millis(200));

    // ---------- ETAPA 4 ----------
    mark(&led, 4, "/var/lib/ai/net-04", "vivo 10s apos escrever CPRMAN");

    // ---------- ETAPA 5: GENET ----------
    let dtb_data = match fs::read("kernel.dtb:") {
        Ok(d) => d,
        Err(_) => {
            if let Some(l) = &led {
                l.error();
            }
            breadcrumb::write(0, "dtb read failed", Some("cannot read kernel.dtb:"));
            loop {
                thread::sleep(Duration::from_secs(10));
            }
        }
    };

    let fdt = match Fdt::new(&dtb_data) {
        Ok(f) => f,
        Err(_) => {
            if let Some(l) = &led {
                l.error();
            }
            breadcrumb::write(0, "fdt parse failed", Some("Fdt::new failed"));
            loop {
                thread::sleep(Duration::from_secs(10));
            }
        }
    };

    let genet_node = match fdt.find_compatible(&["brcm,bcm2711-genet-v5"]) {
        Some(n) => n,
        None => {
            if let Some(l) = &led {
                l.error();
            }
            breadcrumb::write(1, "genet node not found", Some("no compatible brcm,bcm2711-genet-v5"));
            loop {
                thread::sleep(Duration::from_secs(10));
            }
        }
    };

    let reg = match genet_node.reg().and_then(|mut r| r.next()) {
        Some(r) => r,
        None => {
            if let Some(l) = &led {
                l.error();
            }
            breadcrumb::write(2, "genet reg missing", Some("no reg property"));
            loop {
                thread::sleep(Duration::from_secs(10));
            }
        }
    };
    let child_addr = reg.starting_address as usize;
    let size = reg.size.unwrap_or(0x10000) as usize;
    let cpu_addr = genet::translate(&fdt, child_addr);

    let base = match unsafe { common::physmap(cpu_addr, size, common::Prot::RW, common::MemoryType::DeviceMemory) } {
        Ok(p) => p as usize,
        Err(_) => {
            if let Some(l) = &led {
                l.error();
            }
            breadcrumb::write(2, "physmap failed", Some("common::physmap GENET failed"));
            loop {
                thread::sleep(Duration::from_secs(10));
            }
        }
    };

    // MAC from DT (firmware sets local-mac-address).
    let mut mac = [0u8; 6];
    let mut have_mac = false;
    if let Some(prop) = genet_node.property("local-mac-address") {
        let v = prop.value;
        if v.len() >= 6 {
            mac.copy_from_slice(&v[..6]);
            have_mac = true;
        }
    }
    if !have_mac {
        mac = [0x02, 0x00, 0x00, 0x00, 0x00, 0x01];
    }

    // PHY address + max-speed from DT.
    let (phyaddr, max_speed) = genet::find_phy(&fdt, &genet_node);

    // rxid from phy-mode (Pi4: rgmii-rxid).
    let phy_mode = genet_node
        .property("phy-mode")
        .and_then(|p| p.as_str())
        .unwrap_or("?");
    let rxid = phy_mode == "rgmii-rxid";
    mark(
        &led,
        5,
        "/var/lib/ai/net-05a",
        &format!("phyaddr={phyaddr} max_speed={max_speed} phy-mode={phy_mode} rxid={}", rxid as u8),
    );

    match unsafe { common::physmap(GENET_PHYS, 0x1000, common::Prot::RW, common::MemoryType::DeviceMemory) } {
        Ok(p) => {
            let rev = unsafe { core::ptr::read_volatile(p as *const u32) };
            mark(&led, 5, "/var/lib/ai/net-05", &format!("GENET SYS_REV={:08x}", rev));
        }
        Err(e) => {
            mark(&led, 5, "/var/lib/ai/net-05", &format!("physmap GENET falhou ({:?})", e));
        }
    }
    thread::sleep(Duration::from_millis(100));

    // ---------- ETAPA 6: GENET init + beacon loop ----------
    let genet = match Genet::new(base, mac, phyaddr, max_speed, rxid) {
        Ok(g) => g,
        Err(e) => {
            if let Some(l) = &led {
                l.error();
            }
            mark(&led, 6, "/var/lib/ai/net-06", &format!("genet new failed: {e}"));
            // Never block init forever: type=notify waits for ready().
            let _ = daemon.ready();
            loop {
                thread::sleep(Duration::from_secs(10));
            }
        }
    };

    let mut genet = genet;
    match genet.init(led.as_ref()) {
        Ok(()) => {}
        Err(e) => {
            if let Some(l) = &led {
                l.error();
            }
            mark(&led, 6, "/var/lib/ai/net-06", &format!("genet init failed: {e}"));
            let _ = daemon.ready();
            loop {
                thread::sleep(Duration::from_secs(10));
            }
        }
    }

    mark(&led, 6, "/var/lib/ai/net-06", "sobreviveu ao GENET — beacon-only M1");

    // Ready before any scheme/TX: etapa 7+ hung the board when NetworkScheme
    // was registered first. M1 = beacons only; M2 scheme later.
    let _ = daemon.ready();
    mark(&led, 7, "/var/lib/ai/net-07", "daemon.ready — a transmitir AIOS-BOOT");

    let mut genet = genet;
    let mut counter: u64 = 0;
    loop {
        println!("AIOS genet: loop n={counter} pre-phy");
        let linked = genet.poll_phy_and_adjust();
        let speed = genet.max_speed();
        println!("AIOS genet: phy link={} speed={}", linked as u8, speed);
        if !linked || counter % 10 == 0 {
            let diag = genet.mdio_diag();
            println!("AIOS genet: {diag}");
            if counter % 10 == 0 {
                breadcrumb::write(4, &diag, None);
            }
        }

        counter += 1;
        let msg = format!(
            "AIOS-BOOT n={} mac={:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x} speed={} link={}",
            counter, mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], speed, linked as u8
        );
        match genet.tx_beacon(counter, &msg) {
            Ok(()) => println!("AIOS genet: beacon {counter} ok"),
            Err(e) => println!("AIOS genet: beacon {counter} err: {e}"),
        }
        genet.poll_rx();

        if counter % 10 == 0 {
            breadcrumb::write(
                4,
                &format!("beacon-only speed={speed} link={}", linked as u8),
                None,
            );
        }
        if let Some(l) = &led {
            l.success();
        }
        thread::sleep(Duration::from_secs(2));
    }
}

fn translate(fdt: &Fdt, child_addr: usize) -> usize {
    genet::translate(fdt, child_addr)
}

fn find_phy(fdt: &Fdt, genet: &fdt::node::FdtNode) -> (u32, u32) {
    genet::find_phy(fdt, genet)
}