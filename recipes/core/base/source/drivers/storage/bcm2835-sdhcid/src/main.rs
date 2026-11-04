use std::process;
use std::sync::{Arc, Mutex};

use driver_block::{Disk, DiskScheme, ExecutorTrait, TrivialExecutor};
use event::{EventFlags, RawEventQueue};
use fdt::Fdt;

mod logdump;
mod sd;
mod sdhci;

/// AIOS: share the EMMC2 controller between the disk scheme and the early log
/// dumper thread, so both serialise on the same mutex.
struct SharedCtrl(Arc<Mutex<sdhci::SdhciCtrl>>);

impl Disk for SharedCtrl {
    fn block_size(&self) -> u32 {
        self.0.lock().unwrap().block_size()
    }
    fn size(&self) -> u64 {
        self.0.lock().unwrap().size()
    }
    async fn read(&mut self, block: u64, buffer: &mut [u8]) -> syscall::Result<usize> {
        let mut c = self.0.lock().unwrap();
        unsafe { c.read_blocks(block, buffer) }
    }
    async fn write(&mut self, block: u64, buffer: &[u8]) -> syscall::Result<usize> {
        let mut c = self.0.lock().unwrap();
        unsafe { c.write_blocks(block, buffer) }
    }
}

#[cfg(target_os = "redox")]
fn get_dtb() -> Vec<u8> {
    std::fs::read("kernel.dtb:").unwrap()
}

#[cfg(target_os = "linux")]
fn get_dtb() -> Vec<u8> {
    use std::env;
    if let Some(arg1) = env::args().nth(1) {
        std::fs::read(arg1).unwrap()
    } else {
        Vec::new()
    }
}

fn main() {
    daemon::Daemon::new(daemon);
}

/// Serve the disk scheme with the given controller until the process is killed.
fn run_scheme<D: Disk + 'static>(daemon: daemon::Daemon, disk: D) -> ! {
    let mut disks = vec![disk];
    let mut scheme = DiskScheme::new(
        Some(daemon),
        "disk.mmc".to_string(),
        disks
            .into_iter()
            .enumerate()
            .map(|(i, disk)| (i as u32, disk))
            .collect(),
        &TrivialExecutor, // TODO: real executor
    );

    let event_queue = RawEventQueue::new().expect("mmcd: failed to open event file");
    event_queue
        .subscribe(scheme.event_handle().raw(), 0, EventFlags::READ)
        .expect("mmcd: failed to event disk scheme");

    // AIOS: keep the initfs namespace so the log dumper thread can keep
    // opening /scheme/sys/log after the driver is serving the disk.
    // libredox::call::setrens(0, 0).expect("mmcd: failed to enter null namespace");

    for event in event_queue {
        let event = event.unwrap();
        if event.fd == scheme.event_handle().raw() {
            TrivialExecutor.block_on(scheme.tick()).unwrap();
        } else {
            println!("Unknown event {}", event.fd);
        }
    }
    process::exit(0);
}

fn daemon(daemon: daemon::Daemon) -> ! {
    common::init();
    let dtb_data = get_dtb();
    println!("read from OS, len = {}", dtb_data.len());
    if dtb_data.len() == 0 {
        process::exit(0);
    }

    let fdt = Fdt::new(&dtb_data).unwrap();
    println!("DTB model = {}", fdt.root().model());

    // The Raspberry Pi 4 (BCM2711) has the SD card on the EMMC2 controller,
    // which implements the standard SDHCI register set. The Raspberry Pi 3
    // uses the legacy controller with the proprietary BCM2835 layout.
    let (compat_node, emmc2) = if let Some(node) = fdt.find_compatible(&["brcm,bcm2711-emmc2"]) {
        (node, true)
    } else if let Some(node) = fdt.find_compatible(&["brcm,bcm2835-sdhci"]) {
        (node, false)
    } else {
        println!("bcm2835-sdhcid: no supported SD host controller in the device tree");
        process::exit(1);
    };

    let reg = compat_node.reg().unwrap().next().unwrap();
    let reg_size = reg.size.unwrap();
    let mut reg_addr = reg.starting_address as usize;
    println!(
        "DeviceMemory start = 0x{:08x}, size = 0x{:08x}",
        reg_addr, reg_size
    );

    // Translate the device address into the CPU address space with the ranges
    // of the bus the controller sits on: the EMMC2 is on /emmc2bus
    // (0x7e340000 -> 0xfe340000), the Pi 3 controller is on /soc.
    for bus in ["/emmc2bus", "/soc"] {
        if let Some(mut ranges) = fdt.find_node(bus).and_then(|f| f.ranges()) {
            if let Some(range) = ranges
                .find(|f| f.child_bus_address <= reg_addr && reg_addr - f.child_bus_address < f.size)
            {
                reg_addr = range.parent_bus_address + (reg_addr - range.child_bus_address);
                println!(
                    "DeviceMemory remapped onto CPU address space: start = 0x{:08x}, size = 0x{:08x}",
                    reg_addr, reg_size
                );
                break;
            }
        }
    }

    let addr = unsafe {
        common::physmap(
            reg_addr,
            reg_size,
            common::Prot::RW,
            common::MemoryType::DeviceMemory,
        )
        .expect("bcm2835-sdhcid: failed to map address") as usize
    };
    println!(
        "ioremap 0x{:08x} to 0x{:08x}",
        reg.starting_address as usize, addr
    );

    if emmc2 {
        // The firmware leaves the EMMC2 clock at 100 MHz by default.
        let mut controller = sdhci::SdhciCtrl::new(addr, 100_000_000);
        unsafe {
            controller
                .init()
                .expect("emmc2: failed to initialize the SD card");
        }
        let shared = Arc::new(Mutex::new(controller));
        // Logdump races with redoxfs switchroot on Pi 4 HW (Lacks grant /
        // Partition(0,2) drop → total ENOENT). Disabled until DiskScheme is
        // exclusive; recover logs via serial or a post-login oneshot later.
        println!(
            "AIOS: logdump DISABLED (was LBA {}) — avoids redoxfs abort",
            logdump::LOG_LBA
        );
        run_scheme(daemon, SharedCtrl(shared));
    } else {
        let mut controller = sd::SdHostCtrl::new(addr);
        unsafe {
            controller.init();
        }
        run_scheme(daemon, controller);
    }
}
