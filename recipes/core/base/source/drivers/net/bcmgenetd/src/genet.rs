//! BCM2711 GENET v5 Ethernet driver (Raspberry Pi 4).
//!
//! Based on the U-Boot `bcmgenet` driver (drivers/net/bcmgenet.c) and the
//! Linux kernel driver (drivers/net/ethernet/broadcom/genet/bcmgenet.c).
//! Only GENET v5 (Pi 4) is supported. Polling-based (no IRQs).

use std::thread;
use std::time::{Duration, Instant};

use common::dma::Dma;
use common::{physmap, Prot, MemoryType};
use fdt::Fdt;

// Register block offsets (from U-Boot bcmgenet.c / Linux bcmgenet.h)
const GENET_SYS_OFF: usize = 0x0000;
const GENET_EXT_OFF: usize = 0x0080;
const GENET_RBUF_OFF: usize = 0x0300;
const GENET_UMAC_OFF: usize = 0x0800;
const GENET_RX_OFF: usize = 0x2000;
const GENET_TX_OFF: usize = 0x4000;

const SYS_REV_CTRL: usize = GENET_SYS_OFF + 0x00;
const SYS_PORT_CTRL: usize = GENET_SYS_OFF + 0x04;
const SYS_RBUF_FLUSH_CTRL: usize = GENET_SYS_OFF + 0x08;

const PORT_MODE_EXT_GPHY: u32 = 3;

const EXT_RGMII_OOB_CTRL: usize = GENET_EXT_OFF + 0x0C;
const RGMII_LINK: u32 = 1 << 4;
const OOB_DISABLE: u32 = 1 << 5;
const RGMII_MODE_EN: u32 = 1 << 6;
const ID_MODE_DIS: u32 = 1 << 16;

const RBUF_CTRL: usize = GENET_RBUF_OFF + 0x00;
const RBUF_TBUF_SIZE_CTRL: usize = GENET_RBUF_OFF + 0xB4;
const RBUF_ALIGN_2B: u32 = 1 << 1;

const UMAC_CMD: usize = GENET_UMAC_OFF + 0x08;
const UMAC_MAC0: usize = GENET_UMAC_OFF + 0x0C;
const UMAC_MAC1: usize = GENET_UMAC_OFF + 0x10;
const UMAC_MAX_FRAME_LEN: usize = GENET_UMAC_OFF + 0x14;
const UMAC_MIB_CTRL: usize = GENET_UMAC_OFF + 0x580;
const MDIO_CMD: usize = GENET_UMAC_OFF + 0x614;
const UMAC_TX_FLUSH: usize = GENET_UMAC_OFF + 0x334;

const CMD_TX_EN: u32 = 1 << 0;
const CMD_RX_EN: u32 = 1 << 1;
const CMD_SW_RESET: u32 = 1 << 13;
const CMD_LCL_LOOP_EN: u32 = 1 << 15;
const CMD_SPEED_SHIFT: u32 = 2;
const CMD_SPEED_MASK: u32 = 3;
const UMAC_SPEED_10: u32 = 0;
const UMAC_SPEED_100: u32 = 1;
const UMAC_SPEED_1000: u32 = 2;

const MIB_RESET_RX: u32 = 1 << 0;
const MIB_RESET_RUNT: u32 = 1 << 1;
const MIB_RESET_TX: u32 = 1 << 2;

const MDIO_START_BUSY: u32 = 1 << 29;
const MDIO_READ_FAIL: u32 = 1 << 28;
const MDIO_RD: u32 = 2 << 26;
const MDIO_WR: u32 = 1 << 26;
const MDIO_PMD_SHIFT: u32 = 21;
const MDIO_REG_SHIFT: u32 = 16;

const DMA_DESC_LENGTH_STATUS: usize = 0x00;
const DMA_DESC_ADDRESS_LO: usize = 0x04;
const DMA_DESC_ADDRESS_HI: usize = 0x08;
const DMA_DESC_SIZE: usize = 12;

const TOTAL_DESCS: usize = 256;
const RX_BUF_LENGTH: usize = 2048;
const TX_BUF_LENGTH: usize = 2048;
const RX_TOTAL_BUFSIZE: usize = RX_BUF_LENGTH * TOTAL_DESCS;
const DEFAULT_Q: usize = 0x10;
const DMA_RING_SIZE: usize = 0x40;
const DMA_MAX_BURST_LENGTH: u32 = 0x8;
const DMA_FC_THRESH_HI: u32 = (TOTAL_DESCS >> 4) as u32;
const DMA_FC_THRESH_LO: u32 = 5;
const DMA_FC_THRESH_VALUE: u32 = (DMA_FC_THRESH_LO << 16) | DMA_FC_THRESH_HI;

const DMA_RING_BUF_EN_SHIFT: u32 = 1;
const DMA_EN: u32 = 1 << 0;

const DMA_OWN: u32 = 0x8000;
const DMA_EOP: u32 = 0x4000;
const DMA_SOP: u32 = 0x2000;
const DMA_BUFLENGTH_SHIFT: u32 = 16;
const DMA_BUFLENGTH_MASK: u32 = 0x0fff;
const DMA_TX_APPEND_CRC: u32 = 0x0040;
const DMA_TX_QTAG_SHIFT: u32 = 7;

const GENET_RDMA_REG_OFF: usize = GENET_RX_OFF + TOTAL_DESCS * DMA_DESC_SIZE;
const GENET_TDMA_REG_OFF: usize = GENET_TX_OFF + TOTAL_DESCS * DMA_DESC_SIZE;
const DMA_RINGS_SIZE: usize = DMA_RING_SIZE * (DEFAULT_Q + 1);

const DMA_READ_PTR: usize = 0x00;
const DMA_CONS_INDEX: usize = 0x08;
const DMA_PROD_INDEX: usize = 0x0C;
const DMA_RING_BUF_SIZE: usize = 0x10;
const DMA_START_ADDR: usize = 0x14;
const DMA_END_ADDR: usize = 0x1C;
const DMA_MBUF_DONE_THRESH: usize = 0x24;
const DMA_FLOW_PERIOD: usize = 0x28;
const DMA_WRITE_PTR: usize = 0x2C;
const RDMA_XON_XOFF_THRESH: usize = 0x28;

const DMA_RING_CFG: usize = 0x00;
const DMA_CTRL: usize = 0x04;
const DMA_SCB_BURST_SIZE: usize = 0x0C;

const RDMA_RING_BASE: usize = GENET_RDMA_REG_OFF + DEFAULT_Q * DMA_RING_SIZE;
const RDMA_CTRL_BASE: usize = GENET_RDMA_REG_OFF + DMA_RINGS_SIZE;
const TDMA_RING_BASE: usize = GENET_TDMA_REG_OFF + DEFAULT_Q * DMA_RING_SIZE;
const TDMA_CTRL_BASE: usize = GENET_TDMA_REG_OFF + DMA_RINGS_SIZE;

const ENET_MAX_MTU_SIZE: u32 = 1528;

fn rd32(base: usize, off: usize) -> u32 {
    unsafe { core::ptr::read_volatile((base + off) as *const u32) }
}
fn wr32(base: usize, off: usize, val: u32) {
    unsafe { core::ptr::write_volatile((base + off) as *mut u32, val) }
}
fn sleep_ms(ms: u64) {
    std::thread::sleep(std::time::Duration::from_millis(ms));
}

pub struct Genet {
    pub base: usize,
    pub mac: [u8; 6],
    pub phyaddr: u32,
    pub max_speed: u32,
    rxid: bool,
    rx_buf: common::dma::Dma<[u8]>,
    tx_buf: common::dma::Dma<[u8]>,
    tx_slot: usize,
    tx_prod: u32,
    rx_index: usize,
    c_index: usize,
    rx_queue: Vec<Vec<u8>>,
}

impl Genet {
    pub fn new(base: usize, mac: [u8; 6], phyaddr: u32, max_speed: u32, rxid: bool) -> Result<Self, &'static str> {
        let rx_dma = common::dma::Dma::<[u8]>::zeroed_slice(RX_TOTAL_BUFSIZE)
            .map_err(|_| "dma alloc rx buf failed")?;
        let rx_buf = unsafe { rx_dma.assume_init() };
        let tx_dma = common::dma::Dma::<[u8]>::zeroed_slice(2048)
            .map_err(|_| "dma alloc tx buf failed")?;
        let tx_buf = unsafe { tx_dma.assume_init() };
        Ok(Self {
            base,
            mac,
            phyaddr,
            max_speed,
            rxid,
            rx_buf,
            tx_buf,
            tx_slot: 0,
            tx_prod: 0,
            rx_index: 0,
            c_index: 0,
            rx_queue: Vec::new(),
        })
    }

    pub fn init(&mut self, led: Option<&crate::led::Led>) -> Result<(), &'static str> {
        let base = self.base;
        // stage 1: rev check
        crate::breadcrumb::write(1, "init: reading SYS_REV_CTRL", None);
        if let Some(l) = led {
            l.stage(1);
        }
        let rev = rd32(base, SYS_REV_CTRL);
        let major = (rev >> 24) & 0x0F;
        let minor = (rev >> 16) & 0x0F;
        crate::breadcrumb::write(1, &format!("rev major={} minor={}", major, minor), None);
        if major != 6 {
            crate::breadcrumb::write(1, "unsupported GENET major", Some("major != 6"));
            if let Some(l) = led {
                l.error();
            }
            return Err("unsupported GENET major");
        }

        // stage 2: interface set
        crate::breadcrumb::write(2, "interface set (PORT_MODE_EXT_GPHY)", None);
        if let Some(l) = led {
            l.stage(2);
        }
        wr32(base, SYS_PORT_CTRL, PORT_MODE_EXT_GPHY);

        // umac reset
        self.umac_reset();
        crate::breadcrumb::write(2, "umac reset done", None);

        // mac address
        self.write_hwaddr();
        crate::breadcrumb::write(2, "mac written", None);

        // disable dma
        self.disable_dma();

        // init rings
        self.rx_ring_init();
        self.rx_descs_init();
        self.tx_ring_init();
        crate::breadcrumb::write(3, "rings initialized", None);
        if let Some(l) = led { l.stage(3); }

        // enable dma
        self.enable_dma();
        let ring_cfg = rd32(base, TDMA_CTRL_BASE + DMA_RING_CFG);
        let dma_ctrl = rd32(base, TDMA_CTRL_BASE + DMA_CTRL);
        crate::breadcrumb::write(
            3,
            &format!(
                "dma enabled TDMA RING_CFG={:08x} CTRL={:08x} (expect bit16 cfg, bit17+EN ctrl)",
                ring_cfg, dma_ctrl
            ),
            None,
        );
        println!(
            "AIOS genet: TDMA RING_CFG={:08x} CTRL={:08x}",
            ring_cfg, dma_ctrl
        );

        // phy init
        crate::breadcrumb::write(4, "phy init starting", None);
        if let Some(l) = led { l.stage(4); }
        if let Err(e) = self.phy_init() {
            crate::breadcrumb::write(4, "phy init failed", Some(e));
            if let Some(l) = led { l.error(); }
            return Err(e);
        }
        crate::breadcrumb::write(4, &format!("phy link up, speed={}", self.max_speed), None);

        // adjust link
        self.adjust_link();
        crate::breadcrumb::write(5, &format!("adjust_link speed={}", self.max_speed), None);
        if let Some(l) = led { l.stage(5); }

        // enable mac tx/rx
        let cmd = rd32(base, UMAC_CMD);
        wr32(base, UMAC_CMD, cmd | CMD_TX_EN | CMD_RX_EN);
        crate::breadcrumb::write(5, "mac tx/rx enabled", None);

        // oob — U-Boot: RGMII_LINK|MODE_EN, ID_MODE_DIS for rgmii and rgmii-rxid (Pi4).
        self.apply_rgmii_oob(true);

        Ok(())
    }

    /// EXT_RGMII_OOB_CTRL: clear OOB_DISABLE, set MODE_EN; ID_MODE_DIS for Pi4 phy-modes.
    fn apply_rgmii_oob(&self, link: bool) {
        let base = self.base;
        let mut v = rd32(base, EXT_RGMII_OOB_CTRL);
        v &= !OOB_DISABLE;
        v |= RGMII_MODE_EN;
        if link {
            v |= RGMII_LINK;
        } else {
            v &= !RGMII_LINK;
        }
        // Pi4 is rgmii-rxid; U-Boot also sets ID_MODE_DIS for plain rgmii.
        // self.rxid is true for rgmii-rxid; always set on this board path.
        v |= ID_MODE_DIS;
        let _ = self.rxid; // retained for future phy-mode variants
        wr32(base, EXT_RGMII_OOB_CTRL, v);
    }

    fn umac_reset(&self) {
        let base = self.base;
        let reg = rd32(base, SYS_RBUF_FLUSH_CTRL);
        wr32(base, SYS_RBUF_FLUSH_CTRL, reg | (1 << 1));
        sleep_ms(10);
        wr32(base, SYS_RBUF_FLUSH_CTRL, reg & !(1 << 1));
        sleep_ms(10);
        wr32(base, SYS_RBUF_FLUSH_CTRL, 0);
        sleep_ms(10);

        wr32(base, UMAC_CMD, 0);
        wr32(base, UMAC_CMD, CMD_SW_RESET | CMD_LCL_LOOP_EN);
        sleep_ms(2);
        wr32(base, UMAC_CMD, 0);

        wr32(base, UMAC_MIB_CTRL, MIB_RESET_RX | MIB_RESET_RUNT | MIB_RESET_TX);
        wr32(base, UMAC_MIB_CTRL, 0);

        wr32(base, UMAC_MAX_FRAME_LEN, ENET_MAX_MTU_SIZE);

        let reg = rd32(base, RBUF_CTRL);
        wr32(base, RBUF_CTRL, reg | RBUF_ALIGN_2B);

        wr32(base, RBUF_TBUF_SIZE_CTRL, 1);
    }

    fn write_hwaddr(&self) {
        let base = self.base;
        let mac = self.mac;
        let reg = (mac[0] as u32) << 24 | (mac[1] as u32) << 16 | (mac[2] as u32) << 8 | (mac[3] as u32);
        wr32(base, UMAC_MAC0, reg);
        let reg = (mac[4] as u32) << 8 | (mac[5] as u32);
        wr32(base, UMAC_MAC1, reg);
    }

    fn disable_dma(&self) {
        let base = self.base;
        let v = rd32(base, TDMA_CTRL_BASE + DMA_CTRL);
        wr32(base, TDMA_CTRL_BASE + DMA_CTRL, v & !DMA_EN);
        let v = rd32(base, RDMA_CTRL_BASE + DMA_CTRL);
        wr32(base, RDMA_CTRL_BASE + DMA_CTRL, v & !DMA_EN);
        wr32(base, UMAC_TX_FLUSH, 1);
        sleep_ms(10);
        wr32(base, UMAC_TX_FLUSH, 0);
    }

    fn rx_ring_init(&mut self) {
        let base = self.base;
        wr32(base, RDMA_CTRL_BASE + DMA_SCB_BURST_SIZE, DMA_MAX_BURST_LENGTH);
        wr32(base, RDMA_RING_BASE + DMA_START_ADDR, 0);
        wr32(base, RDMA_RING_BASE + DMA_READ_PTR, 0);
        wr32(base, RDMA_RING_BASE + DMA_WRITE_PTR, 0);
        wr32(base, RDMA_RING_BASE + DMA_END_ADDR, (TOTAL_DESCS * DMA_DESC_SIZE / 4 - 1) as u32);
        let c_index = rd32(base, RDMA_RING_BASE + DMA_PROD_INDEX);
        wr32(base, RDMA_RING_BASE + DMA_CONS_INDEX, c_index);
        self.c_index = (c_index & 0xFFFF) as usize;
        self.rx_index = self.c_index & 0xFF;
        wr32(base, RDMA_RING_BASE + DMA_RING_BUF_SIZE, ((TOTAL_DESCS << 16) | RX_BUF_LENGTH) as u32);
        wr32(base, RDMA_RING_BASE + RDMA_XON_XOFF_THRESH, DMA_FC_THRESH_VALUE);
        // DMA_RING_CFG uses bit q; DMA_CTRL uses bit (q + DMA_RING_BUF_EN_SHIFT). U-Boot bcmgenet.
        wr32(base, RDMA_CTRL_BASE + DMA_RING_CFG, 1 << (DEFAULT_Q as u32));
    }

    fn rx_descs_init(&self) {
        let base = self.base;
        let rx_phys = self.rx_buf.physical();
        let len_stat = (RX_BUF_LENGTH as u32) << DMA_BUFLENGTH_SHIFT | DMA_OWN;
        for i in 0..TOTAL_DESCS {
            let buf_phys = rx_phys + i * RX_BUF_LENGTH;
            let desc = base + GENET_RX_OFF + i * DMA_DESC_SIZE;
            wr32(desc, DMA_DESC_ADDRESS_LO, buf_phys as u32);
            wr32(desc, DMA_DESC_ADDRESS_HI, (buf_phys >> 32) as u32);
            wr32(desc, DMA_DESC_LENGTH_STATUS, len_stat);
        }
    }

    fn tx_ring_init(&mut self) {
        let base = self.base;
        wr32(base, TDMA_CTRL_BASE + DMA_SCB_BURST_SIZE, DMA_MAX_BURST_LENGTH);
        wr32(base, TDMA_RING_BASE + DMA_START_ADDR, 0);
        wr32(base, TDMA_RING_BASE + DMA_READ_PTR, 0);
        wr32(base, TDMA_RING_BASE + DMA_WRITE_PTR, 0);
        wr32(base, TDMA_RING_BASE + DMA_END_ADDR, (TOTAL_DESCS * DMA_DESC_SIZE / 4 - 1) as u32);
        let cons = rd32(base, TDMA_RING_BASE + DMA_CONS_INDEX);
        wr32(base, TDMA_RING_BASE + DMA_PROD_INDEX, cons);
        self.tx_slot = (cons as usize) & 0xFF;
        self.tx_prod = cons;
        wr32(base, TDMA_RING_BASE + DMA_MBUF_DONE_THRESH, 1);
        wr32(base, TDMA_RING_BASE + DMA_FLOW_PERIOD, 0);
        wr32(base, TDMA_RING_BASE + DMA_RING_BUF_SIZE, ((TOTAL_DESCS << 16) | RX_BUF_LENGTH) as u32);
        // DMA_RING_CFG bit = DEFAULT_Q (16), not 16+SHIFT — was enabling wrong ring → TX stall.
        wr32(base, TDMA_CTRL_BASE + DMA_RING_CFG, 1 << (DEFAULT_Q as u32));
    }

    fn enable_dma(&self) {
        let base = self.base;
        let dma_ctrl = (1 << (DEFAULT_Q as u32 + DMA_RING_BUF_EN_SHIFT)) | DMA_EN;
        wr32(base, TDMA_CTRL_BASE + DMA_CTRL, dma_ctrl);
        let v = rd32(base, RDMA_CTRL_BASE + DMA_CTRL);
        wr32(base, RDMA_CTRL_BASE + DMA_CTRL, v | dma_ctrl);
    }

    fn adjust_link(&self) {
        let base = self.base;
        self.apply_rgmii_oob(true);
        let sp = match self.max_speed {
            1000 => UMAC_SPEED_1000,
            100 => UMAC_SPEED_100,
            10 => UMAC_SPEED_10,
            _ => UMAC_SPEED_100,
        };
        let cmd = rd32(base, UMAC_CMD);
        wr32(base, UMAC_CMD, (cmd & !((CMD_SPEED_MASK) << CMD_SPEED_SHIFT)) | (sp << CMD_SPEED_SHIFT));
    }

    /// Dump MDIO: PHYID/BMCR/BMSR at configured addr + scan 0..31 for non-zero IDs.
    pub fn mdio_diag(&self) -> String {
        let base = self.base;
        let phy = self.phyaddr;
        let (bmcr, bmcr_st) = mdio_read_status(base, phy, 0);
        let (bmsr, bmsr_st) = mdio_read_status(base, phy, 1);
        let (id1, id1_st) = mdio_read_status(base, phy, 2);
        let (id2, id2_st) = mdio_read_status(base, phy, 3);
        let mut found = Vec::new();
        for a in 0u32..32 {
            let (hi, st) = mdio_read_status(base, a, 2);
            if st != MdioStatus::Ok || hi == 0 || hi == 0xffff {
                continue;
            }
            let (lo, st2) = mdio_read_status(base, a, 3);
            if st2 == MdioStatus::Ok && lo != 0xffff {
                found.push(format!("a{}={:04x}{:04x}", a, hi, lo));
            }
        }
        let scan = if found.is_empty() {
            "scan=none".to_string()
        } else {
            format!("scan=[{}]", found.join(","))
        };
        format!(
            "phyaddr={} BMCR={:04x}({}) BMSR={:04x}({}) PHYID={:04x}{:04x}({}/{}) {}",
            phy,
            bmcr,
            bmcr_st.as_str(),
            bmsr,
            bmsr_st.as_str(),
            id1,
            id2,
            id1_st.as_str(),
            id2_st.as_str(),
            scan
        )
    }

    fn phy_init(&mut self) -> Result<(), &'static str> {
        let base = self.base;
        let mut phy = self.phyaddr;

        // Before reset: dump + optionally adopt first responding PHY.
        let diag0 = self.mdio_diag();
        crate::breadcrumb::write(4, &diag0, None);
        println!("AIOS genet: mdio pre-reset {diag0}");

        let (id1, st1) = mdio_read_status(base, phy, 2);
        if st1 != MdioStatus::Ok || id1 == 0 || id1 == 0xffff {
            for a in 0u32..32 {
                let (hi, st) = mdio_read_status(base, a, 2);
                if st == MdioStatus::Ok && hi != 0 && hi != 0xffff {
                    println!("AIOS genet: phyaddr {} dead, switching to {}", phy, a);
                    phy = a;
                    self.phyaddr = a;
                    break;
                }
            }
        }

        // reset
        mdio_write(base, phy, 0, 0x8000);
        let deadline = Instant::now() + Duration::from_secs(1);
        let mut reset_ok = false;
        while Instant::now() <= deadline {
            let (bmcr, st) = mdio_read_status(base, phy, 0);
            if st == MdioStatus::Ok && (bmcr & 0x8000) == 0 {
                reset_ok = true;
                break;
            }
            // If MDIO is dead, don't spin claiming "reset done" on zeros.
            if st != MdioStatus::Ok {
                sleep_ms(10);
                continue;
            }
            sleep_ms(10);
        }
        if !reset_ok {
            let diag = self.mdio_diag();
            crate::breadcrumb::write(4, "phy reset incomplete", Some(&diag));
            println!("AIOS genet: phy reset incomplete {diag}");
            // soft-continue: still try advertise / wait link
        }

        // advertising: 10/100 full+half + pause; 1000 full+half
        mdio_write(base, phy, 4, 0x0DE1);
        mdio_write(base, phy, 9, 0x0300);
        // enable autoneg
        mdio_write(base, phy, 0, 0x1200);

        // wait link up — do NOT hard-fail: type=notify would block init/login.
        // Default 10 Mbps if still down (PC saw 10baseT partner); poll_phy later.
        let deadline = Instant::now() + Duration::from_secs(5);
        let mut linked = false;
        loop {
            let (bmsr, st) = mdio_read_status(base, phy, 1);
            if st == MdioStatus::Ok && bmsr & 0x0004 != 0 {
                linked = true;
                break;
            }
            if Instant::now() > deadline {
                break;
            }
            sleep_ms(100);
        }

        let diag1 = self.mdio_diag();
        crate::breadcrumb::write(
            4,
            &format!("phy post-init linked={} {}", linked as u8, diag1),
            None,
        );
        println!("AIOS genet: mdio post-init linked={} {diag1}", linked as u8);

        // Soft-fail default MUST match typical autoneg on Pi4↔PC (100), not 10:
        // UMAC@10 + PHY@100 produces align_errors and zero good RX on the peer.
        self.max_speed = if linked {
            self.read_phy_speed()
        } else {
            100
        };
        Ok(())
    }

    fn read_phy_speed(&self) -> u32 {
        let base = self.base;
        let phy = self.phyaddr;
        let lpagb = mdio_read(base, phy, 10);
        let lpa = mdio_read(base, phy, 5);
        if (lpagb & 0x0800) != 0 || (lpagb & 0x0400) != 0 {
            1000
        } else if (lpa & 0x0100) != 0 || (lpa & 0x0080) != 0 {
            100
        } else if (lpa & 0x0040) != 0 || (lpa & 0x0020) != 0 {
            10
        } else {
            10
        }
    }

    /// Re-read BMSR; if link/speed changed, update UMAC. Call from beacon loop.
    pub fn poll_phy_and_adjust(&mut self) -> bool {
        let base = self.base;
        let phy = self.phyaddr;
        let (bmsr, st) = mdio_read_status(base, phy, 1);
        if st != MdioStatus::Ok || bmsr & 0x0004 == 0 {
            // MDIO blind / link down: keep MAC at 100 so RGMII matches common autoneg.
            if self.max_speed != 100 {
                self.max_speed = 100;
                self.adjust_link();
            }
            return false;
        }
        let speed = self.read_phy_speed();
        if speed != self.max_speed {
            self.max_speed = speed;
            self.adjust_link();
        }
        true
    }

    /// Transmit an Ethernet frame (pads to 60 bytes). Used by NetworkScheme.
    pub fn tx_frame(&mut self, frame: &[u8]) -> Result<usize, &'static str> {
        if frame.is_empty() {
            return Err("empty frame");
        }
        let base = self.base;
        let frame_len = frame.len().max(60).min(TX_BUF_LENGTH);
        let tx_virt = self.tx_buf.virt_addr();
        unsafe {
            core::ptr::write_bytes(tx_virt as *mut u8, 0, frame_len);
            core::ptr::copy_nonoverlapping(
                frame.as_ptr(),
                tx_virt as *mut u8,
                frame.len().min(frame_len),
            );
        }
        let tx_phys = self.tx_buf.physical();

        let desc = base + GENET_TX_OFF + self.tx_slot * DMA_DESC_SIZE;
        wr32(desc, DMA_DESC_ADDRESS_LO, tx_phys as u32);
        wr32(desc, DMA_DESC_ADDRESS_HI, (tx_phys >> 32) as u32);
        let len_stat = ((frame_len as u32) << DMA_BUFLENGTH_SHIFT)
            | (0x3F << DMA_TX_QTAG_SHIFT)
            | DMA_TX_APPEND_CRC
            | DMA_SOP
            | DMA_EOP;
        wr32(desc, DMA_DESC_LENGTH_STATUS, len_stat);

        self.tx_slot = (self.tx_slot + 1) % TOTAL_DESCS;
        self.tx_prod = self.tx_prod.wrapping_add(1);
        wr32(base, TDMA_RING_BASE + DMA_PROD_INDEX, self.tx_prod);

        let deadline = Instant::now() + Duration::from_millis(100);
        loop {
            let cons = rd32(base, TDMA_RING_BASE + DMA_CONS_INDEX) & 0xFFFF;
            if cons == (self.tx_prod & 0xFFFF) {
                return Ok(frame.len());
            }
            if Instant::now() > deadline {
                return Err("tx timeout");
            }
            thread::sleep(Duration::from_millis(1));
        }
    }

    pub fn tx_beacon(&mut self, counter: u64, msg: &str) -> Result<(), &'static str> {
        let mut frame = [0u8; 60];
        frame[0..6].copy_from_slice(&[0xff; 6]);
        frame[6..12].copy_from_slice(&self.mac);
        frame[12] = 0x88;
        frame[13] = 0xB5;
        let payload = format!("AIOS-BOOT n={} {}", counter, msg);
        let pb = payload.as_bytes();
        let n = pb.len().min(46);
        frame[14..14 + n].copy_from_slice(&pb[..n]);
        self.tx_frame(&frame).map(|_| ())
    }

    /// Drain hardware RX descriptors into `rx_queue` (software backlog for NetworkScheme).
    pub fn poll_rx(&mut self) {
        let base = self.base;
        let prod = rd32(base, RDMA_RING_BASE + DMA_PROD_INDEX) & 0xFFFF;
        if prod as usize == (self.c_index & 0xFFFF) {
            return;
        }
        while (self.c_index & 0xFFFF) != prod as usize {
            let slot = self.rx_index;
            let desc = base + GENET_RX_OFF + slot * DMA_DESC_SIZE;
            let len_stat = rd32(desc, DMA_DESC_LENGTH_STATUS);
            let len = ((len_stat >> DMA_BUFLENGTH_SHIFT) & DMA_BUFLENGTH_MASK) as usize;
            // Copy out before re-arming (RBUF_ALIGN_2B → 2-byte pad at start).
            if len > 2 && len <= RX_BUF_LENGTH {
                let payload_len = len - 2;
                let mut pkt = vec![0u8; payload_len];
                let src = self.rx_buf.virt_addr() as *const u8;
                unsafe {
                    core::ptr::copy_nonoverlapping(
                        src.add(slot * RX_BUF_LENGTH + 2),
                        pkt.as_mut_ptr(),
                        payload_len,
                    );
                }
                if self.rx_queue.len() < 64 {
                    self.rx_queue.push(pkt);
                }
            }
            let buf_phys = self.rx_buf.physical() + slot * RX_BUF_LENGTH;
            wr32(desc, DMA_DESC_ADDRESS_LO, buf_phys as u32);
            wr32(desc, DMA_DESC_ADDRESS_HI, (buf_phys >> 32) as u32);
            wr32(
                desc,
                DMA_DESC_LENGTH_STATUS,
                (RX_BUF_LENGTH as u32) << DMA_BUFLENGTH_SHIFT | DMA_OWN,
            );
            self.c_index = (self.c_index + 1) & 0xFFFF;
            wr32(base, RDMA_RING_BASE + DMA_CONS_INDEX, self.c_index as u32);
            self.rx_index = (self.rx_index + 1) % TOTAL_DESCS;
        }
    }

    pub fn rx_pending(&self) -> usize {
        self.rx_queue.len()
    }

    pub fn pop_rx(&mut self) -> Option<Vec<u8>> {
        if self.rx_queue.is_empty() {
            None
        } else {
            Some(self.rx_queue.remove(0))
        }
    }

    pub fn mac(&self) -> [u8; 6] {
        self.mac
    }

    pub fn max_speed(&self) -> u32 {
        self.max_speed
    }
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum MdioStatus {
    Ok,
    Timeout,
    Fail,
}

impl MdioStatus {
    fn as_str(self) -> &'static str {
        match self {
            MdioStatus::Ok => "ok",
            MdioStatus::Timeout => "timeout",
            MdioStatus::Fail => "fail",
        }
    }
}

// MDIO helpers (free functions using base)
fn mdio_read_status(base: usize, phy: u32, reg: u32) -> (u16, MdioStatus) {
    // U-Boot: write cmd, then rewrite with START_BUSY set in one value.
    let cmd = MDIO_RD | (phy << MDIO_PMD_SHIFT) | (reg << MDIO_REG_SHIFT);
    wr32(base, MDIO_CMD, cmd);
    wr32(base, MDIO_CMD, cmd | MDIO_START_BUSY);
    let deadline = Instant::now() + Duration::from_millis(100);
    while rd32(base, MDIO_CMD) & MDIO_START_BUSY != 0 {
        if Instant::now() > deadline {
            return (0, MdioStatus::Timeout);
        }
        thread::sleep(Duration::from_micros(100));
    }
    let val = rd32(base, MDIO_CMD);
    if val & MDIO_READ_FAIL != 0 {
        return (0, MdioStatus::Fail);
    }
    ((val & 0xFFFF) as u16, MdioStatus::Ok)
}

fn mdio_read(base: usize, phy: u32, reg: u32) -> u16 {
    mdio_read_status(base, phy, reg).0
}

fn mdio_write(base: usize, phy: u32, reg: u32, val: u16) {
    let cmd =
        MDIO_WR | (phy << MDIO_PMD_SHIFT) | (reg << MDIO_REG_SHIFT) | (val as u32 & 0xFFFF);
    wr32(base, MDIO_CMD, cmd);
    wr32(base, MDIO_CMD, cmd | MDIO_START_BUSY);
    let deadline = Instant::now() + Duration::from_millis(100);
    while rd32(base, MDIO_CMD) & MDIO_START_BUSY != 0 {
        if Instant::now() > deadline {
            break;
        }
        thread::sleep(Duration::from_micros(100));
    }
}

// FDT helpers (free functions, used by main.rs)
pub fn translate(fdt: &Fdt, child_addr: usize) -> usize {
    for bus in ["/scb", "/soc", "/emmc2bus"] {
        if let Some(node) = fdt.find_node(bus) {
            if let Some(mut ranges) = node.ranges() {
                for r in ranges {
                    if r.child_bus_address <= child_addr && child_addr - r.child_bus_address < r.size {
                        return r.parent_bus_address + (child_addr - r.child_bus_address);
                    }
                }
            }
        }
    }
    child_addr
}

pub fn find_phy(fdt: &Fdt, genet: &fdt::node::FdtNode) -> (u32, u32) {
    for child in genet.children() {
        let name = child.name;
        let compat = child.property("compatible").and_then(|p| p.as_str()).unwrap_or("");
        if name.contains("mdio") || compat.contains("mdio") {
            for gc in child.children() {
                if let Some(mut reg) = gc.reg() {
                    if let Some(r) = reg.next() {
                        let addr = r.starting_address as u32;
                        let max_speed = gc.property("max-speed").and_then(|p| p.as_usize()).unwrap_or(0) as u32;
                        return (addr, max_speed);
                    }
                }
            }
        }
    }
    (1, 0)
}
