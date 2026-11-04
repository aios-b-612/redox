//! SDHCI host controller driver (standard register set, PIO mode).
//!
//! Used for the BCM2711 EMMC2 controller (compatible `brcm,bcm2711-emmc2`) that
//! drives the microSD slot of the Raspberry Pi 4. The Pi 3 controller in
//! `sd/mod.rs` uses the proprietary BCM2835 layout; the EMMC2 is a standard
//! Arasan SDHCI core, so it needs the standard register interface instead.
//!
//! Notes for the BCM2711 EMMC2 (mirrors Linux `sdhci-iproc.c`):
//! * All register accesses must be 32-bit, so 8/16-bit registers are written
//!   with a read-modify-write of their containing 32-bit word.
//! * Successive 16-bit writes to the same 32-bit word can be lost (clock domain
//!   crossing bug in the Arasan core), therefore BLOCK_SIZE/BLOCK_COUNT and
//!   TRANSFER_MODE/COMMAND are committed with two single 32-bit writes right
//!   before the command is issued (`shadow_block`/`shadow_command` below).
//! * Card detection is broken (`broken-cd` in the DT): the card is assumed to be
//!   present all the time.
//! * No DMA: data is transferred through the buffer data port (PIO), the same
//!   strategy Linux uses when DMA is not usable.
//! * Multi-block reads use auto CMD12 (`SDHCI_QUIRK_MULTIBLOCK_READ_ACMD12`).
//!   Writes are issued one block at a time (CMD24): no stop command is needed
//!   and writes are rare in this system.

#![allow(dead_code)]

use core::ptr::{read_volatile, write_volatile};

use driver_block::Disk;
use syscall::{Error, Result, EINVAL};

use crate::sd::wait_msec;

// Register offsets
const SDHCI_BLOCK_SIZE: usize = 0x04;
const SDHCI_BLOCK_COUNT: usize = 0x06;
const SDHCI_ARGUMENT: usize = 0x08;
const SDHCI_TRANSFER_MODE: usize = 0x0C;
const SDHCI_COMMAND: usize = 0x0E;
const SDHCI_RESPONSE: usize = 0x10;
const SDHCI_BUFFER_DATA_PORT: usize = 0x20;
const SDHCI_PRESENT_STATE: usize = 0x24;
const SDHCI_HOST_CONTROL: usize = 0x28;
const SDHCI_POWER_CONTROL: usize = 0x29;
const SDHCI_BLOCK_GAP_CONTROL: usize = 0x2A;
const SDHCI_CLOCK_CONTROL: usize = 0x2C;
const SDHCI_TIMEOUT_CONTROL: usize = 0x2E;
const SDHCI_SOFTWARE_RESET: usize = 0x2F;
const SDHCI_NORMAL_INT_STATUS: usize = 0x30;
const SDHCI_ERROR_INT_STATUS: usize = 0x32;
const SDHCI_NORMAL_INT_STATUS_ENABLE: usize = 0x34;
const SDHCI_ERROR_INT_STATUS_ENABLE: usize = 0x36;
const SDHCI_NORMAL_INT_SIGNAL_ENABLE: usize = 0x38;
const SDHCI_ERROR_INT_SIGNAL_ENABLE: usize = 0x3A;
const SDHCI_AUTO_CMD12_ERROR_STATUS: usize = 0x3C;
const SDHCI_CAPABILITIES: usize = 0x40;
const SDHCI_SLOT_INT_STATUS: usize = 0xFC;
const SDHCI_HOST_VERSION: usize = 0xFE;

// Command register: the command index goes in bits 13:8
const CMD_RESP_NONE: u16 = 0x0000;
const CMD_RESP_LONG: u16 = 0x0001; // R2 (136 bits)
const CMD_RESP_SHORT: u16 = 0x0002; // R1/R3/R6/R7 (48 bits)
const CMD_RESP_SHORT_BUSY: u16 = 0x0003; // R1b/R5b
const CMD_CRC_CHECK: u16 = 0x0008;
const CMD_INDEX_CHECK: u16 = 0x0010;
const CMD_DATA_PRESENT: u16 = 0x0020;

#[inline(always)]
fn make_command(index: u8, flags: u16) -> u16 {
    ((index as u16) << 8) | (flags & 0xFF)
}

// Transfer mode register
const TRNS_DMA_ENABLE: u16 = 0x0001;
const TRNS_BLOCK_COUNT_ENABLE: u16 = 0x0002;
const TRNS_AUTO_CMD12_ENABLE: u16 = 0x0004;
const TRNS_READ: u16 = 0x0010;
const TRNS_MULTI_BLOCK: u16 = 0x0020;

// Present state register
const PRESENT_CMD_INHIBIT: u32 = 0x0000_0001;
const PRESENT_DATA_INHIBIT: u32 = 0x0000_0002;
const PRESENT_SPACE_AVAILABLE: u32 = 0x0000_0400; // buffer can accept data (write)
const PRESENT_DATA_AVAILABLE: u32 = 0x0000_0800; // buffer has data to read

// Normal interrupt status/enable bits (W1C)
const INT_CMD_COMPLETE: u16 = 0x0001;
const INT_XFER_COMPLETE: u16 = 0x0002;
const INT_BUF_WRITE_READY: u16 = 0x0010;
const INT_BUF_READ_READY: u16 = 0x0020;

// Error interrupt status/enable bits (W1C)
const ERR_CMD_TIMEOUT: u16 = 0x0001;
const ERR_CMD_CRC: u16 = 0x0002;
const ERR_CMD_END_BIT: u16 = 0x0004;
const ERR_CMD_INDEX: u16 = 0x0008;
const ERR_DATA_TIMEOUT: u16 = 0x0010;
const ERR_DATA_CRC: u16 = 0x0020;
const ERR_DATA_END_BIT: u16 = 0x0040;
const ERR_CURRENT_LIMIT: u16 = 0x0080;
const ERR_AUTO_CMD12: u16 = 0x0100;
const ERR_ADMA: u16 = 0x0200;

const ERR_MASK: u16 = ERR_CMD_TIMEOUT
    | ERR_CMD_CRC
    | ERR_CMD_END_BIT
    | ERR_CMD_INDEX
    | ERR_DATA_TIMEOUT
    | ERR_DATA_CRC
    | ERR_DATA_END_BIT
    | ERR_CURRENT_LIMIT
    | ERR_AUTO_CMD12
    | ERR_ADMA;

// Clock control
const CLOCK_INTERNAL_ENABLE: u16 = 0x0001;
const CLOCK_INTERNAL_STABLE: u16 = 0x0002;
const CLOCK_SDCLK_ENABLE: u16 = 0x0004;

// Host control
const HOST_CONTROL_4BIT: u8 = 0x02;

// Software reset
const RESET_ALL: u8 = 0x01;
const RESET_CMD_LINE: u8 = 0x02;
const RESET_DATA_LINE: u8 = 0x04;

// SD command indexes used during card initialization
const SD_GO_IDLE_STATE: u8 = 0;
const SD_ALL_SEND_CID: u8 = 2;
const SD_SEND_RELATIVE_ADDR: u8 = 3;
const SD_SET_BUS_WIDTH: u8 = 6;
const SD_SELECT_CARD: u8 = 7;
const SD_SEND_IF_COND: u8 = 8;
const SD_SEND_CSD: u8 = 9;
const SD_STOP_TRANSMISSION: u8 = 12;
const SD_SET_BLOCKLEN: u8 = 16;
const SD_READ_SINGLE_BLOCK: u8 = 17;
const SD_READ_MULTIPLE_BLOCK: u8 = 18;
const SD_WRITE_BLOCK: u8 = 24;
const SD_APP_CMD: u8 = 55;
const SD_SEND_OP_COND: u8 = 41;
const SD_SET_BLOCK_COUNT: u8 = 23;

const SD_BLOCK_SIZE: u32 = 512;

// ACMD41 argument: HCS (SDHC/SDXC support) + 3.2-3.3V/3.3V voltage window
const ACMD41_ARG: u32 = 0x40FF_8000;
const ACMD41_BUSY: u32 = 0x8000_0000;
const ACMD41_CCS: u32 = 0x4000_0000;

pub struct SdhciCtrl {
    base: usize,
    base_clock: u32,
    rca: u32,
    high_capacity: bool,
    size: u64,
    /// BLOCK_SIZE (bits 15:0) | BLOCK_COUNT (bits 31:16)
    shadow_block: u32,
    /// TRANSFER_MODE (bits 15:0) | COMMAND (bits 31:16)
    shadow_command: u32,
}

impl SdhciCtrl {
    pub fn new(base: usize, base_clock: u32) -> Self {
        Self {
            base,
            base_clock,
            rca: 0,
            high_capacity: false,
            size: 0,
            shadow_block: 0,
            shadow_command: 0,
        }
    }

    #[inline(always)]
    unsafe fn read32(&self, offset: usize) -> u32 {
        read_volatile((self.base + offset) as *const u32)
    }

    #[inline(always)]
    unsafe fn write32(&self, offset: usize, value: u32) {
        write_volatile((self.base + offset) as *mut u32, value)
    }

    #[inline(always)]
    unsafe fn read16(&self, offset: usize) -> u16 {
        let word = self.read32(offset & !3);
        (word >> ((offset & 3) * 8)) as u16
    }

    #[inline(always)]
    unsafe fn write16(&self, offset: usize, value: u16) {
        let shift = (offset & 3) * 8;
        let mask = 0xffffu32 << shift;
        let word = self.read32(offset & !3);
        self.write32(offset & !3, (word & !mask) | ((value as u32) << shift));
    }

    #[inline(always)]
    unsafe fn read8(&self, offset: usize) -> u8 {
        let word = self.read32(offset & !3);
        (word >> ((offset & 3) * 8)) as u8
    }

    #[inline(always)]
    unsafe fn write8(&self, offset: usize, value: u8) {
        let shift = (offset & 3) * 8;
        let mask = 0xffu32 << shift;
        let word = self.read32(offset & !3);
        self.write32(offset & !3, (word & !mask) | ((value as u32) << shift));
    }

    /// Poll a register until `value & mask == wanted`.
    unsafe fn wait_bits(&self, offset: usize, mask: u32, wanted: u32, timeout_ms: u32) -> bool {
        let mut timeout = timeout_ms;
        loop {
            if (self.read32(offset) & mask) == wanted {
                return true;
            }
            if timeout == 0 {
                return false;
            }
            timeout -= 1;
            wait_msec(1);
        }
    }

    unsafe fn clear_interrupts(&self) {
        self.write16(SDHCI_NORMAL_INT_STATUS, 0xFFFF);
        self.write16(SDHCI_ERROR_INT_STATUS, 0xFFFF);
    }

    unsafe fn check_error(&self) -> Result<()> {
        let error = self.read16(SDHCI_ERROR_INT_STATUS);
        if error & ERR_MASK != 0 {
            self.write16(SDHCI_ERROR_INT_STATUS, 0xFFFF);
            self.write16(SDHCI_NORMAL_INT_STATUS, 0xFFFF);
            println!("emmc2: error interrupt status 0x{:04x}", error);
            return Err(Error::new(EINVAL));
        }
        Ok(())
    }

    unsafe fn wait_interrupt(&self, bit: u16, timeout_ms: u32) -> Result<()> {
        let mut timeout = timeout_ms;
        loop {
            self.check_error()?;
            if self.read16(SDHCI_NORMAL_INT_STATUS) & bit != 0 {
                self.write16(SDHCI_NORMAL_INT_STATUS, bit);
                return Ok(());
            }
            if timeout == 0 {
                println!("emmc2: timeout waiting for interrupt 0x{:04x}", bit);
                return Err(Error::new(EINVAL));
            }
            timeout -= 1;
            wait_msec(1);
        }
    }

    unsafe fn reset(&mut self, mask: u8) {
        self.write8(SDHCI_SOFTWARE_RESET, mask);
        // The reset bits are in a byte register, so poll it directly
        let mut timeout = 100;
        while self.read8(SDHCI_SOFTWARE_RESET) & mask != 0 {
            if timeout == 0 {
                println!("emmc2: reset 0x{:02x} did not complete", mask);
                break;
            }
            timeout -= 1;
            wait_msec(1);
        }
        if mask & RESET_ALL != 0 {
            self.clear_interrupts();
        }
    }

    /// Reset the command and data lines after a failed command.
    unsafe fn reset_lines(&mut self) {
        self.reset(RESET_CMD_LINE | RESET_DATA_LINE);
    }

    unsafe fn set_power(&mut self, on: bool) {
        if on {
            // SD bus power on + 3.3V
            self.write8(SDHCI_POWER_CONTROL, 0x0F);
            wait_msec(10);
        } else {
            self.write8(SDHCI_POWER_CONTROL, 0x00);
        }
    }

    /// Set the SD clock. SDCLK = base clock / (2 * divider).
    unsafe fn set_clock(&mut self, freq: u32) -> Result<()> {
        // Disable the SD clock first
        let mut clock = self.read16(SDHCI_CLOCK_CONTROL);
        clock &= !(CLOCK_SDCLK_ENABLE | CLOCK_INTERNAL_ENABLE);
        clock &= !0xFF00;
        self.write16(SDHCI_CLOCK_CONTROL, clock);
        wait_msec(1);

        let mut divider = (self.base_clock + 2 * freq - 1) / (2 * freq);
        if divider < 1 {
            divider = 1;
        }
        if divider > 0xFF {
            // 8-bit divider; the BCM2711 floors the SD clock at 200 kHz and the
            // divider maxes out at 196 kHz, close enough for identification
            divider = 0xFF;
        }
        clock |= ((divider as u16) & 0xFF) << 8;
        clock |= CLOCK_INTERNAL_ENABLE;
        self.write16(SDHCI_CLOCK_CONTROL, clock);

        // Wait for the internal clock to become stable
        let mut timeout = 100;
        while self.read16(SDHCI_CLOCK_CONTROL) & CLOCK_INTERNAL_STABLE == 0 {
            if timeout == 0 {
                println!("emmc2: internal clock never became stable");
                break;
            }
            timeout -= 1;
            wait_msec(1);
        }

        self.write16(SDHCI_CLOCK_CONTROL, clock | CLOCK_SDCLK_ENABLE);
        Ok(())
    }

    unsafe fn set_bus_width(&mut self, four_bit: bool) {
        let mut host_control = self.read8(SDHCI_HOST_CONTROL);
        if four_bit {
            host_control |= HOST_CONTROL_4BIT;
        } else {
            host_control &= !HOST_CONTROL_4BIT;
        }
        self.write8(SDHCI_HOST_CONTROL, host_control);
    }

    unsafe fn wait_idle(&self) -> Result<()> {
        let mask = PRESENT_CMD_INHIBIT | PRESENT_DATA_INHIBIT;
        if !self.wait_bits(SDHCI_PRESENT_STATE, mask, 0, 1000) {
            println!("emmc2: controller stuck busy (present state 0x{:08x})", self.read32(SDHCI_PRESENT_STATE));
            return Err(Error::new(EINVAL));
        }
        Ok(())
    }

    /// Commit the shadowed block info and command with single 32-bit writes.
    /// This is the Arasan workaround: BLOCK_SIZE+BLOCK_COUNT in one write,
    /// TRANSFER_MODE+COMMAND in the next one.
    unsafe fn issue_command(&mut self) -> Result<()> {
        self.write32(SDHCI_BLOCK_SIZE, self.shadow_block);
        self.write32(SDHCI_TRANSFER_MODE, self.shadow_command);
        self.wait_interrupt(INT_CMD_COMPLETE, 1000)
    }

    /// Send a command without data transfer. Returns RESPONSE0.
    unsafe fn send_command(&mut self, index: u8, argument: u32, flags: u16) -> Result<u32> {
        self.wait_idle()?;
        self.clear_interrupts();
        self.write32(SDHCI_ARGUMENT, argument);
        self.shadow_block = 0;
        self.shadow_command = (make_command(index, flags) as u32) << 16;
        if let Err(err) = self.issue_command() {
            self.reset_lines();
            return Err(err);
        }
        Ok(self.read32(SDHCI_RESPONSE))
    }

    /// Read a 136-bit (R2) response. The controller strips the CRC byte, so the
    /// response has to be shifted back (same as Linux `sdhci_read_rsp_136`).
    unsafe fn read_response_136(&self) -> [u8; 16] {
        let r0 = self.read32(SDHCI_RESPONSE);
        let r1 = self.read32(SDHCI_RESPONSE + 4);
        let r2 = self.read32(SDHCI_RESPONSE + 8);
        let r3 = self.read32(SDHCI_RESPONSE + 12);

        let w0 = (r3 << 8) | (r2 >> 24);
        let w1 = (r2 << 8) | (r1 >> 24);
        let w2 = (r1 << 8) | (r0 >> 24);
        let w3 = r0 << 8;

        let mut response = [0u8; 16];
        response[0..4].copy_from_slice(&w0.to_be_bytes());
        response[4..8].copy_from_slice(&w1.to_be_bytes());
        response[8..12].copy_from_slice(&w2.to_be_bytes());
        response[12..16].copy_from_slice(&w3.to_be_bytes());
        response
    }

    /// Transfer `blocks` blocks of 512 bytes in PIO mode.
    unsafe fn transfer_blocks(
        &mut self,
        index: u8,
        lba: u64,
        blocks: u32,
        read: bool,
        buffer: &mut [u8],
    ) -> Result<()> {
        let mut result = self.transfer_blocks_inner(index, lba, blocks, read, buffer);
        if result.is_err() {
            // AIOS: one retry after resetting the lines. A slow card can miss a
            // single transfer (internal garbage collection) and then work fine;
            // failing the whole request would abort a filesystem mount.
            self.reset_lines();
            println!(
                "emmc2: retrying {} {} block(s) at lba {}",
                if read { "read" } else { "write" },
                blocks,
                lba
            );
            wait_msec(10);
            result = self.transfer_blocks_inner(index, lba, blocks, read, buffer);
            if result.is_err() {
                self.reset_lines();
                println!(
                    "emmc2: {} of {} block(s) at lba {} failed twice",
                    if read { "read" } else { "write" },
                    blocks,
                    lba
                );
            }
        }
        result
    }

    unsafe fn transfer_blocks_inner(
        &mut self,
        index: u8,
        lba: u64,
        blocks: u32,
        read: bool,
        buffer: &mut [u8],
    ) -> Result<()> {
        let length = blocks as usize * SD_BLOCK_SIZE as usize;
        if buffer.len() != length {
            println!("emmc2: buffer length {} != {} bytes", buffer.len(), length);
            return Err(Error::new(EINVAL));
        }

        // High capacity cards (SDHC/SDXC) address 512 byte blocks, standard
        // capacity cards address bytes.
        let argument = if self.high_capacity {
            lba
        } else {
            lba.checked_mul(SD_BLOCK_SIZE as u64)
                .ok_or_else(|| Error::new(EINVAL))?
        };
        if argument > u32::MAX as u64 {
            println!("emmc2: block address {} is out of range", argument);
            return Err(Error::new(EINVAL));
        }

        self.wait_idle()?;
        self.clear_interrupts();
        self.write32(SDHCI_ARGUMENT, argument as u32);

        self.shadow_block = (SD_BLOCK_SIZE & 0xFFF) | (blocks << 16);

        let mut mode = 0u16;
        if read {
            mode |= TRNS_READ;
        }
        if blocks > 1 {
            mode |= TRNS_BLOCK_COUNT_ENABLE | TRNS_MULTI_BLOCK;
            if read {
                // The BCM2711 supports auto CMD12 for multi-block reads
                mode |= TRNS_AUTO_CMD12_ENABLE;
            }
        }
        let flags = CMD_DATA_PRESENT | CMD_RESP_SHORT;
        self.shadow_command = (mode as u32) | ((make_command(index, flags) as u32) << 16);

        self.issue_command()?;

        // Move the data block by block through the buffer data port.
        //
        // AIOS: wait for "Buffer Read/Write Ready" once per block, which the
        // controller only raises when a WHOLE block fits in its internal
        // buffer. The present-state bits (Data/Space Available) only mean that
        // a few bytes are in the buffer: reading 512 bytes after seeing them
        // returns whatever the buffer happens to hold, which corrupted data
        // silently on real hardware (QEMU is instantaneous, so it never
        // showed there).
        let buffer_ready_bit = if read {
            INT_BUF_READ_READY
        } else {
            INT_BUF_WRITE_READY
        };

        for block in 0..blocks as usize {
            self.wait_interrupt(buffer_ready_bit, 5000)?;
            self.check_error()?;

            let offset = block * SD_BLOCK_SIZE as usize;
            let data = &mut buffer[offset..offset + SD_BLOCK_SIZE as usize];
            if read {
                for chunk in data.chunks_exact_mut(4) {
                    let word = self.read32(SDHCI_BUFFER_DATA_PORT);
                    chunk.copy_from_slice(&word.to_le_bytes());
                }
            } else {
                for chunk in data.chunks_exact(4) {
                    let word = u32::from_le_bytes([chunk[0], chunk[1], chunk[2], chunk[3]]);
                    self.write32(SDHCI_BUFFER_DATA_PORT, word);
                }
            }
        }

        // Slow cards (internal garbage collection) can take a while to
        // complete a transfer.
        self.wait_interrupt(INT_XFER_COMPLETE, 5000)?;

        // Let a possible auto CMD12 finish before the next command
        self.wait_idle()?;

        let auto_cmd12_error = self.read16(SDHCI_AUTO_CMD12_ERROR_STATUS);
        if auto_cmd12_error != 0 {
            println!("emmc2: auto CMD12 error status 0x{:04x}", auto_cmd12_error);
            self.write16(SDHCI_AUTO_CMD12_ERROR_STATUS, 0xFFFF);
            self.check_error()?;
        }

        Ok(())
    }

    /// Initialize the controller itself (not the card).
    pub unsafe fn init(&mut self) -> Result<()> {
        let version = self.read16(SDHCI_HOST_VERSION);
        let capabilities = self.read32(SDHCI_CAPABILITIES);
        println!(
            "emmc2: host version 0x{:04x}, capabilities 0x{:08x}, base clock {} Hz",
            version, capabilities, self.base_clock
        );

        self.reset(RESET_ALL);
        wait_msec(10);

        self.set_power(true);

        // Maximum timeout, maximum timeout for the data line
        self.write8(SDHCI_TIMEOUT_CONTROL, 0x0E);
        self.write8(SDHCI_BLOCK_GAP_CONTROL, 0x00);

        // The card is soldered to the board as far as the controller knows
        // (broken card detection): do not wait for card insert events.
        // Enable all status bits but keep the interrupt signal disabled: this
        // driver polls.
        self.write16(SDHCI_NORMAL_INT_STATUS_ENABLE, 0xFFFF);
        self.write16(SDHCI_ERROR_INT_STATUS_ENABLE, 0xFFFF);
        self.write16(SDHCI_NORMAL_INT_SIGNAL_ENABLE, 0x0000);
        self.write16(SDHCI_ERROR_INT_SIGNAL_ENABLE, 0x0000);
        self.clear_interrupts();

        // Identification clock: 400 kHz
        self.set_clock(400_000)?;

        self.init_card()?;

        Ok(())
    }

    /// Bring the SD card to the transfer state and read its capacity.
    unsafe fn init_card(&mut self) -> Result<()> {
        // CMD0: go idle
        self.send_command(SD_GO_IDLE_STATE, 0, CMD_RESP_NONE)?;

        // CMD8: interface condition (only SD 2.0+ cards answer)
        let mut high_capacity = false;
        match self.send_command(
            SD_SEND_IF_COND,
            0x1AA,
            CMD_RESP_SHORT | CMD_CRC_CHECK | CMD_INDEX_CHECK,
        ) {
            Ok(response) => {
                if response & 0xFFF != 0x1AA {
                    println!("emmc2: CMD8 returned unexpected 0x{:08x}", response);
                }
                // ACMD41: send operating conditions until the card is ready
                let mut ready = false;
                for _ in 0..100 {
                    self.send_command(SD_APP_CMD, 0, CMD_RESP_SHORT)?;
                    let ocr = self.send_command(SD_SEND_OP_COND, ACMD41_ARG, CMD_RESP_SHORT)?;
                    if ocr & ACMD41_BUSY != 0 {
                        high_capacity = ocr & ACMD41_CCS != 0;
                        ready = true;
                        break;
                    }
                    wait_msec(10);
                }
                if !ready {
                    println!("emmc2: card never became ready");
                    return Err(Error::new(EINVAL));
                }
            }
            Err(_) => {
                println!("emmc2: CMD8 failed, assuming an SD 1.x card");
                // A card that does not answer CMD8 is an SD 1.x card: no high
                // capacity support and no CRC check on ACMD41.
                let mut ready = false;
                for _ in 0..100 {
                    self.send_command(SD_APP_CMD, 0, CMD_RESP_SHORT)?;
                    let ocr = self.send_command(SD_SEND_OP_COND, 0x00FF_8000, CMD_RESP_SHORT)?;
                    if ocr & ACMD41_BUSY != 0 {
                        ready = true;
                        break;
                    }
                    wait_msec(10);
                }
                if !ready {
                    println!("emmc2: card never became ready");
                    return Err(Error::new(EINVAL));
                }
            }
        }

        self.high_capacity = high_capacity;

        // CMD2: ask for the CID
        self.send_command(SD_ALL_SEND_CID, 0, CMD_RESP_LONG)?;
        let _cid = self.read_response_136();

        // CMD3: ask for a relative card address
        let response = self.send_command(SD_SEND_RELATIVE_ADDR, 0, CMD_RESP_SHORT)?;
        self.rca = response & 0xFFFF_0000;
        println!(
            "emmc2: RCA 0x{:08x}, {} capacity",
            self.rca,
            if self.high_capacity { "high" } else { "standard" }
        );

        // CMD9: read the card specific data (capacity)
        self.send_command(SD_SEND_CSD, self.rca, CMD_RESP_LONG)?;
        let csd = self.read_response_136();
        self.size = Self::capacity_from_csd(&csd, self.high_capacity);
        println!(
            "emmc2: card size {} MiB ({} bytes)",
            self.size / 1024 / 1024,
            self.size
        );

        // CMD7: select the card
        self.send_command(SD_SELECT_CARD, self.rca, CMD_RESP_SHORT_BUSY | CMD_CRC_CHECK)?;

        // ACMD6: 4-bit bus, then tell the controller about it
        self.send_command(SD_APP_CMD, self.rca, CMD_RESP_SHORT)?;
        self.send_command(SD_SET_BUS_WIDTH, 2, CMD_RESP_SHORT)?;
        self.set_bus_width(true);

        // CMD16: 512 byte blocks
        self.send_command(SD_SET_BLOCKLEN, SD_BLOCK_SIZE, CMD_RESP_SHORT)?;

        // Transfer clock: 25 MHz (default speed)
        self.set_clock(25_000_000)?;

        Ok(())
    }

    /// Capacity in bytes from the CSD register (SD 2.0/3.0 layout).
    fn capacity_from_csd(csd: &[u8; 16], high_capacity: bool) -> u64 {
        if high_capacity {
            // CSD v2: C_SIZE in bits 69:48, 512 KiB units
            let c_size = ((csd[7] as u64 & 0x3F) << 16) | ((csd[8] as u64) << 8) | csd[9] as u64;
            (c_size + 1) * 512 * 1024
        } else {
            // CSD v1: C_SIZE in bits 73:62, C_SIZE_MULT in bits 49:47,
            // READ_BL_LEN in bits 83:80
            let c_size = ((csd[6] as u64 & 0x03) << 10) | ((csd[7] as u64) << 2) | ((csd[8] as u64) >> 6);
            let c_mult = ((csd[9] as u64 & 0x03) << 1) | ((csd[10] as u64) >> 7);
            let read_bl_len = (csd[5] as u64) & 0x0F;
            (c_size + 1) << (c_mult + 2 + read_bl_len)
        }
    }

    /// Read `buffer.len() / 512` blocks starting at `block`.
    pub unsafe fn read_blocks(&mut self, block: u64, buffer: &mut [u8]) -> Result<usize> {
        if buffer.is_empty() || buffer.len() % SD_BLOCK_SIZE as usize != 0 {
            println!("emmc2: read length {} is not a multiple of 512", buffer.len());
            return Err(Error::new(EINVAL));
        }

        // Multi-block commands address at most 65535 blocks (16-bit count):
        // transfer bigger requests in chunks.
        const MAX_BLOCKS: usize = 256;

        let mut lba = block;
        for chunk in buffer.chunks_mut(MAX_BLOCKS * SD_BLOCK_SIZE as usize) {
            let blocks = (chunk.len() / SD_BLOCK_SIZE as usize) as u32;
            let index = if blocks > 1 {
                SD_READ_MULTIPLE_BLOCK
            } else {
                SD_READ_SINGLE_BLOCK
            };
            self.transfer_blocks(index, lba, blocks, true, chunk)?;
            lba += blocks as u64;
        }

        Ok(buffer.len())
    }

    /// Write `buffer.len() / 512` blocks starting at `block`.
    pub unsafe fn write_blocks(&mut self, block: u64, buffer: &[u8]) -> Result<usize> {
        if buffer.is_empty() || buffer.len() % SD_BLOCK_SIZE as usize != 0 {
            println!("emmc2: write length {} is not a multiple of 512", buffer.len());
            return Err(Error::new(EINVAL));
        }

        // One block at a time: a multi-block write would need a stop command
        // (CMD12) at the end, and writes are not on the hot path here.
        let mut data = buffer;
        let mut lba = block;
        while !data.is_empty() {
            let (head, tail) = data.split_at(SD_BLOCK_SIZE as usize);
            let mut block_buffer = [0u8; SD_BLOCK_SIZE as usize];
            block_buffer.copy_from_slice(head);
            self.transfer_blocks(SD_WRITE_BLOCK, lba, 1, false, &mut block_buffer)?;
            data = tail;
            lba += 1;
        }
        Ok(buffer.len())
    }
}

impl Disk for SdhciCtrl {
    fn block_size(&self) -> u32 {
        SD_BLOCK_SIZE
    }

    fn size(&self) -> u64 {
        self.size
    }

    async fn read(&mut self, block: u64, buffer: &mut [u8]) -> Result<usize> {
        unsafe { self.read_blocks(block, buffer) }
    }

    async fn write(&mut self, block: u64, buffer: &[u8]) -> Result<usize> {
        unsafe { self.write_blocks(block, buffer) }
    }
}
