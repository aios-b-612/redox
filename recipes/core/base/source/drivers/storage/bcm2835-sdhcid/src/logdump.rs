//! AIOS: early boot log dumper.
//!
//! Writes the kernel log ring to raw sectors past the GPT (LBA [`LOG_LBA`]),
//! readable on the PC with:
//! `dd if=/dev/sdX bs=512 skip=<LOG_LBA> count=257`.
//!
//! 2026-11-01 hardware findings:
//! - Holding the SDHCI mutex for a multi-block dump during switchroot coincided
//!   with a `redoxfs` abort (FAR=0x40). Large dumps therefore yield the lock
//!   between blocks.
//! - After shipping a 20s "mount quiet", a hanging Pi left the log LBA empty:
//!   the dumper never persisted anything. Marker write must be immediate and
//!   use the same single-block `write_blocks` path that already worked on HW.

use std::io::Read;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use crate::sdhci::SdhciCtrl;

/// 64 MiB after the end of the image (image GPT ends at LBA 4194270).
pub const LOG_LBA: u64 = 4_259_840;

const BLOCK: usize = 512;
const MAX_DATA: usize = 128 * 1024;
const HEAD_BYTES: usize = 64 * 1024;

/// Pause so redoxfs can mount before the first multi-block dump.
/// 3s was too short on Pi 4 HW (redoxfs data-abort / Partition(0,2) drop →
/// total ENOENT after switchroot). Markers stay immediate; only full dumps wait.
const MOUNT_QUIET: Duration = Duration::from_secs(20);
const FAST_INTERVAL: Duration = Duration::from_secs(5);
const FAST_FOR: Duration = Duration::from_secs(120);
const SLOW_INTERVAL: Duration = Duration::from_secs(30);
const SUCCESS_EVERY: u64 = 15;

/// One-block write under a single lock — the path proven on Pi 4 hardware.
fn write_one_block(ctrl: &Arc<Mutex<SdhciCtrl>>, lba: u64, block: &[u8; BLOCK]) -> std::io::Result<()> {
    let mut c = ctrl
        .lock()
        .map_err(|_| std::io::Error::other("poisoned"))?;
    unsafe {
        c.write_blocks(lba, block)
            .map_err(|e| std::io::Error::other(format!("write_blocks: {e:?}")))?;
    }
    Ok(())
}

/// Multi-block dump: release the mutex between blocks so DiskScheme can run.
fn write_blocks_yielding(
    ctrl: &Arc<Mutex<SdhciCtrl>>,
    start_lba: u64,
    buf: &[u8],
) -> std::io::Result<usize> {
    if buf.len() % BLOCK != 0 {
        return Err(std::io::Error::other("buffer not multiple of 512"));
    }
    let mut lba = start_lba;
    for chunk in buf.chunks(BLOCK) {
        let mut block = [0u8; BLOCK];
        block.copy_from_slice(chunk);
        write_one_block(ctrl, lba, &block)?;
        lba += 1;
    }
    Ok(buf.len())
}

fn dump(
    ctrl: &Arc<Mutex<SdhciCtrl>>,
    seq: u64,
    elapsed_ms: u128,
    payload: &[u8],
    head_len: usize,
    original: usize,
) -> std::io::Result<usize> {
    let mut buf = Vec::with_capacity(BLOCK + payload.len() + BLOCK);
    let mut header = Vec::with_capacity(BLOCK);
    header.extend_from_slice(b"AIOS-SDHCI-LOGDUMP v1\n");
    header.extend_from_slice(
        format!(
            "seq={} elapsed_ms={} len={} truncated={} head={} tail={}\n",
            seq,
            elapsed_ms,
            payload.len(),
            original > MAX_DATA,
            head_len,
            payload.len() - head_len
        )
        .as_bytes(),
    );
    header.resize(BLOCK, 0);
    buf.extend_from_slice(&header);
    buf.extend_from_slice(payload);
    buf.resize(buf.len().div_ceil(BLOCK) * BLOCK, 0);
    write_blocks_yielding(ctrl, LOG_LBA, &buf)
}

fn write_boot_marker(ctrl: &Arc<Mutex<SdhciCtrl>>, tag: &str) -> std::io::Result<()> {
    let mut block = [0u8; BLOCK];
    let msg = format!("AIOS-SDHCI-LOGDUMP v1\nseq=0 marker={}\n", tag);
    let bytes = msg.as_bytes();
    let n = bytes.len().min(BLOCK);
    block[..n].copy_from_slice(&bytes[..n]);
    write_one_block(ctrl, LOG_LBA, &block)
}

fn collect() -> std::io::Result<(Vec<u8>, usize, usize)> {
    let mut file = std::fs::File::open("/scheme/sys/log")?;
    let mut log = Vec::with_capacity(MAX_DATA);
    file.read_to_end(&mut log)?;
    let original = log.len();
    if original <= MAX_DATA {
        return Ok((log, original, original));
    }
    let mut payload = Vec::with_capacity(MAX_DATA);
    payload.extend_from_slice(&log[..HEAD_BYTES]);
    payload.extend_from_slice(&log[original - (MAX_DATA - HEAD_BYTES)..]);
    Ok((payload, HEAD_BYTES, original))
}

pub fn spawn(ctrl: Arc<Mutex<SdhciCtrl>>) {
    std::thread::spawn(move || {
        let t0 = Instant::now();
        let mut seq = 0u64;
        let mut last_err_print: Option<Instant> = None;
        let mut last_payload: Option<Vec<u8>> = None;

        // Immediate marker (single-block path). If this fails, we still try
        // again after a short sleep so a transient busy does not silence us.
        for (i, tag) in ["boot-0", "boot-1"].iter().enumerate() {
            match write_boot_marker(&ctrl, tag) {
                Ok(()) => println!("logdump: marker {} ok at LBA {}", tag, LOG_LBA),
                Err(e) => println!("logdump: marker {} failed: {}", tag, e),
            }
            if i == 0 {
                std::thread::sleep(Duration::from_millis(200));
            }
        }

        std::thread::sleep(MOUNT_QUIET);
        match write_boot_marker(&ctrl, "post-quiet") {
            Ok(()) => println!("logdump: marker post-quiet ok"),
            Err(e) => println!("logdump: marker post-quiet failed: {}", e),
        }

        // First full dump right after quiet — do not wait for the interval.
        seq += 1;
        if let Ok((payload, head_len, original)) = collect() {
            match dump(&ctrl, seq, t0.elapsed().as_millis(), &payload, head_len, original) {
                Ok(n) => {
                    last_payload = Some(payload);
                    println!("logdump: seq={} {} bytes at t={}ms", seq, n, t0.elapsed().as_millis());
                }
                Err(e) => println!("logdump: first dump failed: {}", e),
            }
        }

        loop {
            let interval = if t0.elapsed() < MOUNT_QUIET + FAST_FOR {
                FAST_INTERVAL
            } else {
                SLOW_INTERVAL
            };
            std::thread::sleep(interval);
            seq += 1;
            let elapsed = t0.elapsed().as_millis();
            match collect() {
                Ok((payload, head_len, original)) => {
                    if last_payload.as_deref() == Some(payload.as_slice()) {
                        continue;
                    }
                    match dump(&ctrl, seq, elapsed, &payload, head_len, original) {
                        Ok(n) => {
                            last_payload = Some(payload);
                            if seq <= 5 || seq % SUCCESS_EVERY == 0 {
                                println!("logdump: seq={} {} bytes at t={}ms", seq, n, elapsed);
                            }
                        }
                        Err(e) => {
                            let now = Instant::now();
                            let print = last_err_print
                                .map(|t| now.duration_since(t) >= Duration::from_secs(60))
                                .unwrap_or(true);
                            if print {
                                println!("logdump: seq={} failed at t={}ms: {}", seq, elapsed, e);
                                last_err_print = Some(now);
                            }
                        }
                    }
                }
                Err(e) => {
                    let now = Instant::now();
                    let print = last_err_print
                        .map(|t| now.duration_since(t) >= Duration::from_secs(60))
                        .unwrap_or(true);
                    if print {
                        println!("logdump: cannot read /scheme/sys/log at t={}ms: {}", elapsed, e);
                        last_err_print = Some(now);
                    }
                }
            }
        }
    });
}
