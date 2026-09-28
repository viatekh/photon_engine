//! LaserCube / LaserDock over USB.
//!
//! Protocol as implemented in Wicked Lasers' open-source libLaserdockCore:
//! * interface 0, bulk endpoint 0x01/0x81: 64-byte command/response packets
//!   (response[1] == 0 means success, u32 values little-endian from response[2]).
//! * interface 1 (alt setting 1), bulk endpoint 0x03: samples, 8 bytes each:
//!   u16 rg (red low byte, green high byte), u16 b, u16 x, u16 y (12-bit, 0..4095).
//!   X is mirrored (libLaserdockCore sends `4095 - x`), so +x in laser space is to the right.
//! * The interlock state is not reported over USB (only over the WiFi protocol).

use super::Dac;
use anyhow::{bail, Context};
use photon_core::LaserPoint;
use rusb::{DeviceHandle, GlobalContext};
use std::time::{Duration, Instant};

const VID: u16 = 0x1fc9;
const PID: u16 = 0x04d8;
const EP_CMD_OUT: u8 = 0x01;
const EP_CMD_IN: u8 = 0x81;
const EP_DATA_OUT: u8 = 0x03;
const TIMEOUT: Duration = Duration::from_millis(200);

const CMD_SET_OUTPUT: u8 = 0x80;
const CMD_SET_DAC_RATE: u8 = 0x82;
const CMD_GET_MAX_DAC_RATE: u8 = 0x84;
const CMD_GET_MIN_DAC_VALUE: u8 = 0x87;
const CMD_GET_MAX_DAC_VALUE: u8 = 0x88;
const CMD_GET_RINGBUFFER_SIZE: u8 = 0x89;
const CMD_GET_RINGBUFFER_EMPTY: u8 = 0x8A;
const CMD_GET_VERSION_MAJOR: u8 = 0x8B;
const CMD_GET_VERSION_MINOR: u8 = 0x8C;
const CMD_CLEAR_RINGBUFFER: u8 = 0x8D;
const CMD_GET_BULK_PACKET_SAMPLES: u8 = 0x8E;

/// Largest batch per bulk transfer (libLaserdockCore uses 768).
const MAX_SAMPLES_PER_TRANSFER: usize = 768;

/// Used when the device doesn't report its ring buffer state.
const FALLBACK_BUFFER: usize = 1000;

pub struct LaserCubeUsb {
    handle: DeviceHandle<GlobalContext>,
    version: (u32, u32),
    max_pps: u32,
    pps: u32,
    dac_min: u32,
    dac_max: u32,
    packet_samples: usize,
    /// None if the device answers the "empty samples" query; otherwise we pace by time.
    timed: Option<(f64, Instant)>,
    bytes: Vec<u8>,
    /// Transfers the cube only partly accepted (the rest was re-sent).
    partial_writes: u64,
}

/// How many LaserCubes are on USB (whether or not another app has them open).
pub fn count() -> usize {
    use rusb::UsbContext;
    // A private context: the global one panics if USB can't be initialised.
    rusb::Context::new()
        .and_then(|ctx| ctx.devices())
        .map(|list| {
            list.iter()
                .filter(|d| d.device_descriptor().is_ok_and(|dd| dd.vendor_id() == VID && dd.product_id() == PID))
                .count()
        })
        .unwrap_or(0)
}

impl LaserCubeUsb {
    pub fn open() -> anyhow::Result<Self> {
        let handle = rusb::open_device_with_vid_pid(VID, PID).context(
            "no LaserCube found on USB (is it plugged in, and not open in LaserOS or another app?)",
        )?;
        let _ = handle.set_auto_detach_kernel_driver(true);
        handle.claim_interface(0).context("claim control interface")?;
        handle.claim_interface(1).context("claim data interface")?;
        handle.set_alternate_setting(1, 1).context("select data alt setting")?;

        let mut dev = Self {
            handle,
            version: (0, 0),
            max_pps: 30_000,
            pps: 30_000,
            dac_min: 0,
            dac_max: 4095,
            packet_samples: 64,
            timed: None,
            bytes: Vec::new(),
            partial_writes: 0,
        };
        dev.version = (
            dev.get_u32(CMD_GET_VERSION_MAJOR).unwrap_or(0),
            dev.get_u32(CMD_GET_VERSION_MINOR).unwrap_or(0),
        );
        if let Ok(v) = dev.get_u32(CMD_GET_MAX_DAC_RATE) {
            if v > 0 {
                dev.max_pps = v;
            }
        }
        if let (Ok(lo), Ok(hi)) = (dev.get_u32(CMD_GET_MIN_DAC_VALUE), dev.get_u32(CMD_GET_MAX_DAC_VALUE)) {
            if hi > lo {
                dev.dac_min = lo;
                dev.dac_max = hi;
            }
        }
        if let Ok(n) = dev.get_u32(CMD_GET_BULK_PACKET_SAMPLES) {
            if (1..=4096).contains(&n) {
                dev.packet_samples = n as usize;
            }
        }
        let buffer = dev.get_u32(CMD_GET_RINGBUFFER_SIZE).unwrap_or(0);
        if dev.get_u32(CMD_GET_RINGBUFFER_EMPTY).is_err() {
            log::warn!("LaserCube did not report ring buffer space; pacing output by time");
            dev.timed = Some((0.0, Instant::now()));
        }
        let _ = dev.command(&[CMD_CLEAR_RINGBUFFER, 0]);
        log::info!(
            "LaserCube USB fw {}.{}, max {} pps, DAC range {}..{}, ring buffer {}, {} samples/packet",
            dev.version.0, dev.version.1, dev.max_pps, dev.dac_min, dev.dac_max, buffer, dev.packet_samples
        );
        Ok(dev)
    }

    fn command(&self, data: &[u8]) -> anyhow::Result<[u8; 64]> {
        self.handle.write_bulk(EP_CMD_OUT, data, TIMEOUT).context("LaserCube command write")?;
        let mut resp = [0u8; 64];
        let n = self.handle.read_bulk(EP_CMD_IN, &mut resp, TIMEOUT).context("LaserCube command read")?;
        if n < 2 || resp[1] != 0 {
            bail!("LaserCube rejected command {:#04x}", data[0]);
        }
        Ok(resp)
    }

    fn get_u32(&self, cmd: u8) -> anyhow::Result<u32> {
        let r = self.command(&[cmd])?;
        Ok(u32::from_le_bytes([r[2], r[3], r[4], r[5]]))
    }

    fn to_dac(&self, v: f32) -> u16 {
        let t = (v.clamp(-1.0, 1.0) + 1.0) * 0.5;
        (self.dac_min as f32 + t * (self.dac_max - self.dac_min) as f32).round() as u16
    }
}

impl Dac for LaserCubeUsb {
    fn description(&self) -> String {
        format!("LaserCube USB (fw {}.{}, max {} pps)", self.version.0, self.version.1, self.max_pps)
    }

    fn max_pps(&self) -> u32 {
        self.max_pps
    }

    fn set_pps(&mut self, pps: u32) -> anyhow::Result<()> {
        let pps = pps.clamp(1000, self.max_pps);
        let b = pps.to_le_bytes();
        self.command(&[CMD_SET_DAC_RATE, b[0], b[1], b[2], b[3]])?;
        self.pps = pps;
        Ok(())
    }

    fn set_enabled(&mut self, on: bool) -> anyhow::Result<()> {
        self.command(&[CMD_SET_OUTPUT, on as u8])?;
        Ok(())
    }

    fn free_space(&mut self) -> anyhow::Result<usize> {
        match &mut self.timed {
            None => Ok(self.get_u32(CMD_GET_RINGBUFFER_EMPTY)? as usize),
            Some((buffered, last)) => {
                let now = Instant::now();
                *buffered = (*buffered - now.duration_since(*last).as_secs_f64() * self.pps as f64).max(0.0);
                *last = now;
                Ok(FALLBACK_BUFFER.saturating_sub(*buffered as usize))
            }
        }
    }

    fn recover(&mut self) -> anyhow::Result<()> {
        LaserCubeUsb::recover(self)
    }

    fn write(&mut self, points: &[LaserPoint]) -> anyhow::Result<()> {
        // Transfers of the size the cube reports (bulk packet sample count), as in the version
        // confirmed working on an LC-2000; the output loop sends several per batch.
        for chunk in points.chunks(self.packet_samples.clamp(1, MAX_SAMPLES_PER_TRANSFER)) {
            self.bytes.clear();
            for p in chunk {
                let c = |v: f32| (v.clamp(0.0, 1.0) * 255.0).round() as u16;
                let rg = c(p.r) | (c(p.g) << 8);
                for v in [rg, c(p.b), self.to_dac(-p.x), self.to_dac(p.y)] {
                    self.bytes.extend_from_slice(&v.to_le_bytes());
                }
            }
            // The cube sometimes takes only part of a transfer (seen on an LC-2000 on macOS:
            // 128 of 512 bytes); send the rest instead of dropping the connection.
            let mut sent = 0;
            let deadline = Instant::now() + TIMEOUT;
            while sent < self.bytes.len() {
                let n = self
                    .handle
                    .write_bulk(EP_DATA_OUT, &self.bytes[sent..], TIMEOUT)
                    .context("LaserCube sample write")?;
                sent += n;
                if sent < self.bytes.len() {
                    self.partial_writes += 1;
                    if self.partial_writes.is_power_of_two() {
                        log::warn!(
                            "LaserCube took {n} of {} bytes; re-sending the rest ({} partial transfers so far)",
                            self.bytes.len() - (sent - n),
                            self.partial_writes
                        );
                    }
                    if Instant::now() > deadline {
                        bail!("LaserCube accepted only {sent} of {} bytes", self.bytes.len());
                    }
                }
            }
        }
        if let Some((buffered, _)) = &mut self.timed {
            *buffered += points.len() as f64;
        }
        Ok(())
    }
}

impl LaserCubeUsb {
    /// Clear the cube's buffer and re-enable output (used by the output watchdog).
    pub fn recover(&mut self) -> anyhow::Result<()> {
        self.command(&[CMD_CLEAR_RINGBUFFER, 0])?;
        self.command(&[CMD_SET_OUTPUT, 1])?;
        Ok(())
    }
}

impl Drop for LaserCubeUsb {
    fn drop(&mut self) {
        // Leave the device dark and idle.
        let _ = self.command(&[CMD_SET_OUTPUT, 0]);
        let _ = self.command(&[CMD_CLEAR_RINGBUFFER, 0]);
        let _ = self.handle.release_interface(1);
        let _ = self.handle.release_interface(0);
    }
}
