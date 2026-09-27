//! Consumes points at the configured rate, like a real DAC with a buffer.

use super::Dac;
use photon_core::LaserPoint;
use std::time::Instant;

const CAPACITY: f64 = 2000.0;

pub struct SimulatorDac {
    pps: u32,
    buffered: f64,
    last: Instant,
}

impl SimulatorDac {
    pub fn new() -> Self {
        Self { pps: 30_000, buffered: 0.0, last: Instant::now() }
    }

    fn drain(&mut self) {
        let now = Instant::now();
        let dt = now.duration_since(self.last).as_secs_f64();
        self.last = now;
        self.buffered = (self.buffered - dt * self.pps as f64).max(0.0);
    }
}

impl Dac for SimulatorDac {
    fn description(&self) -> String {
        "Simulator".into()
    }
    fn max_pps(&self) -> u32 {
        100_000
    }
    fn set_pps(&mut self, pps: u32) -> anyhow::Result<()> {
        self.pps = pps.max(1);
        Ok(())
    }
    fn set_enabled(&mut self, _on: bool) -> anyhow::Result<()> {
        Ok(())
    }
    fn free_space(&mut self) -> anyhow::Result<usize> {
        self.drain();
        Ok((CAPACITY - self.buffered) as usize)
    }
    fn write(&mut self, points: &[LaserPoint]) -> anyhow::Result<()> {
        self.drain();
        self.buffered += points.len() as f64;
        Ok(())
    }
}
