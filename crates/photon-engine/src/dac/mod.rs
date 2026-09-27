//! Laser DAC outputs.

pub mod lasercube_usb;
pub mod simulator;

use photon_core::LaserPoint;
use serde::{Deserialize, Serialize};

pub trait Dac: Send {
    fn description(&self) -> String;
    /// Highest point rate the device accepts.
    fn max_pps(&self) -> u32;
    fn set_pps(&mut self, pps: u32) -> anyhow::Result<()>;
    /// Hardware output enable (the laser still needs blanked points to be dark).
    fn set_enabled(&mut self, on: bool) -> anyhow::Result<()>;
    /// How many points can be written right now without blocking.
    fn free_space(&mut self) -> anyhow::Result<usize>;
    fn write(&mut self, points: &[LaserPoint]) -> anyhow::Result<()>;
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub enum DacSelection {
    /// Stream to nothing, but at real speed: for testing without hardware.
    #[default]
    Simulator,
    LaserCubeUsb,
}

impl DacSelection {
    pub const ALL: [DacSelection; 2] = [DacSelection::Simulator, DacSelection::LaserCubeUsb];

    pub fn label(self) -> &'static str {
        match self {
            DacSelection::Simulator => "Simulator (no hardware)",
            DacSelection::LaserCubeUsb => "LaserCube / LaserDock USB",
        }
    }

    pub fn open(self) -> anyhow::Result<Box<dyn Dac>> {
        Ok(match self {
            DacSelection::Simulator => Box::new(simulator::SimulatorDac::new()),
            DacSelection::LaserCubeUsb => Box::new(lasercube_usb::LaserCubeUsb::open()?),
        })
    }
}
