//! Everything the user can configure. Persisted between runs (except arming, which never is).

use crate::dac::DacSelection;
use crate::input::SourceSelection;
use photon_core::detail::AutoDetailParams;
use photon_core::keystone::OutputGeometry;
use photon_core::output::ColourParams;
use photon_core::patterns::TestPattern;
use photon_core::planner::PlannerParams;
use photon_core::scan::ScanParams;
use photon_core::vectorise::VectoriseParams;
use serde::{Deserialize, Serialize};

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct Settings {
    /// Bumped when tracing defaults change so old saved settings pick them up.
    /// Missing in old files, so it must deserialize as 0 rather than the current version.
    #[serde(default)]
    pub version: u32,
    pub source: SourceSelection,
    pub flip_input_y: bool,
    pub test_pattern_on: bool,
    pub test_pattern: TestPattern,
    pub vectorise: VectoriseParams,
    pub auto_detail: AutoDetailParams,
    pub planner: PlannerParams,
    pub scan: ScanParams,
    pub colour: ColourParams,
    pub geometry: OutputGeometry,
    pub dac: DacSelection,
    /// Blank if no new frame has arrived for this long.
    pub signal_timeout_ms: u32,
    /// Blank frames whose lit points all fall within this extent (a near-static beam).
    pub static_beam_min_extent: f32,
}

pub const SETTINGS_VERSION: u32 = 5;

impl Settings {
    /// Bring settings saved by an older version up to date. Tracing / planning settings reset to
    /// the new defaults; source, device, output geometry and colour are kept.
    pub fn migrate(self) -> Self {
        if self.version >= SETTINGS_VERSION {
            return self;
        }
        log::info!("settings from version {} - resetting tracing defaults", self.version);
        Settings {
            source: self.source,
            dac: self.dac,
            geometry: self.geometry,
            // v5: keep colour as tuned; scan tuning back to defaults (incl. scanner rating 30 -
            // low ratings were being used to hide flicker, which was really the refresh rate).
            colour: if self.version >= 4 {
                self.colour
            } else {
                ColourParams { colour_delay_us: ColourParams::default().colour_delay_us, ..self.colour }
            },
            scan: ScanParams { pps: self.scan.pps, ..Default::default() },
            ..Default::default()
        }
    }
}

impl Default for Settings {
    fn default() -> Self {
        Self {
            version: SETTINGS_VERSION,
            source: SourceSelection::None,
            flip_input_y: false,
            test_pattern_on: true,
            test_pattern: TestPattern::Frame,
            vectorise: VectoriseParams::default(),
            auto_detail: AutoDetailParams::default(),
            planner: PlannerParams::default(),
            scan: ScanParams::default(),
            colour: ColourParams::default(),
            geometry: OutputGeometry::default(),
            dac: DacSelection::Simulator,
            signal_timeout_ms: 500,
            static_beam_min_extent: 0.05,
        }
    }
}
