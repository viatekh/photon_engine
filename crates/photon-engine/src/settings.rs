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

impl Default for Settings {
    fn default() -> Self {
        Self {
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
