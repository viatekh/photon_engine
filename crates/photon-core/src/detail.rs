//! Auto detail: a feedback loop that adjusts how much the tracer picks up so the traced
//! content roughly fills the scan budget, whatever the source (sparse line art, a film, a
//! dense fractal). The planner still guarantees nothing breaks up; this just means it rarely
//! has to throw whole shapes away, and a busy frame keeps its strongest structure.

use crate::planner::PlanStats;
use crate::vectorise::{TraceMode, VectoriseParams};
use serde::{Deserialize, Serialize};

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct AutoDetailParams {
    pub enabled: bool,
    /// Fraction of the budget to aim for (headroom for blanking estimates being off).
    pub target_fill: f32,
    /// How quickly to react, 0..1 (higher = faster but can oscillate).
    pub speed: f32,
}

impl Default for AutoDetailParams {
    fn default() -> Self {
        Self { enabled: true, target_fill: 0.9, speed: 0.35 }
    }
}

/// Multiplier on the detail thresholds. 1.0 = the user's settings; >1 = less detail.
pub struct AutoDetail {
    pub level: f32,
}

const MIN_LEVEL: f32 = 0.25;
const MAX_LEVEL: f32 = 40.0;

impl Default for AutoDetail {
    fn default() -> Self {
        Self { level: 1.0 }
    }
}

impl AutoDetail {
    /// The vectoriser settings to use this frame.
    pub fn apply(&self, base: &VectoriseParams, auto: &AutoDetailParams) -> VectoriseParams {
        let mut p = base.clone();
        if !auto.enabled {
            return p;
        }
        let k = self.level;
        match p.mode {
            TraceMode::Edges => {
                p.edge_threshold = (base.edge_threshold * k.powf(0.6)).clamp(0.02, 1.2);
                p.min_length_px = base.min_length_px * k.powf(0.7);
                // Busy content: blur more so bold structure wins over fine texture.
                if k > 1.0 {
                    p.blur_px = (base.blur_px + 0.6 * k.ln()).min(5.0);
                }
            }
            TraceMode::Centreline | TraceMode::Outline => {
                p.min_length_px = base.min_length_px * k;
            }
        }
        p
    }

    /// Feed back the result of planning a frame traced with `apply`'s settings.
    pub fn update(&mut self, stats: &PlanStats, auto: &AutoDetailParams) {
        if !auto.enabled {
            self.level = 1.0;
            return;
        }
        let target = stats.capacity as f32 * auto.target_fill.clamp(0.1, 1.0);
        if target <= 0.0 {
            return;
        }
        let ratio = (stats.demand as f32 / target).max(0.05);
        // Dead band so a steady picture settles instead of hunting.
        if (0.85..=1.05).contains(&ratio) {
            return;
        }
        let step = ratio.powf(auto.speed.clamp(0.02, 1.0));
        self.level = (self.level * step).clamp(MIN_LEVEL, MAX_LEVEL);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn stats(demand: usize) -> PlanStats {
        PlanStats { demand, capacity: 1000, ..Default::default() }
    }

    #[test]
    fn too_much_content_raises_level_and_thresholds() {
        let auto = AutoDetailParams::default();
        let mut d = AutoDetail::default();
        d.update(&stats(10_000), &auto);
        assert!(d.level > 1.5);
        let base = VectoriseParams::default();
        let p = d.apply(&base, &auto);
        assert!(p.edge_threshold > base.edge_threshold);
        assert!(p.min_length_px > base.min_length_px);
    }

    #[test]
    fn sparse_content_lowers_level_but_bounded() {
        let auto = AutoDetailParams::default();
        let mut d = AutoDetail::default();
        for _ in 0..100 {
            d.update(&stats(10), &auto);
        }
        assert_eq!(d.level, MIN_LEVEL);
    }

    #[test]
    fn steady_when_on_target() {
        let auto = AutoDetailParams::default();
        let mut d = AutoDetail::default();
        d.update(&stats(880), &auto);
        assert_eq!(d.level, 1.0);
    }
}
