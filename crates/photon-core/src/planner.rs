//! Anti-breakup planning: decide what to draw so the frame never exceeds the scan budget.
//!
//! The budget is `pps / refresh_hz` points. Every strategy ends with the whole-shapes pass,
//! so whatever strategy is selected, a shape is either drawn completely or not at all.

use crate::geom::{Path, Vec2};
use crate::scan::{self, ScanFrame, ScanParams};
use crate::vectorise::simplify;
use serde::{Deserialize, Serialize};

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum Strategy {
    /// A: draw complete shapes in priority order until the budget at the target rate is full.
    WholeShapes,
    /// B: simplify geometry (up to a limit) to fit, then fall back to whole shapes.
    Simplify,
    /// C: let the refresh rate drop to the minimum before dropping shapes.
    AdaptiveRefresh,
    /// D: split shapes into groups drawn on alternate passes (more content, more flicker).
    TakeTurns,
    /// E: adaptive refresh, then simplify, then whole shapes.
    Combined,
}

impl Strategy {
    pub const ALL: [Strategy; 5] = [
        Strategy::WholeShapes,
        Strategy::Simplify,
        Strategy::AdaptiveRefresh,
        Strategy::TakeTurns,
        Strategy::Combined,
    ];

    pub fn label(self) -> &'static str {
        match self {
            Strategy::WholeShapes => "A. Whole shapes only",
            Strategy::Simplify => "B. Simplify, then whole shapes",
            Strategy::AdaptiveRefresh => "C. Adaptive refresh",
            Strategy::TakeTurns => "D. Take turns (multiplex)",
            Strategy::Combined => "E. Combined (C + B + A)",
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum Priority {
    /// Most visually important first: length x strength (edge contrast, or brightness).
    Salient,
    /// Bigger shapes first (bounding box diagonal).
    Largest,
    /// Longer paths first.
    Longest,
    /// Strongest edges / brightest paths first, regardless of size.
    Brightest,
    /// Shapes nearer the centre first.
    Central,
}

impl Priority {
    pub const ALL: [Priority; 5] =
        [Priority::Salient, Priority::Largest, Priority::Longest, Priority::Brightest, Priority::Central];
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct PlannerParams {
    pub strategy: Strategy,
    pub priority: Priority,
    /// Refresh rate the frame must reach (A, B, D).
    pub target_hz: f32,
    /// Lowest acceptable refresh rate (C, E). Below ~25 Hz flicker becomes obvious.
    pub min_hz: f32,
    /// Largest simplification tolerance B/E may use, in laser units (field is 2.0 wide).
    pub max_simplify: f32,
    /// Max number of groups D may alternate between.
    pub max_groups: usize,
    /// 0..1 bonus for shapes drawn last frame, so the selection does not flicker.
    pub stickiness: f32,
}

impl Default for PlannerParams {
    fn default() -> Self {
        Self {
            strategy: Strategy::Combined,
            priority: Priority::Salient,
            target_hz: 40.0,
            min_hz: 25.0,
            max_simplify: 0.02,
            max_groups: 3,
            stickiness: 0.5,
        }
    }
}

#[derive(Clone, Debug, Default)]
pub struct PlanStats {
    pub input_paths: usize,
    pub drawn_paths: usize,
    /// Points in the longest frame.
    pub points: usize,
    pub budget: usize,
    /// Refresh rate of each shape (accounts for groups in D).
    pub refresh_hz: f32,
    pub simplify_used: f32,
    pub groups: usize,
    /// Estimated points needed to draw every input path (for auto detail).
    pub demand: usize,
    /// Points available across all groups (budget x groups for Take Turns).
    pub capacity: usize,
}

/// Output of the planner. `frames` are played in turn (more than one only in Take Turns).
#[derive(Clone, Debug, Default)]
pub struct Plan {
    pub frames: Vec<ScanFrame>,
    /// Paths in laser space that were drawn / left out, for the preview.
    pub drawn: Vec<Path>,
    pub dropped: Vec<Path>,
    pub stats: PlanStats,
}

#[derive(Default)]
pub struct Planner {
    previous: Vec<(Vec2, f32)>,
}

impl Planner {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn plan(&mut self, paths: Vec<Path>, scan: &ScanParams, params: &PlannerParams) -> Plan {
        let pps = scan.pps.max(1) as f32;
        let budget_for = |hz: f32| (pps / hz.max(1.0)).floor() as usize;
        let input_paths = paths.len();
        let demand = paths.iter().map(|p| scan::lit_cost(p, scan)).sum::<usize>()
            + paths.len() * scan.blank_cost(0.1);

        // Rank once; every strategy works through this order.
        let mut ranked = self.rank(paths, params);

        let (groups, dropped, simplify_used, budget) = match params.strategy {
            Strategy::WholeShapes => {
                let b = budget_for(params.target_hz);
                let (sel, drop) = whole_shapes(ranked, scan, b);
                (vec![sel], drop, 0.0, b)
            }
            Strategy::Simplify => {
                let b = budget_for(params.target_hz);
                let (paths, eps) = simplify_to_fit(ranked, scan, b, params.max_simplify);
                let (sel, drop) = whole_shapes(paths, scan, b);
                (vec![sel], drop, eps, b)
            }
            Strategy::AdaptiveRefresh => {
                let b = budget_for(params.min_hz);
                let (sel, drop) = whole_shapes(ranked, scan, b);
                (vec![sel], drop, 0.0, b)
            }
            Strategy::Combined => {
                let b = budget_for(params.min_hz);
                let (paths, eps) = simplify_to_fit(ranked, scan, b, params.max_simplify);
                let (sel, drop) = whole_shapes(paths, scan, b);
                (vec![sel], drop, eps, b)
            }
            Strategy::TakeTurns => {
                let b = budget_for(params.target_hz);
                let mut groups = Vec::new();
                for _ in 0..params.max_groups.max(1) {
                    if ranked.is_empty() {
                        break;
                    }
                    let (sel, rest) = whole_shapes(ranked, scan, b);
                    if sel.is_empty() {
                        ranked = rest;
                        break;
                    }
                    groups.push(sel);
                    ranked = rest;
                }
                (groups, ranked, 0.0, b)
            }
        };

        let frames: Vec<ScanFrame> = groups.iter().map(|g| scan::render(g, scan)).collect();
        let drawn: Vec<Path> = groups.into_iter().flatten().collect();
        self.previous = drawn.iter().map(|p| (p.centroid(), p.length())).collect();

        let points = frames.iter().map(|f| f.points.len()).max().unwrap_or(0);
        let total: usize = frames.iter().map(|f| f.points.len()).sum();
        let refresh_hz = if total > 0 { pps / total as f32 } else { 0.0 };
        Plan {
            stats: PlanStats {
                input_paths,
                drawn_paths: drawn.len(),
                points,
                budget,
                refresh_hz,
                simplify_used,
                groups: frames.len(),
                demand,
                capacity: if params.strategy == Strategy::TakeTurns {
                    budget * params.max_groups.max(1)
                } else {
                    budget
                },
            },
            frames,
            drawn,
            dropped,
        }
    }

    /// Sort paths by priority, best first. Shapes matching last frame's selection get a boost.
    fn rank(&self, paths: Vec<Path>, params: &PlannerParams) -> Vec<Path> {
        let mut scored: Vec<(f32, Path)> = paths
            .into_iter()
            .map(|p| {
                let (lo, hi) = p.bounds();
                let mut s = match params.priority {
                    Priority::Salient => p.weight * p.length(),
                    Priority::Largest => lo.distance(hi),
                    Priority::Longest => p.length(),
                    Priority::Brightest => p.weight * (1.0 + p.length() * 0.01),
                    Priority::Central => 1.0 / (0.05 + p.centroid().length()),
                };
                let c = p.centroid();
                let len = p.length();
                let seen = self.previous.iter().any(|&(pc, pl)| {
                    pc.distance(c) < 0.05 && (pl - len).abs() <= 0.3 * pl.max(len)
                });
                if seen {
                    s *= 1.0 + params.stickiness;
                }
                (s, p)
            })
            .collect();
        scored.sort_by(|a, b| b.0.total_cmp(&a.0));
        scored.into_iter().map(|(_, p)| p).collect()
    }
}

/// Greedily take whole paths (in the given order) whose rendered frame fits `budget`.
/// Returns (selected, rejected), both in priority order.
pub fn whole_shapes(ranked: Vec<Path>, scan: &ScanParams, budget: usize) -> (Vec<Path>, Vec<Path>) {
    let mut selected: Vec<Path> = Vec::new();
    let mut rejected: Vec<Path> = Vec::new();
    let mut used = 0usize;
    for p in ranked {
        let lit = scan::lit_cost(&p, scan);
        // Estimate the extra blanking as a jump from the nearest selected endpoint and back.
        let travel = selected
            .iter()
            .flat_map(|s| [s.start(), s.end()])
            .map(|e| e.distance(p.start()).min(e.distance(p.end())))
            .fold(f32::MAX, f32::min);
        let travel = if selected.is_empty() { 0 } else { scan.blank_cost(travel) };
        let closing = if p.closed { 0 } else { scan.blank_cost(p.start().distance(p.end())) };
        let add = lit + travel + if selected.is_empty() { closing } else { 0 };
        if used + add <= budget {
            used += add;
            selected.push(p);
        } else {
            rejected.push(p);
        }
    }
    // The estimate can be off (ordering differs); verify and shed lowest priority until it fits.
    loop {
        let len = scan::render(&selected, scan).points.len();
        if selected.is_empty() || len <= budget {
            break;
        }
        // Drop enough of the lowest-priority paths to cover the overshoot, then re-check.
        let mut shed = 0;
        while shed < len - budget {
            let Some(p) = selected.pop() else { break };
            shed += scan::lit_cost(&p, scan) + scan.blank_dwell_points() * 2;
            rejected.push(p);
        }
    }
    (selected, rejected)
}

/// Find the smallest simplification (up to `max_eps`) that makes everything fit.
/// Returns the simplified paths (maximally simplified if nothing fits) and the tolerance used.
fn simplify_to_fit(paths: Vec<Path>, scan: &ScanParams, budget: usize, max_eps: f32) -> (Vec<Path>, f32) {
    // Cheap lower bound first (lit points alone), full render only if that passes.
    let fits = |ps: &[Path]| {
        ps.iter().map(|p| scan::lit_cost(p, scan)).sum::<usize>() <= budget
            && scan::render(ps, scan).points.len() <= budget
    };
    if fits(&paths) || max_eps <= 0.0 {
        return (paths, 0.0);
    }
    let apply = |eps: f32| -> Vec<Path> {
        paths
            .iter()
            .map(|p| {
                let pts = simplify(&p.points, p.closed, eps);
                // Never simplify a shape out of existence.
                if pts.len() < if p.closed { 3 } else { 2 } {
                    p.clone()
                } else {
                    Path { points: pts, ..p.clone() }
                }
            })
            .collect()
    };
    let at_max = apply(max_eps);
    if !fits(&at_max) {
        return (at_max, max_eps);
    }
    let (mut lo, mut hi) = (0.0f32, max_eps);
    for _ in 0..7 {
        let mid = 0.5 * (lo + hi);
        if fits(&apply(mid)) {
            hi = mid;
        } else {
            lo = mid;
        }
    }
    (apply(hi), hi)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geom::Rgb;

    fn circle(cx: f32, cy: f32, r: f32, n: usize) -> Path {
        let pts = (0..n)
            .map(|i| {
                let a = i as f32 / n as f32 * std::f32::consts::TAU;
                Vec2::new(cx + r * a.cos(), cy + r * a.sin())
            })
            .collect();
        Path::new(pts, true, Rgb::WHITE)
    }

    fn many_circles() -> Vec<Path> {
        let mut v = Vec::new();
        for i in 0..8 {
            for j in 0..8 {
                let r = 0.02 + 0.01 * ((i * 8 + j) % 5) as f32;
                v.push(circle(-0.85 + i as f32 * 0.24, -0.85 + j as f32 * 0.24, r, 24));
            }
        }
        v.push(circle(0.0, 0.0, 0.9, 64));
        v
    }

    #[test]
    fn every_strategy_respects_budget() {
        let scan = ScanParams::default();
        for strategy in Strategy::ALL {
            let params = PlannerParams { strategy, ..Default::default() };
            let mut planner = Planner::new();
            let plan = planner.plan(many_circles(), &scan, &params);
            assert!(!plan.frames.is_empty(), "{strategy:?}");
            for f in &plan.frames {
                assert!(f.points.len() <= plan.stats.budget, "{strategy:?}: {} > {}", f.points.len(), plan.stats.budget);
            }
            assert!(plan.stats.drawn_paths > 0);
            assert_eq!(plan.stats.drawn_paths + plan.dropped.len(), plan.stats.input_paths, "{strategy:?}");
        }
    }

    #[test]
    fn largest_shape_is_kept_first() {
        let scan = ScanParams::default();
        let params = PlannerParams { strategy: Strategy::WholeShapes, ..Default::default() };
        let plan = Planner::new().plan(many_circles(), &scan, &params);
        assert!(plan.drawn.iter().any(|p| p.points.len() == 64));
        assert!(!plan.dropped.is_empty());
    }

    #[test]
    fn simple_content_is_drawn_in_full() {
        let scan = ScanParams::default();
        for strategy in Strategy::ALL {
            let params = PlannerParams { strategy, ..Default::default() };
            let plan = Planner::new().plan(vec![circle(0.0, 0.0, 0.5, 32)], &scan, &params);
            assert_eq!(plan.stats.drawn_paths, 1);
            assert!(plan.dropped.is_empty());
            assert_eq!(plan.stats.simplify_used, 0.0);
        }
    }

    #[test]
    fn take_turns_draws_more_than_whole_shapes() {
        let scan = ScanParams::default();
        let a = Planner::new().plan(
            many_circles(),
            &scan,
            &PlannerParams { strategy: Strategy::WholeShapes, ..Default::default() },
        );
        let d = Planner::new().plan(
            many_circles(),
            &scan,
            &PlannerParams { strategy: Strategy::TakeTurns, ..Default::default() },
        );
        assert!(d.stats.drawn_paths > a.stats.drawn_paths);
        assert!(d.stats.groups > 1);
    }

    #[test]
    fn shape_bigger_than_budget_is_never_partially_drawn() {
        let scan = ScanParams { pps: 1000, ..Default::default() };
        let params = PlannerParams { strategy: Strategy::WholeShapes, ..Default::default() };
        let plan = Planner::new().plan(vec![circle(0.0, 0.0, 0.9, 64)], &scan, &params);
        assert_eq!(plan.stats.drawn_paths, 0);
        assert!(plan.frames.iter().all(|f| f.points.is_empty()));
    }
}
