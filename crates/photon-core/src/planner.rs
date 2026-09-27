//! Anti-breakup planning: decide what to draw so the frame never exceeds the scan budget.
//!
//! The budget is `pps / refresh_hz` points. Every strategy ends with the whole-shapes pass,
//! so whatever strategy is selected, a shape is either drawn completely or not at all.

use crate::geom::{Path, Vec2, NO_GROUP};
use std::collections::HashMap;
use crate::scan::{self, ScanFrame, ScanParams};
use crate::tracker::{Tracker, TrackingParams};
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
    /// Lowest acceptable refresh rate (C, E). Below ~40 Hz laser flicker becomes visible.
    pub min_hz: f32,
    /// Largest simplification tolerance B/E may use, in laser units (field is 2.0 wide).
    pub max_simplify: f32,
    /// Max number of groups D may alternate between.
    pub max_groups: usize,
    /// 0..1 bonus for shapes drawn last frame, so the selection does not flicker.
    pub stickiness: f32,
    /// A shape needing more than half a frame's budget is split into its separate strokes
    /// (each still drawn whole). Off = shapes are only ever drawn whole (big ones may never be).
    pub split_oversized: bool,
    /// Hysteresis: a shape not drawn last frame only enters if it fits with this fraction of
    /// the budget to spare. Stops shapes at the budget edge flickering in and out.
    pub entry_margin: f32,
    /// Object permanence (follow shapes between frames, commit to them, hold through dropouts).
    pub tracking: TrackingParams,
    /// Commitment: shapes on screen last frame always come before new ones, so a new shape
    /// only appears when there is room and never displaces one being drawn.
    pub commit: bool,
}

impl Default for PlannerParams {
    fn default() -> Self {
        Self {
            strategy: Strategy::Combined,
            priority: Priority::Salient,
            // Laser light redrawn below ~40 Hz visibly strobes (confirmed on an LC-2000), so
            // the floor is 40 Hz: fewer shapes, but a steady image.
            target_hz: 50.0,
            min_hz: 40.0,
            max_simplify: 0.008,
            max_groups: 3,
            stickiness: 0.5,
            split_oversized: true,
            entry_margin: 0.08,
            tracking: TrackingParams::default(),
            commit: true,
        }
    }
}

#[derive(Clone, Debug, Default)]
pub struct PlanStats {
    pub input_paths: usize,
    pub drawn_paths: usize,
    pub input_shapes: usize,
    pub drawn_shapes: usize,
    /// Points in the longest frame.
    pub points: usize,
    pub budget: usize,
    /// Refresh rate of each shape (accounts for groups in D).
    pub refresh_hz: f32,
    pub simplify_used: f32,
    pub groups: usize,
    /// Estimated points needed to draw every input path (for auto detail).
    pub demand: usize,
    /// Part of `demand` from paths auto detail can reduce.
    pub demand_controlled: usize,
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
    /// Paths and shapes drawn last frame, as (centre, length), for stickiness.
    previous: Vec<(Vec2, f32)>,
    previous_shapes: Vec<(Vec2, f32)>,
    tracker: Tracker,
}

impl Planner {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn plan(&mut self, paths: Vec<Path>, scan: &ScanParams, params: &PlannerParams) -> Plan {
        let pps = scan.pps.max(1) as f32;
        let budget_for = |hz: f32| (pps / hz.max(1.0)).floor() as usize;
        // Object permanence: stable ids, held objects; unconfirmed (brand-new) objects wait.
        let (paths, unconfirmed): (Vec<Path>, Vec<Path>) = if params.tracking.enabled {
            let tracked = self.tracker.update(paths, &params.tracking);
            tracked.into_iter().partition(|p| self.tracker.info(p.track).confirmed)
        } else {
            (paths, Vec::new())
        };
        let input_paths = paths.len() + unconfirmed.len();
        let input_shapes = shape_count(&paths) + shape_count(&unconfirmed);
        let cost = |p: &Path| scan::lit_cost(p, scan) + scan.blank_cost(0.1);
        let demand: usize = paths.iter().map(cost).sum();
        let demand_controlled: usize = paths.iter().filter(|p| p.detail_controlled).map(cost).sum();

        if debug_plan() {
            eprintln!("PLAN");
        }
        let split = params.split_oversized;
        let margin = params.entry_margin;
        let incumbent = |shape: &[Path]| self.was_drawn(shape);
        // Rank once; every strategy works through this order.
        let mut ranked = self.rank(paths, params);

        let (groups, dropped, simplify_used, budget) = match params.strategy {
            Strategy::WholeShapes => {
                let b = budget_for(params.target_hz);
                let (sel, drop) = whole_shapes(ranked, scan, b, split, &incumbent, margin);
                (vec![sel], drop, 0.0, b)
            }
            Strategy::Simplify => {
                let b = budget_for(params.target_hz);
                let (paths, eps) = simplify_to_fit(ranked, scan, b, params.max_simplify);
                let (sel, drop) = whole_shapes(paths, scan, b, split, &incumbent, margin);
                (vec![sel], drop, eps, b)
            }
            Strategy::AdaptiveRefresh => {
                let b = budget_for(params.min_hz);
                let (sel, drop) = whole_shapes(ranked, scan, b, split, &incumbent, margin);
                (vec![sel], drop, 0.0, b)
            }
            Strategy::Combined => {
                let b = budget_for(params.min_hz);
                let (paths, eps) = simplify_to_fit(ranked, scan, b, params.max_simplify);
                let (sel, drop) = whole_shapes(paths, scan, b, split, &incumbent, margin);
                (vec![sel], drop, eps, b)
            }
            Strategy::TakeTurns => {
                let b = budget_for(params.target_hz);
                let mut groups = Vec::new();
                for _ in 0..params.max_groups.max(1) {
                    if ranked.is_empty() {
                        break;
                    }
                    let (sel, rest) = whole_shapes(ranked, scan, b, split, &|_: &[Path]| false, 0.0);
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
        let mut dropped = dropped;
        dropped.extend(unconfirmed);
        if params.tracking.enabled {
            self.tracker.drawn(&drawn);
        }
        self.previous = drawn.iter().map(|p| (p.centroid(), p.length())).collect();
        self.previous_shapes = shape_summaries(&drawn).into_iter().map(|(_, c, l)| (c, l)).collect();

        let points = frames.iter().map(|f| f.points.len()).max().unwrap_or(0);
        let total: usize = frames.iter().map(|f| f.points.len()).sum();
        let refresh_hz = if total > 0 { pps / total as f32 } else { 0.0 };
        Plan {
            stats: PlanStats {
                input_paths,
                drawn_paths: drawn.len(),
                input_shapes,
                drawn_shapes: shape_count(&drawn),
                points,
                budget,
                refresh_hz,
                simplify_used,
                groups: frames.len(),
                demand,
                demand_controlled,
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

    /// Whether a shape was on screen last frame: as a whole shape, or mostly (by length) as
    /// individual paths (which survives shapes merging, splitting or being split for size).
    fn was_drawn(&self, shape: &[Path]) -> bool {
        if let Some(p) = shape.first().filter(|p| p.track != 0) {
            // Same object drawn last frame (even if it moved), or - robust to shapes merging,
            // splitting or being split into strokes - the laser was drawing right here.
            // Identity only speaks for the whole object; single strokes of a split object are
            // judged by whether that exact stroke was lit.
            let _ = p;
            return self.tracker.coverage(shape) >= 0.6;
        }
        let similar = |(ac, al): (Vec2, f32), (bc, bl): (Vec2, f32)| {
            ac.distance(bc) < 0.06 && (al - bl).abs() <= 0.35 * al.max(bl)
        };
        let len: f32 = shape.iter().map(|p| p.length()).sum();
        let centre = shape.iter().fold(Vec2::ZERO, |a, p| a + p.centroid() * p.length()) * (1.0 / len.max(1e-6));
        if self.previous_shapes.iter().any(|&prev| similar((centre, len), prev)) {
            return true;
        }
        let seen: f32 = shape
            .iter()
            .filter(|p| self.previous.iter().any(|&prev| similar((p.centroid(), p.length()), prev)))
            .map(|p| p.length())
            .sum();
        seen >= 0.5 * len
    }

    /// Sort by priority, best first, keeping each shape's paths together (a shape's score is the
    /// sum of its paths'). Shapes matching last frame's selection get a boost.
    fn rank(&self, paths: Vec<Path>, params: &PlannerParams) -> Vec<Path> {
        let similar = |(ac, al): (Vec2, f32), (bc, bl): (Vec2, f32)| {
            ac.distance(bc) < 0.06 && (al - bl).abs() <= 0.35 * al.max(bl)
        };
        // (score, path, was this path drawn last frame)
        let scored: Vec<(f32, Path, bool)> = paths
            .into_iter()
            .map(|p| {
                let (lo, hi) = p.bounds();
                let s = match params.priority {
                    Priority::Salient => p.weight * p.length(),
                    Priority::Largest => lo.distance(hi),
                    Priority::Longest => p.length(),
                    Priority::Brightest => p.weight * (1.0 + p.length() * 0.01),
                    Priority::Central => 1.0 / (0.05 + p.centroid().length()),
                };
                let me = (p.centroid(), p.length());
                let seen = self.previous.iter().any(|&prev| similar(me, prev));
                (s, p, seen)
            })
            .collect();
        // Per shape: summed score, total length, length already drawn last frame, bounds.
        let mut shapes_acc: HashMap<u32, (f32, f32, f32, Vec2, Vec2)> = HashMap::new();
        for (s, p, seen) in &scored {
            let key = if p.group == NO_GROUP { continue } else { p.group };
            let (lo, hi) = p.bounds();
            let e = shapes_acc
                .entry(key)
                .or_insert((0.0, 0.0, 0.0, Vec2::new(f32::MAX, f32::MAX), Vec2::new(f32::MIN, f32::MIN)));
            e.0 += s;
            e.1 += p.length();
            if *seen {
                e.2 += p.length();
            }
            e.3 = Vec2::new(e.3.x.min(lo.x), e.3.y.min(lo.y));
            e.4 = Vec2::new(e.4.x.max(hi.x), e.4.y.max(hi.y));
        }
        let salient = params.priority == Priority::Salient;
        let centre_of = |g: u32| {
            let mut sum = Vec2::ZERO;
            let mut len = 0.0;
            for (_, p, _) in scored.iter().filter(|(_, p, _)| p.group == g) {
                sum = sum + p.centroid() * p.length();
                len += p.length();
            }
            sum * (1.0 / len.max(1e-6))
        };
        let shape_score: HashMap<u32, f32> = shapes_acc
            .iter()
            .map(|(&g, &(s, len, seen_len, lo, hi))| {
                // Salient: a tangle (much longer than it is big) is worth less than a clean
                // contour of the same length. A circle has length/diagonal ~2.2; allow ~3.
                let factor = if salient { (3.0 * lo.distance(hi) / len.max(1e-6)).min(1.0) } else { 1.0 };
                // Stickiness: how much of this shape was on screen last frame. Checked per
                // path (survives shapes merging / splitting) and per whole shape (survives
                // the tracer splitting a shape into different pieces).
                let track = scored.iter().find(|(_, p, _)| p.group == g).map(|(_, p, _)| p.track).unwrap_or(0);
                let seen = if track != 0 {
                    // Tracked: on screen if the laser drew here last frame; long-lived objects
                    // weigh more.
                    let members: Vec<Path> =
                        scored.iter().filter(|(_, p, _)| p.group == g).map(|(_, p, _)| p.clone()).collect();
                    let on = if self.tracker.coverage(&members) >= 0.6 {
                        1.0
                    } else {
                        0.0
                    };
                    on * (1.0 + (self.tracker.info(track).age.min(30) as f32) / 30.0)
                } else {
                    let shape_seen = self.previous_shapes.iter().any(|&prev| similar((centre_of(g), len), prev));
                    if shape_seen { 1.0 } else { seen_len / len.max(1e-6) }
                };
                // Commitment: on-screen shapes rank above every new one.
                let committed = if params.commit && seen > 0.0 { 1e6 } else { 1.0 };
                (g, s * factor * (1.0 + params.stickiness * seen) * committed)
            })
            .collect();
        let scored: Vec<(f32, Path)> = scored
            .into_iter()
            .map(|(s, p, seen)| {
                let bonus = if p.group == NO_GROUP && seen {
                    (1.0 + params.stickiness) * if params.commit { 1e6 } else { 1.0 }
                } else {
                    1.0
                };
                (s * bonus, p)
            })
            .collect();
        // Sort key: (shape score, shape id, own score). Ungrouped paths are their own shape.
        let mut keyed: Vec<(f32, u64, f32, Path)> = scored
            .into_iter()
            .enumerate()
            .map(|(i, (s, p))| {
                if p.group == NO_GROUP {
                    (s, (1u64 << 32) + i as u64, s, p)
                } else {
                    (shape_score[&p.group], p.group as u64, s, p)
                }
            })
            .collect();
        keyed.sort_by(|a, b| b.0.total_cmp(&a.0).then(a.1.cmp(&b.1)).then(b.2.total_cmp(&a.2)));
        keyed.into_iter().map(|(_, _, _, p)| p).collect()
    }
}

/// (group, length-weighted centre, total length) per shape; ungrouped paths are their own shape.
fn shape_summaries(paths: &[Path]) -> Vec<(u32, Vec2, f32)> {
    let mut acc: HashMap<u64, (u32, Vec2, f32)> = HashMap::new();
    for (i, p) in paths.iter().enumerate() {
        let key = if p.group == NO_GROUP { (1u64 << 32) + i as u64 } else { p.group as u64 };
        let e = acc.entry(key).or_insert((p.group, Vec2::ZERO, 0.0));
        e.1 = e.1 + p.centroid() * p.length();
        e.2 += p.length();
    }
    acc.into_values().map(|(g, c, l)| (g, c * (1.0 / l.max(1e-6)), l)).collect()
}

/// Set PE_DEBUG_PLAN=1 to log selection decisions (used with --replay).
fn debug_plan() -> bool {
    static ON: std::sync::OnceLock<bool> = std::sync::OnceLock::new();
    *ON.get_or_init(|| std::env::var("PE_DEBUG_PLAN").is_ok())
}

/// Split a ranked list into shapes: consecutive paths sharing a group.
fn shapes(ranked: Vec<Path>) -> Vec<Vec<Path>> {
    let mut out: Vec<Vec<Path>> = Vec::new();
    for p in ranked {
        match out.last_mut() {
            Some(last) if p.group != NO_GROUP && last[0].group == p.group => last.push(p),
            _ => out.push(vec![p]),
        }
    }
    out
}

/// Number of distinct shapes in a list of paths.
pub fn shape_count(paths: &[Path]) -> usize {
    let mut groups: Vec<u32> = paths.iter().filter(|p| p.group != NO_GROUP).map(|p| p.group).collect();
    groups.sort_unstable();
    groups.dedup();
    groups.len() + paths.iter().filter(|p| p.group == NO_GROUP).count()
}

/// Greedily take whole shapes (in the given order) whose rendered frame fits `budget`.
/// A shape is every path sharing a group: it is taken or rejected as a unit.
/// Returns (selected, rejected) paths, both in priority order.
///
/// Hysteresis: a shape for which `incumbent` is true (it was on screen last frame) may use the
/// whole budget; any other shape only gets in if it fits within `budget * (1 - entry_margin)`.
/// So shapes at the edge of the budget don't alternate between drawn and dropped.
pub fn whole_shapes(
    ranked: Vec<Path>,
    scan: &ScanParams,
    budget: usize,
    split_oversized: bool,
    incumbent: &dyn Fn(&[Path]) -> bool,
    entry_margin: f32,
) -> (Vec<Path>, Vec<Path>) {
    let entry_budget = (budget as f32 * (1.0 - entry_margin.clamp(0.0, 0.5))) as usize;
    let mut selected: Vec<Vec<Path>> = Vec::new();
    // Rejected shapes, kept whole (in priority order) for the refill pass.
    let mut rejected_shapes: Vec<Vec<Path>> = Vec::new();
    let mut used = 0usize;
    let mut endpoints: Vec<Vec2> = Vec::new();
    let mut queue: std::collections::VecDeque<Vec<Path>> = shapes(ranked).into();
    while let Some(shape) = queue.pop_front() {
        if split_oversized && shape.len() > 1 {
            let alone: usize = shape.iter().map(|p| scan::lit_cost(p, scan) + scan.blank_cost(0.05)).sum();
            // Split anything over half the budget, not just over the whole budget: a shape near
            // the budget would otherwise flip between "split, mostly drawn" and "whole, dropped"
            // as its size wobbles frame to frame.
            if alone > budget / 2 {
                // Offer its strokes individually, in their own priority order.
                for (i, p) in shape.into_iter().enumerate() {
                    queue.insert(i, vec![p]);
                }
                continue;
            }
        }
        let mut add = 0usize;
        let mut ends = endpoints.clone();
        for p in &shape {
            add += scan::lit_cost(p, scan);
            // Estimate blanking as a jump from the nearest endpoint already in the frame.
            let near = ends
                .iter()
                .map(|e| e.distance(p.start()).min(e.distance(p.end())))
                .fold(f32::MAX, f32::min);
            if ends.is_empty() {
                if !p.closed {
                    add += scan.blank_cost(p.start().distance(p.end()));
                }
            } else {
                add += scan.blank_cost(near);
            }
            ends.push(p.start());
            ends.push(p.end());
        }
        let is_incumbent = incumbent(&shape);
        let limit = if is_incumbent { budget } else { entry_budget };
        if debug_plan() {
            eprintln!("  shape track {} paths {} add {} used {} limit {} -> {}", shape[0].track, shape.len(), add, used, limit, used + add <= limit);
        }
        if used + add <= limit {
            used += add;
            endpoints = ends;
            selected.push(shape);
        } else if split_oversized && is_incumbent && shape.len() > 1 {
            // An on-screen shape that no longer fits: keep as much of it lit as possible by
            // trying its strokes one by one (each stroke is still drawn whole).
            for (i, p) in shape.into_iter().enumerate() {
                queue.insert(i, vec![p]);
            }
        } else {
            rejected_shapes.push(shape);
        }
    }
    // The estimate can be off (ordering differs). Verify with a real render and shed: first
    // shapes that weren't on screen last frame (dropping them causes no visible blink), then
    // on-screen ones, lowest priority first - one at a time, so a small overshoot doesn't
    // throw away a big shape.
    let fits = |shapes: &[Vec<Path>]| {
        let flat: Vec<Path> = shapes.iter().flatten().cloned().collect();
        scan::render(&flat, scan).points.len() <= budget
    };
    while !selected.is_empty() && !fits(&selected) {
        let victim = selected.iter().rposition(|s| !incumbent(s)).unwrap_or(selected.len() - 1);
        if debug_plan() {
            eprintln!("  verify: over budget, shedding track {}", selected[victim][0].track);
        }
        rejected_shapes.push(selected.remove(victim));
    }
    // Refill: the real render may have left room. Retry rejected shapes (on-screen ones first,
    // then by priority) against the actual rendered size; a few attempts at most.
    rejected_shapes.sort_by_key(|s| !incumbent(s));
    let mut attempts = 0;
    let mut i = 0;
    while i < rejected_shapes.len() && attempts < 8 {
        selected.push(rejected_shapes[i].clone());
        attempts += 1;
        if fits(&selected) {
            rejected_shapes.remove(i);
        } else {
            selected.pop();
            i += 1;
        }
    }
    (selected.into_iter().flatten().collect(), rejected_shapes.into_iter().flatten().collect())
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

    /// Single-frame budgeting tests: no object tracking (it delays brand-new objects a frame).
    fn untracked() -> PlannerParams {
        PlannerParams { tracking: TrackingParams { enabled: false, ..Default::default() }, ..Default::default() }
    }

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
            let params = PlannerParams { strategy, ..untracked() };
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
        let params = PlannerParams { strategy: Strategy::WholeShapes, ..untracked() };
        let plan = Planner::new().plan(many_circles(), &scan, &params);
        assert!(plan.drawn.iter().any(|p| p.points.len() == 64));
        assert!(!plan.dropped.is_empty());
    }

    #[test]
    fn simple_content_is_drawn_in_full() {
        let scan = ScanParams::default();
        for strategy in Strategy::ALL {
            let params = PlannerParams { strategy, ..untracked() };
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
            &PlannerParams { strategy: Strategy::WholeShapes, ..untracked() },
        );
        let d = Planner::new().plan(
            many_circles(),
            &scan,
            &PlannerParams { strategy: Strategy::TakeTurns, ..untracked() },
        );
        assert!(d.stats.drawn_paths > a.stats.drawn_paths);
        assert!(d.stats.groups > 1);
    }

    #[test]
    fn grouped_paths_are_drawn_all_or_nothing() {
        let scan = ScanParams::default();
        let params = PlannerParams { strategy: Strategy::WholeShapes, ..untracked() };
        let mut paths = many_circles();
        // Split the big circle into quarter arcs sharing a group, plus give the small circles
        // pairwise groups.
        let big = paths.pop().unwrap();
        for q in 0..4 {
            let n = big.points.len();
            let pts: Vec<Vec2> = (q * n / 4..=((q + 1) * n / 4).min(n - 1)).map(|i| big.points[i]).collect();
            let mut arc = Path::new(pts, false, Rgb::WHITE);
            arc.group = 1000;
            paths.push(arc);
        }
        for (i, p) in paths.iter_mut().enumerate().take(64) {
            p.group = 1 + (i / 2) as u32;
        }
        let plan = Planner::new().plan(paths, &scan, &params);
        assert!(!plan.dropped.is_empty());
        for d in &plan.drawn {
            assert!(!plan.dropped.iter().any(|x| x.group == d.group), "group {} split", d.group);
        }
        assert!(plan.stats.drawn_shapes < plan.stats.drawn_paths);
    }

    #[test]
    fn oversized_shape_splits_into_whole_strokes_only_when_allowed() {
        let scan = ScanParams::default();
        // All circles in one group: far too big for one frame.
        let paths: Vec<Path> = many_circles().into_iter().map(|mut p| { p.group = 7; p }).collect();
        let on = PlannerParams { strategy: Strategy::WholeShapes, ..untracked() };
        let plan = Planner::new().plan(paths.clone(), &scan, &on);
        assert!(plan.stats.drawn_paths > 0);
        assert!(plan.frames[0].points.len() <= plan.stats.budget);
        let off = PlannerParams { split_oversized: false, ..on };
        let plan = Planner::new().plan(paths, &scan, &off);
        assert_eq!(plan.stats.drawn_paths, 0);
    }

    #[test]
    fn tracking_holds_objects_through_a_dropout_and_ignores_one_frame_noise() {
        let scan = ScanParams::default();
        let params = PlannerParams::default();
        let mut planner = Planner::new();
        let ring = |g: u32| {
            let mut c = circle(0.0, 0.0, 0.4, 32);
            c.group = g;
            c
        };
        let blip = || {
            let mut c = circle(0.6, 0.6, 0.05, 12);
            c.group = 99;
            c
        };
        let mut drawn = Vec::new();
        // Frames: ring, ring, ring, (dropout), ring + one-frame blip, ring, ring.
        let frames: Vec<Vec<Path>> = vec![
            vec![ring(1)], vec![ring(2)], vec![ring(3)], vec![], vec![ring(4), blip()], vec![ring(5)], vec![ring(6)],
        ];
        for f in frames {
            let plan = planner.plan(f, &scan, &params);
            drawn.push(plan.drawn.len());
            // The blip is never drawn.
            assert!(plan.drawn.iter().all(|p| p.length() > 1.0), "noise drawn");
        }
        // First frame waits for confirmation; afterwards the ring is drawn every frame,
        // including the dropout frame (held).
        assert_eq!(drawn, vec![0, 1, 1, 1, 1, 1, 1]);
    }

    #[test]
    fn shape_bigger_than_budget_is_never_partially_drawn() {
        let scan = ScanParams::default();
        // 1000 Hz leaves a 30-point budget: far too small for the circle.
        let params = PlannerParams { strategy: Strategy::WholeShapes, target_hz: 1000.0, ..untracked() };
        let plan = Planner::new().plan(vec![circle(0.0, 0.0, 0.9, 64)], &scan, &params);
        assert_eq!(plan.stats.drawn_paths, 0);
        assert!(plan.frames.iter().all(|f| f.points.is_empty()));
    }
}
