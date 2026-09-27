//! Anti-breakup planning: decide what to draw so the frame never exceeds the scan budget.
//!
//! The budget is `pps / refresh_hz` points. Every strategy ends with the whole-shapes pass,
//! so whatever strategy is selected, a shape is either drawn completely or not at all.

use crate::geom::{Path, Vec2, NO_GROUP};
use std::collections::HashMap;
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
    /// A shape needing more than half a frame's budget is split into its separate strokes
    /// (each still drawn whole). Off = shapes are only ever drawn whole (big ones may never be).
    pub split_oversized: bool,
}

impl Default for PlannerParams {
    fn default() -> Self {
        Self {
            strategy: Strategy::Combined,
            priority: Priority::Salient,
            target_hz: 40.0,
            min_hz: 25.0,
            max_simplify: 0.008,
            max_groups: 3,
            stickiness: 0.5,
            split_oversized: true,
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
}

impl Planner {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn plan(&mut self, paths: Vec<Path>, scan: &ScanParams, params: &PlannerParams) -> Plan {
        let pps = scan.pps.max(1) as f32;
        let budget_for = |hz: f32| (pps / hz.max(1.0)).floor() as usize;
        let input_paths = paths.len();
        let input_shapes = shape_count(&paths);
        let cost = |p: &Path| scan::lit_cost(p, scan) + scan.blank_cost(0.1);
        let demand: usize = paths.iter().map(cost).sum();
        let demand_controlled: usize = paths.iter().filter(|p| p.detail_controlled).map(cost).sum();

        let split = params.split_oversized;
        // Rank once; every strategy works through this order.
        let mut ranked = self.rank(paths, params);

        let (groups, dropped, simplify_used, budget) = match params.strategy {
            Strategy::WholeShapes => {
                let b = budget_for(params.target_hz);
                let (sel, drop) = whole_shapes(ranked, scan, b, split);
                (vec![sel], drop, 0.0, b)
            }
            Strategy::Simplify => {
                let b = budget_for(params.target_hz);
                let (paths, eps) = simplify_to_fit(ranked, scan, b, params.max_simplify);
                let (sel, drop) = whole_shapes(paths, scan, b, split);
                (vec![sel], drop, eps, b)
            }
            Strategy::AdaptiveRefresh => {
                let b = budget_for(params.min_hz);
                let (sel, drop) = whole_shapes(ranked, scan, b, split);
                (vec![sel], drop, 0.0, b)
            }
            Strategy::Combined => {
                let b = budget_for(params.min_hz);
                let (paths, eps) = simplify_to_fit(ranked, scan, b, params.max_simplify);
                let (sel, drop) = whole_shapes(paths, scan, b, split);
                (vec![sel], drop, eps, b)
            }
            Strategy::TakeTurns => {
                let b = budget_for(params.target_hz);
                let mut groups = Vec::new();
                for _ in 0..params.max_groups.max(1) {
                    if ranked.is_empty() {
                        break;
                    }
                    let (sel, rest) = whole_shapes(ranked, scan, b, split);
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
                let shape_seen = self.previous_shapes.iter().any(|&prev| similar((centre_of(g), len), prev));
                let seen = if shape_seen { 1.0 } else { seen_len / len.max(1e-6) };
                (g, s * factor * (1.0 + params.stickiness * seen))
            })
            .collect();
        let scored: Vec<(f32, Path)> = scored
            .into_iter()
            .map(|(s, p, seen)| {
                let bonus = if p.group == NO_GROUP && seen { 1.0 + params.stickiness } else { 1.0 };
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
pub fn whole_shapes(
    ranked: Vec<Path>,
    scan: &ScanParams,
    budget: usize,
    split_oversized: bool,
) -> (Vec<Path>, Vec<Path>) {
    let mut selected: Vec<Vec<Path>> = Vec::new();
    let mut rejected: Vec<Path> = Vec::new();
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
        if used + add <= budget {
            used += add;
            endpoints = ends;
            selected.push(shape);
        } else {
            rejected.extend(shape);
        }
    }
    // The estimate can be off (ordering differs); verify and shed lowest-priority shapes.
    loop {
        let flat: Vec<Path> = selected.iter().flatten().cloned().collect();
        let len = scan::render(&flat, scan).points.len();
        if selected.is_empty() || len <= budget {
            break;
        }
        let mut shed = 0;
        while shed < len - budget {
            let Some(shape) = selected.pop() else { break };
            shed += shape.iter().map(|p| scan::lit_cost(p, scan) + scan.blank_dwell_points() * 2).sum::<usize>();
            rejected.extend(shape);
        }
    }
    (selected.into_iter().flatten().collect(), rejected)
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
    fn grouped_paths_are_drawn_all_or_nothing() {
        let scan = ScanParams::default();
        let params = PlannerParams { strategy: Strategy::WholeShapes, ..Default::default() };
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
        let on = PlannerParams { strategy: Strategy::WholeShapes, ..Default::default() };
        let plan = Planner::new().plan(paths.clone(), &scan, &on);
        assert!(plan.stats.drawn_paths > 0);
        assert!(plan.frames[0].points.len() <= plan.stats.budget);
        let off = PlannerParams { split_oversized: false, ..on };
        let plan = Planner::new().plan(paths, &scan, &off);
        assert_eq!(plan.stats.drawn_paths, 0);
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
