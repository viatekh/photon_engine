//! Converting paths into a point stream the scanners can physically follow.
//!
//! Galvo mirrors have inertia, so the point stream is generated from a **motion profile**:
//! the beam accelerates out of corners, cruises at up to the maximum speed, and brakes into
//! corners and path ends, never exceeding the acceleration limit. Corner speed comes from how
//! sharp the turn is (junction deviation, as in CNC / 3D printer planners) and from curvature,
//! so gentle curves run at full speed while sharp corners come almost to rest - no fixed dwell
//! points, no overshoot from slamming into a corner at full speed.
//!
//! Speeds are in laser-space units per second (the field is 2.0 wide), accelerations in
//! units per second squared, holds in microseconds, so settings mean the same at any point rate.

use crate::geom::{LaserPoint, Path, Rgb, Vec2};
use serde::{Deserialize, Serialize};

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct ScanParams {
    /// Points per second sent to the DAC.
    pub pps: u32,
    /// Galvo rating in kpps at ILDA 8 degrees (e.g. 30 for "30K scanners"). Speeds and
    /// accelerations below are for a 30K scanner; speed scales with rating / 30, acceleration
    /// with its square.
    pub scanner_kpps: f32,
    /// Max beam speed while lit.
    pub lit_speed: f32,
    /// Max acceleration while lit. Lower = smoother, less overshoot, more points per shape.
    pub lit_accel: f32,
    /// How far (laser units) a corner may be rounded off at speed. Lower = sharper corners,
    /// slower through them.
    pub corner_tolerance: f32,
    /// Max beam speed while blanked (jumping between shapes).
    pub blank_speed: f32,
    /// Max acceleration while blanked.
    pub blank_accel: f32,
    /// Lit hold at the start and end of every path.
    pub path_dwell_us: f32,
    /// Blanked hold at the old position before a jump (lets the colour switch off).
    pub blank_pre_us: f32,
    /// Blanked hold at the new position after a jump (lets the mirrors settle).
    pub blank_post_us: f32,
    /// Paths whose start is this close to the previous end are joined without blanking.
    pub join_distance: f32,
    /// Closed shapes are drawn this much past their starting point (in time at lit speed).
    pub closed_overlap_us: f32,
}

impl Default for ScanParams {
    fn default() -> Self {
        Self {
            pps: 30_000,
            scanner_kpps: 30.0,
            lit_speed: 600.0,
            lit_accel: 1.2e6,
            corner_tolerance: 0.003,
            blank_speed: 1800.0,
            blank_accel: 3.0e6,
            path_dwell_us: 33.0,
            blank_pre_us: 33.0,
            blank_post_us: 130.0,
            join_distance: 0.004,
            closed_overlap_us: 0.0,
        }
    }
}

/// Speed / acceleration limits in per-point units.
#[derive(Clone, Copy, Debug)]
struct Limits {
    vmax: f32,
    accel: f32,
}

impl ScanParams {
    /// How much faster than a 30K scanner the galvos are.
    fn scanner_factor(&self) -> f32 {
        (self.scanner_kpps / 30.0).clamp(0.1, 10.0)
    }
    fn pps_f(&self) -> f32 {
        self.pps.max(1) as f32
    }
    /// Holds shrink for faster scanners (they settle sooner).
    pub fn points_for(&self, us: f32) -> usize {
        (us / self.scanner_factor() * 1e-6 * self.pps_f()).round().max(0.0) as usize
    }
    fn lit_limits(&self) -> Limits {
        let f = self.scanner_factor();
        Limits {
            vmax: (self.lit_speed * f / self.pps_f()).max(1e-5),
            accel: (self.lit_accel * f * f / (self.pps_f() * self.pps_f())).max(1e-8),
        }
    }
    fn blank_limits(&self) -> Limits {
        let f = self.scanner_factor();
        Limits {
            vmax: (self.blank_speed * f / self.pps_f()).max(1e-5),
            accel: (self.blank_accel * f * f / (self.pps_f() * self.pps_f())).max(1e-8),
        }
    }
    /// Largest distance between consecutive lit points.
    pub fn lit_step(&self) -> f32 {
        self.lit_limits().vmax
    }
    /// Largest distance between consecutive blanked points.
    pub fn blank_step(&self) -> f32 {
        self.blank_limits().vmax
    }
    pub fn path_dwell_points(&self) -> usize {
        self.points_for(self.path_dwell_us)
    }
    /// Blanked hold points around a jump (before + after).
    pub fn blank_dwell_points(&self) -> usize {
        self.points_for(self.blank_pre_us) + self.points_for(self.blank_post_us)
    }

    /// Number of points a blanked jump of `distance` costs (rest to rest, plus holds).
    pub fn blank_cost(&self, distance: f32) -> usize {
        if distance <= self.join_distance {
            return 0;
        }
        self.blank_dwell_points() + travel_points(distance, self.blank_limits())
    }
}

/// Points needed to travel `d` from rest to rest with the given limits.
fn travel_points(d: f32, l: Limits) -> usize {
    let t = if d < l.vmax * l.vmax / l.accel {
        2.0 * (d / l.accel).sqrt()
    } else {
        d / l.vmax + l.vmax / l.accel
    };
    t.ceil().max(1.0) as usize
}

/// One loopable frame of points. The last point leads back into the first.
#[derive(Clone, Debug, Default)]
pub struct ScanFrame {
    pub points: Vec<LaserPoint>,
    pub path_count: usize,
}

/// A path oriented for drawing: vertices in drawing order, closing point included.
fn oriented(path: &Path, reversed: bool, start: usize) -> Vec<Vec2> {
    let mut pts: Vec<Vec2> = if path.closed {
        let n = path.points.len();
        let mut v: Vec<Vec2> = (0..=n).map(|i| path.points[(start + i) % n]).collect();
        if reversed {
            v.reverse();
        }
        v
    } else {
        path.points.clone()
    };
    if !path.closed && reversed {
        pts.reverse();
    }
    pts
}

/// Walk a polyline with a motion profile, pushing one point per sample period.
/// Starts and ends at rest. `make` builds the point (lit or blank) for a position.
fn walk(pts: &[Vec2], l: Limits, corner_tol: f32, out: &mut Vec<LaserPoint>, make: &dyn Fn(Vec2) -> LaserPoint) {
    let n = pts.len();
    if n < 2 {
        return;
    }
    let seg_len: Vec<f32> = pts.windows(2).map(|w| w[0].distance(w[1])).collect();
    // Speed allowed at each vertex from the corner, then limited by braking distance both ways.
    let mut v = vec![l.vmax; n];
    v[0] = 0.0;
    v[n - 1] = 0.0;
    for i in 1..n - 1 {
        v[i] = corner_speed(pts[i - 1], pts[i], pts[i + 1], l, corner_tol);
    }
    for i in 1..n {
        v[i] = v[i].min((v[i - 1] * v[i - 1] + 2.0 * l.accel * seg_len[i - 1]).sqrt());
    }
    for i in (0..n - 1).rev() {
        v[i] = v[i].min((v[i + 1] * v[i + 1] + 2.0 * l.accel * seg_len[i]).sqrt());
    }
    // Speed at distance s into segment i (trapezoid / triangle profile).
    let speed_at = |i: usize, s: f32| -> f32 {
        let len = seg_len[i];
        let a = (v[i] * v[i] + 2.0 * l.accel * s.max(0.0)).sqrt();
        let b = (v[i + 1] * v[i + 1] + 2.0 * l.accel * (len - s).max(0.0)).sqrt();
        l.vmax.min(a).min(b)
    };
    // The first step from rest covers accel/2; never step less, so the walk can't stall.
    let min_step = (l.accel * 0.5).min(l.vmax).max(1e-6);
    let mut i = 0usize;
    let mut s = 0.0f32;
    loop {
        // Midpoint estimate of the distance covered in one sample period.
        let v0 = speed_at(i, s);
        let step = speed_at(i, s + v0 * 0.5).max(min_step);
        let mut s_next = s + step;
        // Crossing vertices: stop exactly on a vertex we must (nearly) stop at; otherwise
        // carry the remaining distance into the next segment.
        while s_next >= seg_len[i] {
            if i + 1 == n - 1 {
                out.push(make(pts[n - 1]));
                return;
            }
            if v[i + 1] < 0.25 * l.vmax {
                s_next = seg_len[i];
                break;
            }
            s_next -= seg_len[i];
            i += 1;
        }
        s = s_next;
        if s >= seg_len[i] {
            // Landed exactly on a slow vertex: emit it and move on.
            out.push(make(pts[i + 1]));
            i += 1;
            s = 0.0;
            if i == n - 1 {
                return;
            }
        } else {
            out.push(make(pts[i].lerp(pts[i + 1], s / seg_len[i].max(1e-9))));
        }
    }
}

/// Max speed through vertex `b` between segments a-b and b-c.
fn corner_speed(a: Vec2, b: Vec2, c: Vec2, l: Limits, tol: f32) -> f32 {
    let (d1, d2) = (b - a, c - b);
    let (l1, l2) = (d1.length(), d2.length());
    if l1 < 1e-9 || l2 < 1e-9 {
        return 0.0;
    }
    // Turn angle: 0 = straight on, PI = full reversal.
    let turn = (d1.dot(d2) / (l1 * l2)).clamp(-1.0, 1.0).acos();
    if turn < 1e-4 {
        return l.vmax;
    }
    // Junction deviation: the corner may be rounded off by at most `tol`.
    let half = ((std::f32::consts::PI - turn) * 0.5).sin(); // sin(angle between segments / 2)
    let junction = if half >= 0.9999 {
        l.vmax
    } else {
        (l.accel * tol.max(1e-6) * half / (1.0 - half)).sqrt()
    };
    // Curvature: treat the vertex as part of an arc through the shorter neighbouring segment.
    let radius = l1.min(l2) / (2.0 * (turn * 0.5).sin());
    let centripetal = (l.accel * radius).sqrt();
    l.vmax.min(junction).min(centripetal)
}

/// Emit the lit points for an already-oriented polyline.
fn emit_lit(pts: &[Vec2], color: Rgb, params: &ScanParams, closed: bool, out: &mut Vec<LaserPoint>) {
    let dwell = params.path_dwell_points();
    let lit = |p: Vec2| LaserPoint::lit(p, color);
    for _ in 0..dwell.max(1) {
        out.push(lit(pts[0]));
    }
    let mut path: Vec<Vec2> = pts.to_vec();
    // Closed shapes: continue a little past the seam so mirror lag doesn't leave a gap.
    if closed && pts.len() > 2 {
        let mut extra = params.lit_step() * params.points_for(params.closed_overlap_us) as f32;
        for i in 1..pts.len() {
            if extra <= 0.0 {
                break;
            }
            let d = pts[i - 1].distance(pts[i]);
            if d >= extra {
                path.push(pts[i - 1].lerp(pts[i], extra / d.max(1e-9)));
                break;
            }
            path.push(pts[i]);
            extra -= d;
        }
    }
    walk(&path, params.lit_limits(), params.corner_tolerance, out, &lit);
    let end = out.last().map(|p| p.pos()).unwrap_or(pts[pts.len() - 1]);
    for _ in 0..dwell {
        out.push(lit(end));
    }
}

/// Blanked move from rest to rest, with holds at both ends.
pub fn emit_blank(from: Vec2, to: Vec2, params: &ScanParams, out: &mut Vec<LaserPoint>) {
    if from.distance(to) <= params.join_distance {
        return;
    }
    for _ in 0..params.points_for(params.blank_pre_us) {
        out.push(LaserPoint::blank(from));
    }
    walk(&[from, to], params.blank_limits(), 0.0, out, &LaserPoint::blank);
    for _ in 0..params.points_for(params.blank_post_us) {
        out.push(LaserPoint::blank(to));
    }
}

/// Lit point count for a path on its own (independent of orientation).
pub fn lit_cost(path: &Path, params: &ScanParams) -> usize {
    let mut v = Vec::new();
    emit_lit(&oriented(path, false, 0), path.color, params, path.closed, &mut v);
    v.len()
}

struct Choice {
    index: usize,
    reversed: bool,
    start: usize,
}

/// Greedy nearest-neighbour ordering: pick the path whose entry point is closest to the beam.
fn order(paths: &[Path]) -> Vec<Choice> {
    let mut remaining: Vec<usize> = (0..paths.len()).collect();
    let mut out = Vec::with_capacity(paths.len());
    let mut pos = match paths.first() {
        Some(p) => p.start(),
        None => return out,
    };
    while !remaining.is_empty() {
        let mut best = (f32::MAX, 0usize, false, 0usize); // (dist, slot, reversed, start)
        for (slot, &i) in remaining.iter().enumerate() {
            let p = &paths[i];
            if p.closed && p.fixed_start {
                // Tracked shape: keep its seam and direction so it looks the same every frame.
                let d = p.points[0].distance(pos);
                if d < best.0 {
                    best = (d, slot, false, 0);
                }
            } else if p.closed {
                for (k, &v) in p.points.iter().enumerate() {
                    let d = v.distance(pos);
                    if d < best.0 {
                        best = (d, slot, false, k);
                    }
                }
            } else {
                let ds = p.start().distance(pos);
                let de = p.end().distance(pos);
                if ds < best.0 {
                    best = (ds, slot, false, 0);
                }
                if de < best.0 {
                    best = (de, slot, true, 0);
                }
            }
        }
        let index = remaining.swap_remove(best.1);
        let choice = Choice { index, reversed: best.2, start: best.3 };
        let pts = oriented(&paths[index], choice.reversed, choice.start);
        pos = pts[pts.len() - 1];
        out.push(choice);
    }
    out
}

/// Order and render paths into a loopable frame (includes the jump back to the start).
pub fn render(paths: &[Path], params: &ScanParams) -> ScanFrame {
    let mut points = Vec::new();
    let mut first: Option<Vec2> = None;
    let mut pos: Option<Vec2> = None;
    for choice in order(paths) {
        let path = &paths[choice.index];
        let pts = oriented(path, choice.reversed, choice.start);
        if let Some(p) = pos {
            emit_blank(p, pts[0], params, &mut points);
        }
        first.get_or_insert(pts[0]);
        emit_lit(&pts, path.color, params, path.closed, &mut points);
        pos = points.last().map(|p| p.pos());
    }
    if let (Some(p), Some(f)) = (pos, first) {
        emit_blank(p, f, params, &mut points);
    }
    ScanFrame { points, path_count: paths.len() }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn line(x0: f32, x1: f32, y: f32) -> Path {
        Path::new(vec![Vec2::new(x0, y), Vec2::new(x1, y)], false, Rgb::WHITE)
    }

    #[test]
    fn lit_steps_never_exceed_speed_limit() {
        let params = ScanParams::default();
        let sq = Path::new(
            vec![Vec2::new(-1.0, -1.0), Vec2::new(1.0, -1.0), Vec2::new(1.0, 1.0), Vec2::new(-1.0, 1.0)],
            true,
            Rgb::WHITE,
        );
        let frame = render(&[sq, line(-0.5, 0.5, 0.0)], &params);
        let pts = &frame.points;
        let mut max_lit = 0.0f32;
        let mut max_blank = 0.0f32;
        for i in 0..pts.len() {
            let (a, b) = (pts[i], pts[(i + 1) % pts.len()]);
            let d = a.pos().distance(b.pos());
            if a.is_lit() && b.is_lit() {
                max_lit = max_lit.max(d);
            } else {
                max_blank = max_blank.max(d);
            }
        }
        assert!(max_lit <= params.lit_step() * 1.001, "{max_lit}");
        assert!(max_blank <= params.blank_step() * 1.001, "{max_blank}");
    }

    /// Speed changes point to point stay within the acceleration limit (lit and blanked),
    /// except the deliberate near-stops at sharp corners.
    #[test]
    fn acceleration_is_limited() {
        let params = ScanParams::default();
        let tri = Path::new(
            vec![Vec2::new(-0.8, -0.6), Vec2::new(0.8, -0.6), Vec2::new(0.0, 0.8)],
            true,
            Rgb::WHITE,
        );
        let circle = Path::new(
            (0..48)
                .map(|i| {
                    let a = i as f32 / 48.0 * std::f32::consts::TAU;
                    Vec2::new(0.5 * a.cos(), 0.5 * a.sin())
                })
                .collect(),
            true,
            Rgb::WHITE,
        );
        let frame = render(&[tri, circle], &params);
        let pts = &frame.points;
        let (la, ba) = (params.lit_limits().accel, params.blank_limits().accel);
        for i in 1..pts.len() - 1 {
            let v1 = pts[i].pos() - pts[i - 1].pos();
            let v2 = pts[i + 1].pos() - pts[i].pos();
            let dv = (v2.length() - v1.length()).abs();
            let limit = if pts[i].is_lit() { la } else { ba };
            assert!(dv <= limit * 1.6 + 1e-5, "speed change {dv} > {limit} at {i}");
        }
    }

    #[test]
    fn sharp_corners_slow_down_gentle_curves_do_not() {
        let l = ScanParams::default().lit_limits();
        let sharp = corner_speed(Vec2::new(0.0, 0.0), Vec2::new(1.0, 0.0), Vec2::new(1.0, 1.0), l, 0.003);
        let gentle = corner_speed(Vec2::new(0.0, 0.0), Vec2::new(0.5, 0.0), Vec2::new(1.0, 0.02), l, 0.003);
        assert!(sharp < 0.2 * l.vmax, "{sharp}");
        assert!(gentle > 0.9 * l.vmax, "{gentle}");
    }

    #[test]
    fn adjacent_paths_are_joined_without_blanking() {
        let params = ScanParams::default();
        let frame = render(&[line(-0.5, 0.0, 0.0), line(0.0, 0.5, 0.0)], &params);
        // Only the jump back to the start of the frame should be blanked.
        let blank_runs = frame
            .points
            .windows(2)
            .filter(|w| w[0].is_lit() && !w[1].is_lit())
            .count();
        assert_eq!(blank_runs, 1);
    }

    #[test]
    fn nearest_neighbour_reverses_open_paths() {
        let params = ScanParams::default();
        // Second line is closer via its end.
        let paths = [line(-1.0, -0.5, 0.0), line(0.5, -0.4, 0.0)];
        let frame = render(&paths, &params);
        let lit: Vec<_> = frame.points.iter().filter(|p| p.is_lit()).collect();
        // After the first line (ending at -0.5) we should continue at -0.4, not jump to 0.5.
        let idx = lit.iter().position(|p| (p.x - (-0.4)).abs() < 1e-4).unwrap();
        let idx_far = lit.iter().position(|p| (p.x - 0.5).abs() < 1e-4).unwrap();
        assert!(idx < idx_far);
    }

    #[test]
    fn closed_overlap_draws_past_the_seam_without_jumps() {
        let sq = Path::new(
            vec![Vec2::new(-0.5, -0.5), Vec2::new(0.5, -0.5), Vec2::new(0.5, 0.5), Vec2::new(-0.5, 0.5)],
            true,
            Rgb::WHITE,
        );
        let base = ScanParams::default();
        let over = ScanParams { closed_overlap_us: 300.0, ..Default::default() };
        let a = render(&[sq.clone()], &base).points;
        let b = render(&[sq], &over).points;
        let lit = |v: &[LaserPoint]| v.iter().filter(|p| p.is_lit()).count();
        assert!(lit(&b) > lit(&a) + 3, "{} vs {}", lit(&b), lit(&a));
        // Loops seamlessly: every step (including the blanked return) within limits.
        for i in 0..b.len() {
            let d = b[i].pos().distance(b[(i + 1) % b.len()].pos());
            assert!(d <= over.blank_step() * 1.6, "{d}");
        }
    }

    #[test]
    fn empty_render_is_empty() {
        assert!(render(&[], &ScanParams::default()).points.is_empty());
    }
}
