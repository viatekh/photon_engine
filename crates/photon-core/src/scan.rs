//! Converting paths into a point stream the scanners can physically follow.
//!
//! Speeds are in laser-space units per second (the field is 2.0 wide) and dwells are in
//! microseconds, so the same settings behave the same at any point rate.

use crate::geom::{LaserPoint, Path, Rgb, Vec2};
use serde::{Deserialize, Serialize};

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct ScanParams {
    /// Points per second sent to the DAC.
    pub pps: u32,
    /// Galvo rating in kpps at ILDA 8 degrees (e.g. 30 for "30K scanners"). Speeds and dwells
    /// below are specified for a 30K scanner and scaled by `scanner_kpps / 30`, so changing
    /// this one number retunes everything for faster or slower scanners.
    pub scanner_kpps: f32,
    /// Max beam speed while lit (at 30K). Faster = more content fits, but corners round off and the
    /// line dims. Units: field widths/2 per second.
    pub lit_speed: f32,
    /// Max beam speed while blanked, jumping between shapes (at 30K).
    pub blank_speed: f32,
    /// Extra time at a full 180-degree corner (scaled down for gentler corners).
    pub corner_dwell_us: f32,
    /// Direction changes below this many degrees get no corner dwell.
    pub corner_min_angle: f32,
    /// Lit hold at the start and end of every path (lets the mirrors catch up).
    pub path_dwell_us: f32,
    /// Blanked hold before a jump (lets the colour switch off) and after (lets mirrors settle).
    pub blank_dwell_us: f32,
    /// Paths whose start is this close to the previous end are joined without blanking.
    pub join_distance: f32,
    /// Closed shapes are drawn this much past their starting point (in time at lit speed),
    /// hiding the gap mirror lag leaves at the seam.
    pub closed_overlap_us: f32,
}

impl Default for ScanParams {
    fn default() -> Self {
        Self {
            pps: 30_000,
            scanner_kpps: 30.0,
            lit_speed: 450.0,
            blank_speed: 1500.0,
            corner_dwell_us: 130.0,
            corner_min_angle: 25.0,
            path_dwell_us: 70.0,
            blank_dwell_us: 100.0,
            join_distance: 0.004,
            closed_overlap_us: 0.0,
        }
    }
}

impl ScanParams {
    /// How much faster than a 30K scanner the galvos are.
    fn scanner_factor(&self) -> f32 {
        (self.scanner_kpps / 30.0).clamp(0.1, 10.0)
    }
    /// Dwell times shrink for faster scanners (they settle sooner).
    pub fn points_for(&self, us: f32) -> usize {
        (us / self.scanner_factor() * 1e-6 * self.pps as f32).round().max(0.0) as usize
    }
    pub fn lit_step(&self) -> f32 {
        (self.lit_speed * self.scanner_factor() / self.pps.max(1) as f32).max(1e-5)
    }
    pub fn blank_step(&self) -> f32 {
        (self.blank_speed * self.scanner_factor() / self.pps.max(1) as f32).max(1e-5)
    }
    pub fn corner_points(&self) -> usize {
        self.points_for(self.corner_dwell_us)
    }
    pub fn path_dwell_points(&self) -> usize {
        self.points_for(self.path_dwell_us)
    }
    pub fn blank_dwell_points(&self) -> usize {
        self.points_for(self.blank_dwell_us)
    }

    /// Number of points a blanked jump of `distance` costs.
    pub fn blank_cost(&self, distance: f32) -> usize {
        if distance <= self.join_distance {
            0
        } else {
            2 * self.blank_dwell_points() + (distance / self.blank_step()).ceil() as usize
        }
    }
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

/// Emit the lit points for an already-oriented polyline.
fn emit_lit(pts: &[Vec2], color: Rgb, params: &ScanParams, closed: bool, out: &mut Vec<LaserPoint>) {
    let step = params.lit_step();
    let dwell = params.path_dwell_points();
    let corner = params.corner_points() as f32;
    let min_angle = params.corner_min_angle.to_radians();

    for _ in 0..dwell.max(1) {
        out.push(LaserPoint::lit(pts[0], color));
    }
    for i in 1..pts.len() {
        let (a, b) = (pts[i - 1], pts[i]);
        let n = (a.distance(b) / step).ceil().max(1.0) as usize;
        for k in 1..=n {
            out.push(LaserPoint::lit(a.lerp(b, k as f32 / n as f32), color));
        }
        // Corner dwell at interior vertices (and at the seam of a closed path).
        let next = if i + 1 < pts.len() {
            Some(pts[i + 1])
        } else if closed && pts.len() > 2 {
            Some(pts[1])
        } else {
            None
        };
        if let Some(c) = next {
            let angle = turn_angle(a, b, c);
            if angle > min_angle && i + 1 < pts.len() {
                let extra = (corner * angle / std::f32::consts::PI).round() as usize;
                for _ in 0..extra {
                    out.push(LaserPoint::lit(b, color));
                }
            }
        }
    }
    // Closed shapes: keep drawing a little past the seam so mirror lag doesn't leave a gap.
    let overlap = params.points_for(params.closed_overlap_us);
    if closed && overlap > 0 && pts.len() > 2 {
        let mut left = overlap;
        'walk: for i in 1..pts.len() {
            let (a, b) = (pts[i - 1], pts[i]);
            let n = (a.distance(b) / step).ceil().max(1.0) as usize;
            for k in 1..=n {
                out.push(LaserPoint::lit(a.lerp(b, k as f32 / n as f32), color));
                left -= 1;
                if left == 0 {
                    break 'walk;
                }
            }
        }
    }
    let end = out.last().map(|p| p.pos()).unwrap_or(pts[pts.len() - 1]);
    for _ in 0..dwell {
        out.push(LaserPoint::lit(end, color));
    }
}

/// Direction change at `b` in radians (0 = straight on, PI = full reversal).
fn turn_angle(a: Vec2, b: Vec2, c: Vec2) -> f32 {
    let (d1, d2) = (b - a, c - b);
    let (l1, l2) = (d1.length(), d2.length());
    if l1 < 1e-9 || l2 < 1e-9 {
        return 0.0;
    }
    (d1.dot(d2) / (l1 * l2)).clamp(-1.0, 1.0).acos()
}

/// Blanked move with dwell at both ends. Eases in/out to reduce mirror overshoot.
pub fn emit_blank(from: Vec2, to: Vec2, params: &ScanParams, out: &mut Vec<LaserPoint>) {
    let d = from.distance(to);
    if d <= params.join_distance {
        return;
    }
    let dwell = params.blank_dwell_points();
    for _ in 0..dwell {
        out.push(LaserPoint::blank(from));
    }
    let n = (d / params.blank_step()).ceil().max(1.0) as usize;
    for k in 1..=n {
        let t = k as f32 / n as f32;
        let eased = 0.5 - 0.5 * (std::f32::consts::PI * t).cos();
        out.push(LaserPoint::blank(from.lerp(to, eased)));
    }
    for _ in 0..dwell {
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
            if p.closed {
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
        // Eased moves peak at PI/2 times the average step.
        assert!(max_blank <= params.blank_step() * 1.6, "{max_blank}");
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
        assert_eq!(lit(&b), lit(&a) + over.points_for(300.0));
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
