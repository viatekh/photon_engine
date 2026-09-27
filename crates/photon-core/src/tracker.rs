//! Object permanence: follow shapes from frame to frame so the planner can commit to them.
//!
//! Every frame, each shape (paths sharing a group) is matched to a known object by predicted
//! position, size, length and colour. Matched shapes keep the object's id. That gives:
//! * **commitment** - objects already on screen keep priority (see the planner);
//! * **confirmation** - a brand-new object must be seen for `confirm_frames` frames before it is
//!   drawn, so one-frame noise never reaches the laser;
//! * **holding** - an object that was being drawn and briefly disappears from the tracing is
//!   held (at its predicted position) for up to `hold_frames` frames instead of blinking;
//! * **stable seams** - closed shapes keep their winding and start point, so the seam doesn't
//!   crawl around the shape frame to frame.

use crate::geom::{Path, Rgb, Vec2, NO_GROUP};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;

/// Tracked paths get group `TRACK_GROUP_BASE + id`.
pub const TRACK_GROUP_BASE: u32 = 0x4000_0000;

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct TrackingParams {
    pub enabled: bool,
    /// Frames a new object must be seen before it is drawn (1 = immediately).
    pub confirm_frames: u32,
    /// Frames a drawn object is held after it disappears from the tracing.
    pub hold_frames: u32,
}

impl Default for TrackingParams {
    fn default() -> Self {
        Self { enabled: true, confirm_frames: 2, hold_frames: 2 }
    }
}

#[derive(Clone, Debug)]
struct Track {
    id: u32,
    centre: Vec2,
    velocity: Vec2,
    length: f32,
    size: f32,
    colour: Rgb,
    /// Consecutive frames seen.
    age: u32,
    /// Consecutive frames missing.
    missed: u32,
    drawn_last: bool,
    /// Last geometry, for holding through dropouts.
    paths: Vec<Path>,
    /// Seam (start point) of each closed path last frame, for stable seams.
    seams: Vec<Vec2>,
}

/// What the planner needs to know about a path's object.
#[derive(Clone, Copy, Debug, Default)]
pub struct TrackInfo {
    pub age: u32,
    /// Drawn last frame: this object by identity (it may have moved), or its spot spatially.
    pub drawn_last: bool,
    /// Drawn last frame as the same object (identity match), regardless of position.
    pub drawn_by_identity: bool,
    pub confirmed: bool,
    pub held: bool,
    /// Number of paths making up this object this frame.
    pub paths: usize,
}

#[derive(Default)]
pub struct Tracker {
    tracks: Vec<Track>,
    next_id: u32,
    info: HashMap<u32, TrackInfo>,
    /// Grid cells the laser drew last frame (spatial permanence).
    drawn_cells: std::collections::HashSet<(i32, i32)>,
}

/// Grid cell size for spatial permanence (field is 2.0 wide -> 100 cells).
const CELL: f32 = 0.02;

fn cells_of(paths: &[Path]) -> Vec<(i32, i32)> {
    let mut out = Vec::new();
    for p in paths {
        let mut pts = p.points.clone();
        if p.closed && pts.len() > 2 {
            pts.push(pts[0]);
        }
        for w in pts.windows(2) {
            let n = (w[0].distance(w[1]) / (CELL * 0.5)).ceil().max(1.0) as usize;
            for k in 0..n {
                let v = w[0].lerp(w[1], k as f32 / n as f32);
                out.push(((v.x / CELL).floor() as i32, (v.y / CELL).floor() as i32));
            }
        }
    }
    out
}

struct Shape {
    paths: Vec<Path>,
    centre: Vec2,
    length: f32,
    size: f32,
    colour: Rgb,
}

fn describe(paths: Vec<Path>) -> Shape {
    let length: f32 = paths.iter().map(|p| p.length()).sum::<f32>().max(1e-6);
    let mut centre = Vec2::ZERO;
    let mut colour = Rgb::BLACK;
    let (mut lo, mut hi) = (Vec2::new(f32::MAX, f32::MAX), Vec2::new(f32::MIN, f32::MIN));
    for p in &paths {
        let l = p.length();
        centre = centre + p.centroid() * l;
        colour = Rgb::new(colour.r + p.color.r * l, colour.g + p.color.g * l, colour.b + p.color.b * l);
        let (a, b) = p.bounds();
        lo = Vec2::new(lo.x.min(a.x), lo.y.min(a.y));
        hi = Vec2::new(hi.x.max(b.x), hi.y.max(b.y));
    }
    Shape {
        centre: centre * (1.0 / length),
        colour: colour.scale(1.0 / length),
        size: lo.distance(hi),
        length,
        paths,
    }
}

impl Tracker {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn info(&self, track: u32) -> TrackInfo {
        self.info.get(&track).copied().unwrap_or_default()
    }

    /// Assign object ids to this frame's paths (in laser space); returns the paths plus any held
    /// objects. Every returned path has `track` set.
    pub fn update(&mut self, paths: Vec<Path>, params: &TrackingParams) -> Vec<Path> {
        // Group paths into shapes.
        let mut groups: Vec<Vec<Path>> = Vec::new();
        let mut index: HashMap<u32, usize> = HashMap::new();
        for p in paths {
            if p.group == NO_GROUP {
                groups.push(vec![p]);
            } else if let Some(&i) = index.get(&p.group) {
                groups[i].push(p);
            } else {
                index.insert(p.group, groups.len());
                groups.push(vec![p]);
            }
        }
        let shapes: Vec<Shape> = groups.into_iter().map(describe).collect();

        // Candidate matches, cheapest first; greedy one-to-one assignment.
        let mut candidates: Vec<(f32, usize, usize)> = Vec::new();
        for (si, s) in shapes.iter().enumerate() {
            for (ti, t) in self.tracks.iter().enumerate() {
                let predicted = t.centre + t.velocity;
                let scale = (0.5 * s.size.max(t.size)).max(0.06);
                let dist = s.centre.distance(predicted) / scale;
                let len_ratio = (s.length / t.length.max(1e-6)).ln().abs();
                let dc = (s.colour.r - t.colour.r).abs() + (s.colour.g - t.colour.g).abs() + (s.colour.b - t.colour.b).abs();
                if dist > 1.0 || len_ratio > 0.7 || dc > 0.9 {
                    continue;
                }
                candidates.push((dist + len_ratio + 0.5 * dc, si, ti));
            }
        }
        candidates.sort_by(|a, b| a.0.total_cmp(&b.0));
        let mut shape_track: Vec<Option<usize>> = vec![None; shapes.len()];
        let mut track_taken = vec![false; self.tracks.len()];
        for (_, si, ti) in candidates {
            if shape_track[si].is_none() && !track_taken[ti] {
                shape_track[si] = Some(ti);
                track_taken[ti] = true;
            }
        }

        let mut next_tracks: Vec<Track> = Vec::new();
        let mut out: Vec<Path> = Vec::new();
        self.info.clear();

        for (si, shape) in shapes.into_iter().enumerate() {
            // Spatial permanence: how much of this shape lies where the laser drew last frame.
            // A piece that split off a drawn object (or was re-traced differently) is still
            // "on screen" even though it is a new object by identity.
            let cells = cells_of(&shape.paths);
            let near = |&(x, y): &(i32, i32)| {
                (-1..=1).any(|dx| (-1..=1).any(|dy| self.drawn_cells.contains(&(x + dx, y + dy))))
            };
            let coverage = cells.iter().filter(|c| near(c)).count() as f32 / cells.len().max(1) as f32;
            let on_screen = coverage >= 0.6;
            let (id, age, drawn_last, velocity, old_seams) = match shape_track[si] {
                Some(ti) => {
                    let t = &self.tracks[ti];
                    let v = (shape.centre - t.centre) * 0.5 + t.velocity * 0.5;
                    (t.id, t.age + 1, t.drawn_last, v, t.seams.clone())
                }
                None => {
                    self.next_id += 1;
                    (self.next_id, 1, false, Vec2::ZERO, Vec::new())
                }
            };
            let mut paths = shape.paths;
            let seams = stabilise(&mut paths, &old_seams);
            for p in &mut paths {
                p.track = id;
                // One shape per object, with a group id that can't collide with the tracer's.
                p.group = TRACK_GROUP_BASE + id;
            }
            let by_identity = drawn_last;
            let drawn_last = drawn_last || on_screen;
            self.info.insert(
                id,
                TrackInfo {
                    age,
                    drawn_last,
                    drawn_by_identity: by_identity,
                    confirmed: age >= params.confirm_frames.max(1) || on_screen,
                    held: false,
                    paths: paths.len(),
                },
            );
            out.extend(paths.iter().cloned());
            next_tracks.push(Track {
                id,
                centre: shape.centre,
                velocity,
                length: shape.length,
                size: shape.size,
                colour: shape.colour,
                age,
                missed: 0,
                drawn_last,
                paths,
                seams,
            });
        }

        // Objects that vanished: hold drawn ones for a few frames at their predicted position -
        // unless this frame's tracing already covers that spot (then it was just re-identified,
        // and holding it would draw the same lines twice).
        let current: std::collections::HashSet<(i32, i32)> = cells_of(&out).into_iter().collect();
        for (ti, t) in self.tracks.iter().enumerate() {
            if track_taken[ti] || !t.drawn_last || t.missed >= params.hold_frames {
                continue;
            }
            let cells = cells_of(&t.paths);
            let covered = cells
                .iter()
                .filter(|&&(x, y)| (-1..=1).any(|dx| (-1..=1).any(|dy| current.contains(&(x + dx, y + dy)))))
                .count();
            if covered as f32 >= 0.5 * cells.len() as f32 {
                continue;
            }
            let shift = t.velocity;
            let held: Vec<Path> = t
                .paths
                .iter()
                .map(|p| {
                    let mut q = p.clone();
                    for v in &mut q.points {
                        *v = *v + shift;
                    }
                    q
                })
                .collect();
            self.info.insert(
                t.id,
                TrackInfo {
                    age: t.age,
                    drawn_last: true,
                    drawn_by_identity: true,
                    confirmed: true,
                    held: true,
                    paths: held.len(),
                },
            );
            out.extend(held.iter().cloned());
            next_tracks.push(Track {
                centre: t.centre + shift,
                missed: t.missed + 1,
                paths: held,
                ..t.clone()
            });
        }
        self.tracks = next_tracks;
        out
    }

    /// Fraction (0..1) of these paths lying where the laser drew last frame.
    pub fn coverage(&self, paths: &[Path]) -> f32 {
        let cells = cells_of(paths);
        if cells.is_empty() {
            return 0.0;
        }
        let near = cells
            .iter()
            .filter(|&&(x, y)| (-1..=1).any(|dx| (-1..=1).any(|dy| self.drawn_cells.contains(&(x + dx, y + dy)))))
            .count();
        near as f32 / cells.len() as f32
    }

    /// Tell the tracker which objects were actually drawn this frame.
    pub fn drawn(&mut self, drawn: &[Path]) {
        self.drawn_cells = cells_of(drawn).into_iter().collect();
        let ids: std::collections::HashSet<u32> = drawn.iter().map(|p| p.track).collect();
        for t in &mut self.tracks {
            t.drawn_last = ids.contains(&t.id);
        }
    }
}

/// Consistent winding (counter-clockwise) and a seam near last frame's, for closed paths.
/// Returns the seams chosen.
fn stabilise(paths: &mut [Path], old_seams: &[Vec2]) -> Vec<Vec2> {
    let mut seams = Vec::new();
    for p in paths.iter_mut().filter(|p| p.closed && p.points.len() > 2) {
        let area: f32 = (0..p.points.len())
            .map(|i| {
                let (a, b) = (p.points[i], p.points[(i + 1) % p.points.len()]);
                a.x * b.y - b.x * a.y
            })
            .sum();
        if area < 0.0 {
            p.points.reverse();
        }
        // Seam: the vertex nearest an old seam of this object, else the topmost vertex.
        let target = old_seams
            .iter()
            .copied()
            .min_by(|a, b| {
                let da = p.points.iter().map(|v| v.distance(*a)).fold(f32::MAX, f32::min);
                let db = p.points.iter().map(|v| v.distance(*b)).fold(f32::MAX, f32::min);
                da.total_cmp(&db)
            })
            .unwrap_or_else(|| {
                let c = p.centroid();
                Vec2::new(c.x, c.y + 10.0)
            });
        let k = (0..p.points.len())
            .min_by(|&a, &b| p.points[a].distance(target).total_cmp(&p.points[b].distance(target)))
            .unwrap();
        p.points.rotate_left(k);
        p.fixed_start = true;
        seams.push(p.points[0]);
    }
    seams
}

#[cfg(test)]
mod tests {
    use super::*;

    fn ring(cx: f32, cy: f32, r: f32, phase: f32, group: u32) -> Path {
        let pts = (0..24)
            .map(|i| {
                let a = phase + i as f32 / 24.0 * std::f32::consts::TAU;
                Vec2::new(cx + r * a.cos(), cy + r * a.sin())
            })
            .collect();
        let mut p = Path::new(pts, true, Rgb::new(0.0, 1.0, 0.0));
        p.group = group;
        p
    }

    #[test]
    fn same_object_keeps_its_id_while_moving() {
        let mut t = Tracker::new();
        let params = TrackingParams::default();
        let mut ids = Vec::new();
        for f in 0..10 {
            // Group ids change every frame, as the tracer's labels do.
            let out = t.update(vec![ring(-0.5 + f as f32 * 0.03, 0.0, 0.2, 0.0, 7 + f)], &params);
            ids.push(out[0].track);
            t.drawn(&out);
        }
        assert!(ids.windows(2).all(|w| w[0] == w[1]), "{ids:?}");
        assert!(t.info(ids[0]).confirmed);
    }

    #[test]
    fn new_objects_need_confirmation_and_drawn_ones_are_held() {
        let mut t = Tracker::new();
        let params = TrackingParams::default();
        let out = t.update(vec![ring(0.0, 0.0, 0.2, 0.0, 1)], &params);
        assert!(!t.info(out[0].track).confirmed, "one frame is not enough");
        let out = t.update(vec![ring(0.0, 0.0, 0.2, 0.0, 1)], &params);
        let id = out[0].track;
        assert!(t.info(id).confirmed);
        t.drawn(&out);
        // Tracing loses it for one frame: it is held.
        let out = t.update(vec![], &params);
        assert_eq!(out.len(), 1);
        assert!(t.info(id).held);
        t.drawn(&out);
        // Comes back: same object.
        let out = t.update(vec![ring(0.0, 0.0, 0.2, 0.0, 3)], &params);
        assert_eq!(out[0].track, id);
        // Gone for good: released after hold_frames.
        t.drawn(&out);
        let mut n = 0;
        for _ in 0..5 {
            n = t.update(vec![], &params).len();
        }
        assert_eq!(n, 0);
    }

    #[test]
    fn piece_splitting_off_a_drawn_object_stays_on_screen() {
        let mut t = Tracker::new();
        let params = TrackingParams::default();
        // Two touching rings traced as one shape, drawn for a while.
        for f in 0..3 {
            let mut a = ring(0.0, 0.0, 0.2, 0.0, 1 + f);
            let mut b = ring(0.35, 0.0, 0.2, 0.0, 1 + f);
            a.group = 1;
            b.group = 1;
            let out = t.update(vec![a, b], &params);
            t.drawn(&out);
        }
        // Now traced as two separate shapes: the smaller-match one is a "new" object by id,
        // but it is where the laser was drawing, so it is confirmed and committed at once.
        let out = t.update(vec![ring(0.0, 0.0, 0.2, 0.0, 5), ring(0.35, 0.0, 0.2, 0.0, 6)], &params);
        for p in &out {
            let info = t.info(p.track);
            assert!(info.confirmed && info.drawn_last, "{info:?}");
        }
    }

    #[test]
    fn seam_stays_put_even_if_tracing_starts_elsewhere() {
        let mut t = Tracker::new();
        let params = TrackingParams::default();
        let first = t.update(vec![ring(0.0, 0.0, 0.3, 0.0, 1)], &params)[0].points[0];
        for phase in [0.9f32, 2.1, 4.0] {
            // Same ring, traced starting from a different point (and winding) each frame.
            let mut r = ring(0.0, 0.0, 0.3, phase, 1);
            if phase > 2.0 {
                r.points.reverse();
            }
            let p = t.update(vec![r], &params)[0].clone();
            assert!(p.points[0].distance(first) < 0.1, "seam moved to {:?}", p.points[0]);
            // Counter-clockwise.
            let area: f32 = (0..p.points.len())
                .map(|i| {
                    let (a, b) = (p.points[i], p.points[(i + 1) % p.points.len()]);
                    a.x * b.y - b.x * a.y
                })
                .sum();
            assert!(area > 0.0);
        }
    }
}
