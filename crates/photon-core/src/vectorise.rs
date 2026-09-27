//! Turning a raster frame into laser paths.
//!
//! Two tracing modes:
//! * **Outline** traces the boundary of every bright region (marching squares, sub-pixel).
//!   Robust for any content, but a thin line becomes a thin loop, which costs twice the scan time.
//! * **Centreline** thins bright regions to a 1px skeleton (Zhang-Suen) and follows it.
//!   Ideal for laser-style content (thin lines on black); filled shapes collapse to their "spine".

use crate::geom::{Path, Rgb, Vec2};
use crate::image::WorkImage;
use serde::{Deserialize, Serialize};
use std::collections::HashMap;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum TraceMode {
    Outline,
    Centreline,
}

/// How the (usually 16:9) input maps onto the (square) laser field.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum FitMode {
    /// Keep aspect ratio; the longest side spans the full laser field.
    Fit,
    /// Stretch both axes to fill the laser field.
    Stretch,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct VectoriseParams {
    pub mode: TraceMode,
    pub fit: FitMode,
    /// Brightness (0..1) above which a pixel counts as "on".
    pub threshold: f32,
    /// Longest side of the working image in pixels. Higher = more detail, slower, noisier.
    pub resolution: usize,
    /// Douglas-Peucker tolerance in working-image pixels.
    pub simplify_px: f32,
    /// Paths shorter than this (working-image pixels) are dropped as noise.
    pub min_length_px: f32,
    /// Scale colours so the brightest channel is 1.0 (lasers look best fully driven).
    pub normalise_colour: bool,
}

impl Default for VectoriseParams {
    fn default() -> Self {
        Self {
            mode: TraceMode::Centreline,
            fit: FitMode::Fit,
            threshold: 0.3,
            resolution: 240,
            simplify_px: 0.6,
            min_length_px: 6.0,
            normalise_colour: true,
        }
    }
}

pub fn vectorise(img: &WorkImage, params: &VectoriseParams) -> Vec<Path> {
    if img.width < 2 || img.height < 2 {
        return Vec::new();
    }
    let raw = match params.mode {
        TraceMode::Outline => trace_outlines(img, params.threshold),
        TraceMode::Centreline => trace_centrelines(img, params.threshold),
    };
    let mapper = PixelMapper::new(img.width, img.height, params.fit);
    raw.into_iter()
        .filter_map(|(pts, closed)| {
            let pts = simplify(&pts, closed, params.simplify_px);
            let min_pts = if closed { 3 } else { 2 };
            if pts.len() < min_pts {
                return None;
            }
            let pixel_path = Path::new(pts, closed, Rgb::BLACK);
            if pixel_path.length() < params.min_length_px {
                return None;
            }
            let mut color = sample_colour(img, &pixel_path.points, params.threshold);
            if params.normalise_colour {
                let m = color.max_channel();
                if m > 0.0 {
                    color = color.scale(1.0 / m);
                }
            }
            if color.is_black() {
                return None;
            }
            let points = pixel_path.points.iter().map(|&p| mapper.map(p)).collect();
            Some(Path::new(points, closed, color))
        })
        .collect()
}

/// Maps working-image pixel coordinates (y down) to laser space (y up).
pub struct PixelMapper {
    cx: f32,
    cy: f32,
    sx: f32,
    sy: f32,
}

impl PixelMapper {
    pub fn new(width: usize, height: usize, fit: FitMode) -> Self {
        let (w, h) = (width as f32, height as f32);
        let (sx, sy) = match fit {
            FitMode::Stretch => (2.0 / w, 2.0 / h),
            FitMode::Fit => {
                let s = 2.0 / w.max(h);
                (s, s)
            }
        };
        Self { cx: w / 2.0, cy: h / 2.0, sx, sy }
    }

    pub fn map(&self, p: Vec2) -> Vec2 {
        Vec2::new((p.x - self.cx) * self.sx, (self.cy - p.y) * self.sy)
    }
}

/// Average colour of the "on" pixels near the path's vertices.
fn sample_colour(img: &WorkImage, pts: &[Vec2], threshold: f32) -> Rgb {
    let mut sum = Rgb::BLACK;
    let mut n = 0usize;
    for p in pts {
        // Look at the 2x2 pixels around the vertex; only count the lit ones.
        let x0 = (p.x - 0.5).floor().max(0.0) as usize;
        let y0 = (p.y - 0.5).floor().max(0.0) as usize;
        for y in y0..(y0 + 2).min(img.height) {
            for x in x0..(x0 + 2).min(img.width) {
                let c = img.get(x, y);
                if c.max_channel() >= threshold {
                    sum = Rgb::new(sum.r + c.r, sum.g + c.g, sum.b + c.b);
                    n += 1;
                }
            }
        }
    }
    if n == 0 {
        return Rgb::BLACK;
    }
    sum.scale(1.0 / n as f32)
}

// ---------------------------------------------------------------------------------------------
// Outline tracing: marching squares.

/// Returns closed loops in pixel coordinates (pixel centres at x+0.5).
fn trace_outlines(img: &WorkImage, iso: f32) -> Vec<(Vec<Vec2>, bool)> {
    // Pad with a black border so every contour closes.
    let w = img.width + 2;
    let h = img.height + 2;
    let mut v = vec![0.0f32; w * h];
    for y in 0..img.height {
        for x in 0..img.width {
            v[(y + 1) * w + x + 1] = img.value(x, y);
        }
    }
    let val = |x: usize, y: usize| v[y * w + x];
    let inside = |x: usize, y: usize| val(x, y) >= iso;

    // Edge ids: horizontal edge from grid (x,y) to (x+1,y) = 2*(y*w+x); vertical (x,y)-(x,y+1) = +1.
    let h_edge = |x: usize, y: usize| 2 * (y * w + x);
    let v_edge = |x: usize, y: usize| 2 * (y * w + x) + 1;

    let mut positions: HashMap<usize, Vec2> = HashMap::new();
    let mut links: HashMap<usize, [usize; 2]> = HashMap::new();
    let add_link = |a: usize, b: usize, links: &mut HashMap<usize, [usize; 2]>| {
        for (from, to) in [(a, b), (b, a)] {
            let e = links.entry(from).or_insert([usize::MAX; 2]);
            if e[0] == usize::MAX {
                e[0] = to;
            } else {
                e[1] = to;
            }
        }
    };

    let interp = |a: f32, b: f32| {
        let d = b - a;
        if d.abs() < 1e-6 { 0.5 } else { ((iso - a) / d).clamp(0.0, 1.0) }
    };

    for y in 0..h - 1 {
        for x in 0..w - 1 {
            let tl = inside(x, y);
            let tr = inside(x + 1, y);
            let br = inside(x + 1, y + 1);
            let bl = inside(x, y + 1);
            let case = (tl as u8) << 3 | (tr as u8) << 2 | (br as u8) << 1 | bl as u8;
            if case == 0 || case == 15 {
                continue;
            }
            // Edge ids and their crossing positions (grid units; pixel centre offset applied later).
            let t = h_edge(x, y);
            let b = h_edge(x, y + 1);
            let l = v_edge(x, y);
            let r = v_edge(x + 1, y);
            let (fx, fy) = (x as f32, y as f32);
            let mut pos = |id: usize| {
                positions.entry(id).or_insert_with(|| {
                    if id == t {
                        Vec2::new(fx + interp(val(x, y), val(x + 1, y)), fy)
                    } else if id == b {
                        Vec2::new(fx + interp(val(x, y + 1), val(x + 1, y + 1)), fy + 1.0)
                    } else if id == l {
                        Vec2::new(fx, fy + interp(val(x, y), val(x, y + 1)))
                    } else {
                        Vec2::new(fx + 1.0, fy + interp(val(x + 1, y), val(x + 1, y + 1)))
                    }
                });
            };
            let centre_in = || {
                (val(x, y) + val(x + 1, y) + val(x + 1, y + 1) + val(x, y + 1)) / 4.0 >= iso
            };
            let segs: &[(usize, usize)] = match case {
                1 => &[(l, b)],
                2 => &[(b, r)],
                3 => &[(l, r)],
                4 => &[(t, r)],
                5 => {
                    if centre_in() { &[(l, t), (b, r)] } else { &[(t, r), (l, b)] }
                }
                6 => &[(t, b)],
                7 => &[(l, t)],
                8 => &[(l, t)],
                9 => &[(t, b)],
                10 => {
                    if centre_in() { &[(t, r), (l, b)] } else { &[(l, t), (b, r)] }
                }
                11 => &[(t, r)],
                12 => &[(l, r)],
                13 => &[(b, r)],
                14 => &[(l, b)],
                _ => &[],
            };
            for &(a, c) in segs {
                pos(a);
                pos(c);
                add_link(a, c, &mut links);
            }
        }
    }

    // Walk the loops. Every crossing has exactly two neighbours, so each component is a cycle.
    let mut visited: HashMap<usize, bool> = HashMap::with_capacity(links.len());
    let mut keys: Vec<usize> = links.keys().copied().collect();
    keys.sort_unstable(); // deterministic output
    let mut loops = Vec::new();
    for start in keys {
        if visited.contains_key(&start) {
            continue;
        }
        let mut pts = Vec::new();
        let mut prev = usize::MAX;
        let mut cur = start;
        loop {
            visited.insert(cur, true);
            // Padded grid coordinate (x) -> image pixel coordinate (x - 1) -> centre (+0.5).
            let p = positions[&cur];
            pts.push(Vec2::new(p.x - 0.5, p.y - 0.5));
            let [a, b] = links[&cur];
            let next = if a != prev { a } else { b };
            if next == usize::MAX || next == start || visited.contains_key(&next) {
                break;
            }
            prev = cur;
            cur = next;
        }
        if pts.len() >= 3 {
            loops.push((pts, true));
        }
    }
    loops
}

// ---------------------------------------------------------------------------------------------
// Centreline tracing: Zhang-Suen thinning, then follow the skeleton.

fn trace_centrelines(img: &WorkImage, threshold: f32) -> Vec<(Vec<Vec2>, bool)> {
    let w = img.width + 2;
    let h = img.height + 2;
    let mut m = vec![false; w * h];
    for y in 0..img.height {
        for x in 0..img.width {
            m[(y + 1) * w + x + 1] = img.value(x, y) >= threshold;
        }
    }
    thin(&mut m, w, h);

    // m-adjacency: a diagonal only counts if neither shared 4-neighbour is set.
    // This removes the little triangles 8-connectivity creates at staircase corners.
    let neighbours = |m: &[bool], i: usize| -> Vec<usize> {
        let (x, y) = ((i % w) as isize, (i / w) as isize);
        let at = |dx: isize, dy: isize| ((y + dy) as usize) * w + (x + dx) as usize;
        let mut out = Vec::with_capacity(4);
        for (dx, dy) in [(1, 0), (-1, 0), (0, 1), (0, -1)] {
            if m[at(dx, dy)] {
                out.push(at(dx, dy));
            }
        }
        for (dx, dy) in [(1, 1), (1, -1), (-1, 1), (-1, -1)] {
            if m[at(dx, dy)] && !m[at(dx, 0)] && !m[at(0, dy)] {
                out.push(at(dx, dy));
            }
        }
        out
    };

    let set: Vec<usize> = (0..w * h).filter(|&i| m[i]).collect();
    let adj: HashMap<usize, Vec<usize>> = set.iter().map(|&i| (i, neighbours(&m, i))).collect();
    let edge_key = |a: usize, b: usize| if a < b { (a, b) } else { (b, a) };
    let mut used: std::collections::HashSet<(usize, usize)> = Default::default();
    let to_pt = |i: usize| Vec2::new((i % w) as f32 - 0.5, (i / w) as f32 - 0.5);

    let mut out = Vec::new();

    // Isolated single pixels become tiny dots; drop them (min length filter would anyway).
    // Walk from every endpoint / junction along each unused edge.
    for &node in &set {
        if adj[&node].len() == 2 {
            continue;
        }
        for &first in &adj[&node] {
            if used.contains(&edge_key(node, first)) {
                continue;
            }
            used.insert(edge_key(node, first));
            let mut pts = vec![to_pt(node), to_pt(first)];
            let (mut prev, mut cur) = (node, first);
            while adj[&cur].len() == 2 {
                let next = adj[&cur].iter().copied().find(|&n| n != prev);
                let Some(next) = next else { break };
                if !used.insert(edge_key(cur, next)) {
                    break;
                }
                pts.push(to_pt(next));
                prev = cur;
                cur = next;
            }
            out.push((pts, false));
        }
    }

    // Whatever is left are pure loops (every pixel has two neighbours).
    for &start in &set {
        if adj[&start].len() != 2 {
            continue;
        }
        let first = adj[&start][0];
        if used.contains(&edge_key(start, first)) {
            continue;
        }
        used.insert(edge_key(start, first));
        let mut pts = vec![to_pt(start)];
        let (mut prev, mut cur) = (start, first);
        let mut closed = false;
        loop {
            if cur == start {
                closed = true;
                break;
            }
            pts.push(to_pt(cur));
            let next = adj[&cur].iter().copied().find(|&n| n != prev);
            let Some(next) = next else { break };
            if !used.insert(edge_key(cur, next)) {
                closed = next == start;
                break;
            }
            prev = cur;
            cur = next;
        }
        out.push((pts, closed));
    }
    out
}

/// Zhang-Suen thinning in place. `m` must have a false border.
fn thin(m: &mut [bool], w: usize, h: usize) {
    let mut to_clear = Vec::new();
    loop {
        let mut changed = false;
        for pass in 0..2 {
            to_clear.clear();
            for y in 1..h - 1 {
                for x in 1..w - 1 {
                    let i = y * w + x;
                    if !m[i] {
                        continue;
                    }
                    // P2..P9 clockwise from north.
                    let p = [
                        m[i - w],
                        m[i - w + 1],
                        m[i + 1],
                        m[i + w + 1],
                        m[i + w],
                        m[i + w - 1],
                        m[i - 1],
                        m[i - w - 1],
                    ];
                    let b = p.iter().filter(|&&v| v).count();
                    if !(2..=6).contains(&b) {
                        continue;
                    }
                    let a = (0..8).filter(|&k| !p[k] && p[(k + 1) % 8]).count();
                    if a != 1 {
                        continue;
                    }
                    let (p2, p4, p6, p8) = (p[0], p[2], p[4], p[6]);
                    let ok = if pass == 0 {
                        !(p2 && p4 && p6) && !(p4 && p6 && p8)
                    } else {
                        !(p2 && p4 && p8) && !(p2 && p6 && p8)
                    };
                    if ok {
                        to_clear.push(i);
                    }
                }
            }
            for &i in &to_clear {
                m[i] = false;
            }
            changed |= !to_clear.is_empty();
        }
        if !changed {
            break;
        }
    }
}

// ---------------------------------------------------------------------------------------------
// Simplification.

/// Douglas-Peucker. For closed paths the first point is not repeated at the end.
pub fn simplify(pts: &[Vec2], closed: bool, epsilon: f32) -> Vec<Vec2> {
    if pts.len() < 3 || epsilon <= 0.0 {
        return pts.to_vec();
    }
    if closed {
        // Split at the point farthest from the start so both halves are open polylines.
        let far = (1..pts.len())
            .max_by(|&a, &b| pts[0].distance(pts[a]).total_cmp(&pts[0].distance(pts[b])))
            .unwrap();
        let mut first: Vec<Vec2> = pts[..=far].to_vec();
        let mut second: Vec<Vec2> = pts[far..].to_vec();
        second.push(pts[0]);
        first = rdp(&first, epsilon);
        second = rdp(&second, epsilon);
        first.pop();
        second.pop();
        first.extend(second);
        return first;
    }
    rdp(pts, epsilon)
}

fn rdp(pts: &[Vec2], eps: f32) -> Vec<Vec2> {
    if pts.len() < 3 {
        return pts.to_vec();
    }
    let mut keep = vec![false; pts.len()];
    keep[0] = true;
    keep[pts.len() - 1] = true;
    let mut stack = vec![(0usize, pts.len() - 1)];
    while let Some((s, e)) = stack.pop() {
        let (a, b) = (pts[s], pts[e]);
        let mut best = (0.0f32, 0usize);
        for (i, &p) in pts.iter().enumerate().take(e).skip(s + 1) {
            let d = point_segment_distance(p, a, b);
            if d > best.0 {
                best = (d, i);
            }
        }
        if best.0 > eps {
            keep[best.1] = true;
            stack.push((s, best.1));
            stack.push((best.1, e));
        }
    }
    pts.iter().zip(keep).filter(|(_, k)| *k).map(|(p, _)| *p).collect()
}

pub fn point_segment_distance(p: Vec2, a: Vec2, b: Vec2) -> f32 {
    let ab = b - a;
    let len2 = ab.dot(ab);
    if len2 <= 1e-12 {
        return p.distance(a);
    }
    let t = ((p - a).dot(ab) / len2).clamp(0.0, 1.0);
    p.distance(a + ab * t)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn image_from(rows: &[&str]) -> WorkImage {
        let h = rows.len();
        let w = rows[0].len();
        let mut img = WorkImage::new(w, h);
        for (y, row) in rows.iter().enumerate() {
            for (x, c) in row.chars().enumerate() {
                if c == '#' {
                    img.pixels[y * w + x] = Rgb::new(0.0, 1.0, 0.0);
                }
            }
        }
        img
    }

    fn filled_square(size: usize, pad: usize) -> WorkImage {
        let n = size + 2 * pad;
        let mut img = WorkImage::new(n, n);
        for y in pad..pad + size {
            for x in pad..pad + size {
                img.pixels[y * n + x] = Rgb::WHITE;
            }
        }
        img
    }

    #[test]
    fn outline_of_square_is_one_closed_loop() {
        let img = filled_square(10, 5);
        let params = VectoriseParams { mode: TraceMode::Outline, ..Default::default() };
        let paths = vectorise(&img, &params);
        assert_eq!(paths.len(), 1);
        assert!(paths[0].closed);
        // Simplified to (roughly) the 4 corners, possibly with cut corners.
        assert!(paths[0].points.len() <= 8, "{:?}", paths[0].points);
        // Perimeter of a 10px square in a 20px image mapped to width 2.0 => ~ 4 * 10 * 0.1 = 4.0
        assert!((paths[0].length() - 4.0).abs() < 0.3, "len {}", paths[0].length());
    }

    #[test]
    fn outline_ring_gives_two_loops() {
        let img = image_from(&[
            "........",
            ".######.",
            ".#....#.",
            ".#....#.",
            ".#....#.",
            ".######.",
            "........",
        ]);
        let params = VectoriseParams {
            mode: TraceMode::Outline,
            min_length_px: 1.0,
            ..Default::default()
        };
        let paths = vectorise(&img, &params);
        assert_eq!(paths.len(), 2);
    }

    #[test]
    fn centreline_of_thick_horizontal_line_is_one_open_path() {
        let img = image_from(&[
            "..................",
            "..................",
            "..##############..",
            "..##############..",
            "..##############..",
            "..................",
            "..................",
        ]);
        let params = VectoriseParams {
            mode: TraceMode::Centreline,
            min_length_px: 3.0,
            ..Default::default()
        };
        let paths = vectorise(&img, &params);
        assert_eq!(paths.len(), 1, "{paths:?}");
        assert!(!paths[0].closed);
        // Colour is sampled from the image and normalised.
        assert_eq!(paths[0].color, Rgb::new(0.0, 1.0, 0.0));
    }

    #[test]
    fn centreline_of_thin_ring_is_closed() {
        let img = image_from(&[
            "..........",
            "..######..",
            ".#......#.",
            ".#......#.",
            ".#......#.",
            "..######..",
            "..........",
        ]);
        let params = VectoriseParams {
            mode: TraceMode::Centreline,
            min_length_px: 3.0,
            ..Default::default()
        };
        let paths = vectorise(&img, &params);
        assert_eq!(paths.len(), 1, "{paths:?}");
        assert!(paths[0].closed);
    }

    #[test]
    fn centreline_cross_splits_at_junction() {
        let img = image_from(&[
            ".........",
            "....#....",
            "....#....",
            "....#....",
            ".#######.",
            "....#....",
            "....#....",
            "....#....",
            ".........",
        ]);
        let params = VectoriseParams {
            mode: TraceMode::Centreline,
            min_length_px: 1.0,
            simplify_px: 0.0,
            ..Default::default()
        };
        let paths = vectorise(&img, &params);
        assert_eq!(paths.len(), 4, "{paths:?}");
    }

    #[test]
    fn simplify_closed_square_keeps_corners() {
        let mut pts = Vec::new();
        for i in 0..10 {
            pts.push(Vec2::new(i as f32, 0.0));
        }
        for i in 0..10 {
            pts.push(Vec2::new(10.0, i as f32));
        }
        for i in 0..10 {
            pts.push(Vec2::new(10.0 - i as f32, 10.0));
        }
        for i in 0..10 {
            pts.push(Vec2::new(0.0, 10.0 - i as f32));
        }
        let s = simplify(&pts, true, 0.1);
        assert_eq!(s.len(), 4, "{s:?}");
    }

    #[test]
    fn empty_image_has_no_paths() {
        let img = WorkImage::new(32, 18);
        for mode in [TraceMode::Outline, TraceMode::Centreline] {
            let params = VectoriseParams { mode, ..Default::default() };
            assert!(vectorise(&img, &params).is_empty());
        }
    }
}
