//! Turning a raster frame into laser paths.
//!
//! Tracing modes:
//! * **Auto** (default) handles any content. Thin bright strokes (up to `stroke_width_px`) are
//!   found with a ridge detector and traced once along their middle; everything else (filled
//!   shapes, film, photos, fractals) is traced along its contrast edges, skipping edges that just
//!   border a stroke so lines are never drawn twice.
//! * **Edges** traces contrast edges only (Canny-style: blur, gradient, non-max suppression,
//!   hysteresis). A thin line gives two edges.
//! * **Outline** traces the boundary of every region brighter than a threshold.
//! * **Centreline** thins regions brighter than a threshold to a 1px skeleton and follows it.
//!
//! Every path gets a **group**: paths traced from the same connected piece of the image share
//! one, so the planner can treat e.g. a ring split at a crossing as one shape and never draw
//! only part of it.

use crate::geom::{Path, Rgb, Vec2};
use crate::image::WorkImage;
use serde::{Deserialize, Serialize};
use std::collections::HashMap;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum TraceMode {
    Auto,
    Edges,
    Outline,
    Centreline,
}

impl TraceMode {
    pub const ALL: [TraceMode; 4] =
        [TraceMode::Auto, TraceMode::Edges, TraceMode::Centreline, TraceMode::Outline];

    pub fn label(self) -> &'static str {
        match self {
            TraceMode::Auto => "Auto (strokes + edges)",
            TraceMode::Edges => "Edges only",
            TraceMode::Centreline => "Centreline (threshold)",
            TraceMode::Outline => "Outline (threshold)",
        }
    }

    /// Whether the mode uses edge detection (edge threshold / blur apply).
    pub fn uses_edges(self) -> bool {
        matches!(self, TraceMode::Auto | TraceMode::Edges)
    }
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
    /// Brightness (0..1) above which a pixel counts as "on" (Centreline / Outline).
    pub threshold: f32,
    /// Edge strength (brightness change across the edge, 0..1) needed to start an edge.
    pub edge_threshold: f32,
    /// Gaussian blur before edge detection, in working-image pixels. Suppresses fine texture.
    pub blur_px: f32,
    /// Widest line (working-image pixels) treated as a stroke and traced once (Auto).
    pub stroke_width_px: f32,
    /// How much brighter than both sides a stroke must be (0..1) (Auto).
    pub stroke_threshold: f32,
    /// Blur before stroke detection (Auto). Normally 0; auto detail raises it on busy content
    /// so fine texture stops registering as strokes.
    pub stroke_blur_px: f32,
    /// 0..0.95: blend each frame with the previous to calm flickering edges on noisy video.
    /// Leaves ghost trails on moving content, so keep it low.
    pub temporal_smoothing: f32,
    /// Longest side of the working image in pixels. Higher = more detail, slower, noisier.
    pub resolution: usize,
    /// Smoothing passes applied before simplifying (removes pixel staircase).
    pub smoothing: u32,
    /// Douglas-Peucker tolerance in working-image pixels.
    pub simplify_px: f32,
    /// Shapes shorter than this (working-image pixels) are dropped as noise.
    pub min_length_px: f32,
    /// Extra multiplier on the minimum length for edge-traced shapes only (set by auto detail
    /// to thin out film / texture detail without touching strokes).
    pub edge_min_length_scale: f32,
    /// Scale colours so the brightest channel is 1.0 (lasers look best fully driven).
    pub normalise_colour: bool,
}

impl Default for VectoriseParams {
    fn default() -> Self {
        Self {
            mode: TraceMode::Auto,
            fit: FitMode::Fit,
            threshold: 0.3,
            edge_threshold: 0.12,
            blur_px: 1.0,
            stroke_width_px: 3.0,
            stroke_threshold: 0.12,
            stroke_blur_px: 0.0,
            temporal_smoothing: 0.0,
            resolution: 320,
            smoothing: 2,
            simplify_px: 0.6,
            min_length_px: 6.0,
            edge_min_length_scale: 1.0,
            normalise_colour: true,
        }
    }
}

/// How a raw path was found (decides how it is coloured and weighted).
#[derive(Clone, Copy, PartialEq)]
#[repr(u8)]
enum Kind {
    /// Centre of a bright line / region (threshold modes).
    Region,
    /// Centre of a thin stroke found by the ridge detector.
    Stroke,
    /// A contrast edge.
    Edge,
}

struct Raw {
    pts: Vec<Vec2>,
    closed: bool,
    kind: Kind,
    group: u32,
}

pub fn vectorise(img: &WorkImage, params: &VectoriseParams) -> Vec<Path> {
    if img.width < 3 || img.height < 3 {
        return Vec::new();
    }
    let (w, h) = (img.width, img.height);
    let (pw, ph) = (w + 2, h + 2);
    let mut edges: Option<EdgeMap> = None;
    let mut ridge: Option<Vec<f32>> = None;

    let raws: Vec<Raw> = match params.mode {
        TraceMode::Outline => {
            let mask = threshold_mask(img, params.threshold);
            let labels = label(&mask, pw, ph);
            trace_outlines(img, params.threshold)
                .into_iter()
                .map(|(pts, closed)| {
                    // Contour points sit between pixels: take the label of an adjacent lit pixel.
                    let group = pts
                        .first()
                        .and_then(|p| {
                            let (x, y) = (p.x.floor() as isize, p.y.floor() as isize);
                            [(0, 0), (1, 0), (0, 1), (1, 1)].iter().find_map(|(dx, dy)| {
                                let (px, py) = (x + dx + 1, y + dy + 1);
                                if px < 0 || py < 0 || px as usize >= pw || py as usize >= ph {
                                    return None;
                                }
                                let l = labels[py as usize * pw + px as usize];
                                (l != 0).then_some(l)
                            })
                        })
                        .unwrap_or(0);
                    Raw { pts, closed, kind: Kind::Region, group }
                })
                .collect()
        }
        TraceMode::Centreline => {
            let mask = threshold_mask(img, params.threshold);
            let labels = label(&mask, pw, ph);
            with_groups(trace_skeleton(mask, pw, ph), &labels, pw, Kind::Region)
        }
        TraceMode::Edges => {
            let e = detect_edges(img, params.blur_px, params.edge_threshold);
            let labels = label(&e.mask, pw, ph);
            let out = with_groups(trace_skeleton(e.mask.clone(), pw, ph), &labels, pw, Kind::Edge);
            edges = Some(e);
            out
        }
        TraceMode::Auto => {
            let (stroke_mask, strength) =
                detect_strokes(img, params.stroke_width_px, params.stroke_threshold, params.stroke_blur_px);
            // Keep only stroke components that look like lines; branchy texture is left to edges.
            let stroke_mask = line_like(stroke_mask, pw, ph, params.min_length_px);
            let mut e = detect_edges(img, params.blur_px, params.edge_threshold);
            // Drop edges that merely border a stroke (they would draw the line twice).
            let reach = (params.stroke_width_px / 2.0).ceil() as isize + 1;
            let near_stroke = dilate(&stroke_mask, pw, ph, reach);
            for (m, n) in e.mask.iter_mut().zip(&near_stroke) {
                *m &= !n;
            }
            let mut union = stroke_mask.clone();
            for (u, m) in union.iter_mut().zip(&e.mask) {
                *u |= m;
            }
            let labels = label(&union, pw, ph);
            let mut out = with_groups(trace_skeleton(stroke_mask, pw, ph), &labels, pw, Kind::Stroke);
            out.extend(with_groups(trace_skeleton(e.mask.clone(), pw, ph), &labels, pw, Kind::Edge));
            edges = Some(e);
            ridge = Some(strength);
            out
        }
    };

    let raws = stitch(raws, img);

    // Minimum length applies per shape (group), so a shape never loses short pieces between
    // its junctions; only shapes that are small overall are dropped as noise.
    let raw_len = |r: &Raw| Path::new(r.pts.clone(), r.closed, Rgb::BLACK).length();
    let mut group_len: HashMap<u32, f32> = HashMap::new();
    for r in &raws {
        *group_len.entry(r.group).or_default() += raw_len(r);
    }
    let mapper = PixelMapper::new(w, h, params.fit);
    raws.into_iter()
        .filter(|r| {
            let len = if r.group == 0 { raw_len(r) } else { group_len[&r.group] };
            let scale = if r.kind == Kind::Edge { params.edge_min_length_scale.max(1.0) } else { 1.0 };
            len >= params.min_length_px * scale
        })
        .filter_map(|raw| {
            let closed = raw.closed;
            let pts = smooth(&raw.pts, closed, params.smoothing);
            let pts = simplify(&pts, closed, params.simplify_px);
            if pts.len() < if closed { 3 } else { 2 } {
                return None;
            }
            let pixel_path = Path::new(pts, closed, Rgb::BLACK);
            let (mut color, weight) = match raw.kind {
                Kind::Region => {
                    let c = sample_colour(img, &pixel_path.points, params.threshold);
                    (c, c.max_channel())
                }
                Kind::Stroke => {
                    let r = ridge.as_ref().expect("ridge map");
                    (sample_edge_colour(img, &pixel_path.points), mean_at(r, w, h, &pixel_path.points))
                }
                Kind::Edge => {
                    let e = edges.as_ref().expect("edge map");
                    (sample_edge_colour(img, &pixel_path.points), e.mean_strength(&pixel_path.points))
                }
            };
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
            let mut path = Path::new(points, closed, color);
            path.weight = weight;
            path.group = raw.group;
            // Auto detail steers edge / threshold content. Strokes are left to the planner,
            // whose selection has hysteresis (a tracing cutoff would make them flicker).
            path.detail_controlled = raw.kind != Kind::Stroke;
            Some(path)
        })
        .collect()
}

/// Join open paths of the same shape and kind that meet end to end (the skeleton tracer splits
/// at every junction) into longer strokes, continuing as straight as possible through each
/// junction. Fewer pieces = fewer dwell points, so more content fits and a shape's cost stays
/// stable from frame to frame.
/// Pieces of different colour (e.g. a white line touching a pink ring) are never joined.
fn stitch(raws: Vec<Raw>, img: &WorkImage) -> Vec<Raw> {
    const JOIN: f32 = 1.5; // pixels
    let hue = |pts: &[Vec2]| {
        let c = sample_edge_colour(img, pts);
        let m = c.max_channel().max(1e-6);
        Rgb::new(c.r / m, c.g / m, c.b / m)
    };
    let (closed, open): (Vec<Raw>, Vec<Raw>) = raws.into_iter().partition(|r| r.closed || r.pts.len() < 2);
    let mut out = closed;
    // Bucket by (group, kind) so only pieces of the same shape are joined.
    let mut buckets: HashMap<(u32, u8), Vec<Raw>> = HashMap::new();
    for r in open {
        let key = (r.group, r.kind as u8);
        buckets.entry(key).or_default().push(r);
    }
    let mut keys: Vec<(u32, u8)> = buckets.keys().copied().collect();
    keys.sort_unstable();
    for key in keys {
        let mut pieces = buckets.remove(&key).unwrap();
        if key.0 == 0 {
            out.extend(pieces);
            continue;
        }
        // Longest first, so the main strokes grow and short spurs attach to them.
        pieces.sort_by(|a, b| b.pts.len().cmp(&a.pts.len()));
        let colours: Vec<Rgb> = pieces.iter().map(|p| hue(&p.pts)).collect();
        let close = |a: Rgb, b: Rgb| (a.r - b.r).abs() + (a.g - b.g).abs() + (a.b - b.b).abs() < 0.45;
        let mut used = vec![false; pieces.len()];
        for i in 0..pieces.len() {
            if used[i] {
                continue;
            }
            used[i] = true;
            let mut chain = pieces[i].pts.clone();
            // Extend at the end, then (reversed) at the start.
            for _ in 0..2 {
                loop {
                    let n = chain.len();
                    let end = chain[n - 1];
                    let dir = direction(&chain, true);
                    let mut best: Option<(usize, bool, f32)> = None;
                    for (j, pc) in pieces.iter().enumerate() {
                        if used[j] || !close(colours[i], colours[j]) {
                            continue;
                        }
                        for (rev, at) in [(false, pc.pts[0]), (true, pc.pts[pc.pts.len() - 1])] {
                            if at.distance(end) > JOIN {
                                continue;
                            }
                            let mut cand = pc.pts.clone();
                            if rev {
                                cand.reverse();
                            }
                            let cdir = direction(&cand, false);
                            let straight = dir.dot(cdir); // 1 = straight on
                            if best.is_none_or(|b| straight > b.2) {
                                best = Some((j, rev, straight));
                            }
                        }
                    }
                    let Some((j, rev, _)) = best else { break };
                    used[j] = true;
                    let mut next = pieces[j].pts.clone();
                    if rev {
                        next.reverse();
                    }
                    let skip = if next[0].distance(end) < 1e-3 { 1 } else { 0 };
                    chain.extend_from_slice(&next[skip..]);
                }
                chain.reverse();
            }
            let closed = chain.len() > 3 && chain[0].distance(chain[chain.len() - 1]) <= JOIN;
            if closed {
                chain.pop();
            }
            out.push(Raw { pts: chain, closed, kind: pieces[i].kind, group: key.0 });
        }
    }
    out
}

/// Unit direction at one end of a polyline, looking a few points in; pointing outwards at the
/// end (`at_end`), or inwards from the start.
fn direction(pts: &[Vec2], at_end: bool) -> Vec2 {
    let n = pts.len();
    let k = 4.min(n - 1);
    let d = if at_end { pts[n - 1] - pts[n - 1 - k] } else { pts[k] - pts[0] };
    let l = d.length();
    if l < 1e-6 { Vec2::ZERO } else { d * (1.0 / l) }
}

fn with_groups(paths: Vec<(Vec<Vec2>, bool)>, labels: &[u32], pw: usize, kind: Kind) -> Vec<Raw> {
    paths
        .into_iter()
        .map(|(pts, closed)| {
            // Skeleton points are pixel centres (x + 0.5) in unpadded coordinates.
            let group = pts
                .first()
                .map(|p| labels[(p.y.floor() as usize + 1) * pw + p.x.floor() as usize + 1])
                .unwrap_or(0);
            Raw { pts, closed, kind, group }
        })
        .collect()
}

/// Padded mask of pixels at or above `threshold`.
fn threshold_mask(img: &WorkImage, threshold: f32) -> Vec<bool> {
    let pw = img.width + 2;
    let mut m = vec![false; pw * (img.height + 2)];
    for y in 0..img.height {
        for x in 0..img.width {
            m[(y + 1) * pw + x + 1] = img.value(x, y) >= threshold;
        }
    }
    m
}

/// 8-connected component labels (0 = background, 1.. = component) of a padded mask.
fn label(mask: &[bool], w: usize, h: usize) -> Vec<u32> {
    let mut labels = vec![0u32; w * h];
    let mut next = 1u32;
    let mut stack = Vec::new();
    for start in 0..w * h {
        if !mask[start] || labels[start] != 0 {
            continue;
        }
        labels[start] = next;
        stack.push(start);
        while let Some(i) = stack.pop() {
            let (x, y) = (i % w, i / w);
            for ny in y.saturating_sub(1)..=(y + 1).min(h - 1) {
                for nx in x.saturating_sub(1)..=(x + 1).min(w - 1) {
                    let j = ny * w + nx;
                    if mask[j] && labels[j] == 0 {
                        labels[j] = next;
                        stack.push(j);
                    }
                }
            }
        }
        next += 1;
    }
    labels
}

/// Filter a padded stroke mask to components whose skeleton is line-like: at least `min_len`
/// pixels long, with few loose ends. Line art crosses itself (dense ring clusters have a
/// junction every ~7 px) but has few loose ends; texture / filigree skeletons are full of spurs.
fn line_like(mask: Vec<bool>, w: usize, h: usize, min_len: f32) -> Vec<bool> {
    let labels = label(&mask, w, h);
    let mut skel = mask.clone();
    thin(&mut skel, w, h);
    let n = labels.iter().copied().max().unwrap_or(0) as usize;
    let mut len = vec![0usize; n + 1];
    let mut nodes = vec![0usize; n + 1];
    let mut junctions = vec![0usize; n + 1];
    for y in 1..h - 1 {
        for x in 1..w - 1 {
            let i = y * w + x;
            if !skel[i] {
                continue;
            }
            let l = labels[i] as usize;
            len[l] += 1;
            // m-adjacency (as in trace_skeleton): diagonals only count without a shared
            // 4-neighbour, so staircase corners aren't mistaken for junctions.
            let at = |dx: isize, dy: isize| skel[(y as isize + dy) as usize * w + (x as isize + dx) as usize];
            let mut k = [(1, 0), (-1, 0), (0, 1), (0, -1)].iter().filter(|&&(dx, dy)| at(dx, dy)).count();
            k += [(1, 1), (1, -1), (-1, 1), (-1, -1)]
                .iter()
                .filter(|&&(dx, dy)| at(dx, dy) && !at(dx, 0) && !at(0, dy))
                .count();
            if k <= 1 {
                nodes[l] += 1;
            }
            if k >= 3 {
                junctions[l] += 1;
            }
        }
    }
    if std::env::var("PE_DEBUG_STROKES").is_ok() {
        for l in 1..=n {
            if len[l] >= 20 {
                eprintln!("stroke comp len {} ends {} junctions {}", len[l], nodes[l], junctions[l]);
            }
        }
    }
    let keep: Vec<bool> = (0..=n)
        .map(|l| {
            // Measured loose ends per skeleton pixel: overlapping ring clusters <= 0.018,
            // fractal filigree >= 0.057. Junction density does NOT separate them (dense ring
            // clusters reach 0.15, like filigree). A plain line has 2 ends at any length.
            l > 0 && len[l] as f32 >= min_len.max(4.0) && nodes[l] * 40 <= len[l] + 80
        })
        .collect();
    mask.iter().zip(&labels).map(|(&m, &l)| m && keep[l as usize]).collect()
}

/// Square dilation of a padded mask by `r` pixels.
fn dilate(mask: &[bool], w: usize, h: usize, r: isize) -> Vec<bool> {
    // Separable: rows then columns.
    let mut tmp = vec![false; w * h];
    for y in 0..h {
        let mut last: isize = -(r + 1) - 1;
        let mut row = vec![false; w];
        for x in 0..w {
            if mask[y * w + x] {
                last = x as isize;
            }
            row[x] = x as isize - last <= r;
        }
        last = isize::MAX / 2;
        for x in (0..w).rev() {
            if mask[y * w + x] {
                last = x as isize;
            }
            tmp[y * w + x] = row[x] || last - x as isize <= r;
        }
    }
    let mut out = vec![false; w * h];
    for x in 0..w {
        let mut last: isize = -(r + 1) - 1;
        for y in 0..h {
            if tmp[y * w + x] {
                last = y as isize;
            }
            out[y * w + x] = y as isize - last <= r;
        }
        last = isize::MAX / 2;
        for y in (0..h).rev() {
            if tmp[y * w + x] {
                last = y as isize;
            }
            out[y * w + x] |= last - y as isize <= r;
        }
    }
    out
}

/// Ridge detector for thin bright strokes. A pixel is on a stroke if, along some direction, it
/// is brighter than the pixels `r` away on *both* sides by at least `threshold`.
/// Returns the padded stroke mask and the (unpadded) ridge strength.
fn detect_strokes(img: &WorkImage, max_width: f32, threshold: f32, blur: f32) -> (Vec<bool>, Vec<f32>) {
    let (w, h) = (img.width, img.height);
    let r = ((max_width / 2.0 + blur).floor() as isize + 1).max(1);
    let mut v: Vec<f32> = img.pixels.iter().map(|c| c.max_channel()).collect();
    gaussian_blur(&mut v, w, h, blur);
    let at = |x: isize, y: isize| {
        if x < 0 || y < 0 || x >= w as isize || y >= h as isize {
            0.0
        } else {
            v[y as usize * w + x as usize]
        }
    };
    let pw = w + 2;
    let mut strength = vec![0.0f32; w * h];
    let low = threshold * 0.4;
    for y in 0..h as isize {
        for x in 0..w as isize {
            let c = at(x, y);
            if c < low {
                continue;
            }
            let mut best = 0.0f32;
            for (dx, dy) in [(1, 0), (0, 1), (1, 1), (1, -1)] {
                let a = c - at(x + dx * r, y + dy * r);
                let b = c - at(x - dx * r, y - dy * r);
                best = best.max(a.min(b));
            }
            strength[y as usize * w + x as usize] = best;
        }
    }
    // Hysteresis: weak ridge pixels count only when connected to a strong one, so a line whose
    // contrast dips along its length (anti-aliasing, blur, crossings) stays in one piece.
    let mut mask = vec![false; pw * (h + 2)];
    let mut stack: Vec<usize> = (0..w * h).filter(|&i| strength[i] >= threshold).collect();
    for &i in &stack {
        mask[(i / w + 1) * pw + i % w + 1] = true;
    }
    while let Some(i) = stack.pop() {
        let (x, y) = (i % w, i / w);
        for ny in y.saturating_sub(1)..=(y + 1).min(h - 1) {
            for nx in x.saturating_sub(1)..=(x + 1).min(w - 1) {
                let j = ny * w + nx;
                let pj = (ny + 1) * pw + nx + 1;
                if strength[j] >= low && !mask[pj] {
                    mask[pj] = true;
                    stack.push(j);
                }
            }
        }
    }
    (mask, strength)
}

fn mean_at(map: &[f32], w: usize, h: usize, pts: &[Vec2]) -> f32 {
    if pts.is_empty() {
        return 0.0;
    }
    let sum: f32 = pts
        .iter()
        .map(|p| {
            let x = (p.x.floor().max(0.0) as usize).min(w - 1);
            let y = (p.y.floor().max(0.0) as usize).min(h - 1);
            map[y * w + x]
        })
        .sum();
    sum / pts.len() as f32
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

/// Colour of a stroke / the brighter side of an edge, sampled every pixel along the path.
/// At each sample the pixel under the path is used if it is lit, else the brightest neighbour
/// (for edges, which sit between pixels). The per-channel **median** is taken, and the ends are
/// skipped when possible, so pixels of another shape at a junction don't tint the result.
fn sample_edge_colour(img: &WorkImage, pts: &[Vec2]) -> Rgb {
    let mut samples: Vec<Vec2> = Vec::new();
    for w in pts.windows(2) {
        let n = (w[0].distance(w[1]).ceil() as usize).max(1);
        for k in 0..n {
            samples.push(w[0].lerp(w[1], k as f32 / n as f32));
        }
    }
    if let Some(&last) = pts.last() {
        samples.push(last);
    }
    if samples.len() > 8 {
        let trim = 2;
        samples = samples[trim..samples.len() - trim].to_vec();
    }
    let mut rs = Vec::with_capacity(samples.len());
    let mut gs = Vec::with_capacity(samples.len());
    let mut bs = Vec::with_capacity(samples.len());
    for p in &samples {
        let cx = (p.x.floor() as isize).clamp(0, img.width as isize - 1);
        let cy = (p.y.floor() as isize).clamp(0, img.height as isize - 1);
        let centre = img.get(cx as usize, cy as usize);
        let mut best = Rgb::BLACK;
        for y in (cy - 1).max(0)..=(cy + 1).min(img.height as isize - 1) {
            for x in (cx - 1).max(0)..=(cx + 1).min(img.width as isize - 1) {
                let c = img.get(x as usize, y as usize);
                if c.max_channel() > best.max_channel() {
                    best = c;
                }
            }
        }
        let c = if centre.max_channel() >= 0.5 * best.max_channel() { centre } else { best };
        rs.push(c.r);
        gs.push(c.g);
        bs.push(c.b);
    }
    if rs.is_empty() {
        return Rgb::BLACK;
    }
    let median = |v: &mut Vec<f32>| {
        v.sort_by(|a, b| a.total_cmp(b));
        v[v.len() / 2]
    };
    Rgb::new(median(&mut rs), median(&mut gs), median(&mut bs))
}

// ---------------------------------------------------------------------------------------------
// Edge detection (Canny-style).

pub struct EdgeMap {
    /// Padded (width + 2) x (height + 2) mask of edge pixels.
    pub mask: Vec<bool>,
    /// Unpadded gradient magnitude (0..~1.4, 1.0 = a full black-to-white step).
    pub strength: Vec<f32>,
    pub width: usize,
    pub height: usize,
}

impl EdgeMap {
    /// Mean edge strength under a pixel-space polyline.
    pub fn mean_strength(&self, pts: &[Vec2]) -> f32 {
        let mut sum = 0.0;
        for p in pts {
            let x = (p.x.floor().max(0.0) as usize).min(self.width - 1);
            let y = (p.y.floor().max(0.0) as usize).min(self.height - 1);
            sum += self.strength[y * self.width + x];
        }
        if pts.is_empty() { 0.0 } else { sum / pts.len() as f32 }
    }
}

pub fn detect_edges(img: &WorkImage, blur_px: f32, high: f32) -> EdgeMap {
    let (w, h) = (img.width, img.height);
    let mut v: Vec<f32> = (0..w * h).map(|i| img.pixels[i].max_channel()).collect();
    gaussian_blur(&mut v, w, h, blur_px);

    let at = |x: isize, y: isize| {
        let x = x.clamp(0, w as isize - 1) as usize;
        let y = y.clamp(0, h as isize - 1) as usize;
        v[y * w + x]
    };
    let mut mag = vec![0.0f32; w * h];
    let mut dir = vec![(0i8, 0i8); w * h];
    for y in 0..h as isize {
        for x in 0..w as isize {
            let gx = (at(x + 1, y - 1) + 2.0 * at(x + 1, y) + at(x + 1, y + 1))
                - (at(x - 1, y - 1) + 2.0 * at(x - 1, y) + at(x - 1, y + 1));
            let gy = (at(x - 1, y + 1) + 2.0 * at(x, y + 1) + at(x + 1, y + 1))
                - (at(x - 1, y - 1) + 2.0 * at(x, y - 1) + at(x + 1, y - 1));
            let m = (gx * gx + gy * gy).sqrt() / 4.0;
            let i = y as usize * w + x as usize;
            mag[i] = m;
            if m > 1e-6 {
                // Quantise the gradient direction to one of 8 neighbours.
                let (ux, uy) = (gx / (4.0 * m), gy / (4.0 * m));
                let q = |u: f32| if u > 0.3827 { 1 } else if u < -0.3827 { -1 } else { 0 };
                dir[i] = (q(ux), q(uy));
            }
        }
    }

    // Non-maximum suppression: keep only ridge pixels across the edge.
    let low = high * 0.5;
    let mg = |x: isize, y: isize| {
        if x < 0 || y < 0 || x >= w as isize || y >= h as isize { 0.0 } else { mag[y as usize * w + x as usize] }
    };
    let mut ridge = vec![0u8; w * h]; // 0 none, 1 weak, 2 strong
    for y in 0..h as isize {
        for x in 0..w as isize {
            let i = y as usize * w + x as usize;
            let m = mag[i];
            if m < low {
                continue;
            }
            let (dx, dy) = (dir[i].0 as isize, dir[i].1 as isize);
            if m >= mg(x + dx, y + dy) && m > mg(x - dx, y - dy) {
                ridge[i] = if m >= high { 2 } else { 1 };
            }
        }
    }

    // Hysteresis: weak pixels survive only if connected to a strong one.
    let pw = w + 2;
    let mut mask = vec![false; pw * (h + 2)];
    let mut stack: Vec<usize> = (0..w * h).filter(|&i| ridge[i] == 2).collect();
    for &i in &stack {
        mask[(i / w + 1) * pw + i % w + 1] = true;
    }
    while let Some(i) = stack.pop() {
        let (x, y) = ((i % w) as isize, (i / w) as isize);
        for dy in -1..=1 {
            for dx in -1..=1 {
                let (nx, ny) = (x + dx, y + dy);
                if nx < 0 || ny < 0 || nx >= w as isize || ny >= h as isize {
                    continue;
                }
                let j = ny as usize * w + nx as usize;
                let pj = (ny as usize + 1) * pw + nx as usize + 1;
                if ridge[j] == 1 && !mask[pj] {
                    mask[pj] = true;
                    stack.push(j);
                }
            }
        }
    }
    EdgeMap { mask, strength: mag, width: w, height: h }
}

fn gaussian_blur(v: &mut [f32], w: usize, h: usize, sigma: f32) {
    if sigma < 0.3 {
        return;
    }
    let r = (sigma * 3.0).ceil() as isize;
    let kernel: Vec<f32> = (-r..=r).map(|i| (-(i * i) as f32 / (2.0 * sigma * sigma)).exp()).collect();
    let norm: f32 = kernel.iter().sum();
    let kernel: Vec<f32> = kernel.iter().map(|k| k / norm).collect();
    let mut tmp = vec![0.0f32; w * h];
    for y in 0..h {
        for x in 0..w {
            let mut acc = 0.0;
            for (k, kv) in kernel.iter().enumerate() {
                let sx = (x as isize + k as isize - r).clamp(0, w as isize - 1) as usize;
                acc += v[y * w + sx] * kv;
            }
            tmp[y * w + x] = acc;
        }
    }
    for y in 0..h {
        for x in 0..w {
            let mut acc = 0.0;
            for (k, kv) in kernel.iter().enumerate() {
                let sy = (y as isize + k as isize - r).clamp(0, h as isize - 1) as usize;
                acc += tmp[sy * w + x] * kv;
            }
            v[y * w + x] = acc;
        }
    }
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

/// Thin a padded mask (false border) to a 1px skeleton and follow it into polylines.
fn trace_skeleton(mut m: Vec<bool>, w: usize, h: usize) -> Vec<(Vec<Vec2>, bool)> {
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
// Smoothing and simplification.

/// Repeated 1-2-1 averaging of vertices. Open paths keep their end points.
pub fn smooth(pts: &[Vec2], closed: bool, passes: u32) -> Vec<Vec2> {
    let n = pts.len();
    if n < 3 {
        return pts.to_vec();
    }
    let mut cur = pts.to_vec();
    let mut next = cur.clone();
    for _ in 0..passes {
        for i in 0..n {
            let (prev, nxt) = if closed {
                ((i + n - 1) % n, (i + 1) % n)
            } else if i == 0 || i == n - 1 {
                (i, i)
            } else {
                (i - 1, i + 1)
            };
            next[i] = (cur[prev] + cur[i] * 2.0 + cur[nxt]) * 0.25;
        }
        std::mem::swap(&mut cur, &mut next);
    }
    cur
}

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
    fn centreline_cross_becomes_two_straight_strokes() {
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
        assert_eq!(paths.len(), 2, "{paths:?}");
        // Stitched straight through the junction: one vertical, one horizontal stroke.
        let vertical = paths.iter().any(|p| p.points.iter().all(|v| v.x.abs() < 1e-4));
        let horizontal = paths.iter().any(|p| p.points.iter().all(|v| v.y.abs() < 1e-4));
        assert!(vertical && horizontal, "{paths:?}");
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
