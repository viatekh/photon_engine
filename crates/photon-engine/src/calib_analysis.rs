//! Offline analysis of a calibration session (recordings/calib-*.jsonl.gz):
//!
//!     photon-engine --analyse-calibration recordings/calib-123.jsonl.gz out_dir
//!
//! 1. Registration: finds the drawn border in the camera image (background subtracted) and the
//!    red corner mark, giving a homography from laser coordinates to camera pixels.
//! 2. Per step: overlays the commanded path on the camera image (PPM files in out_dir) and
//!    measures how well the light matches the command:
//!    * coverage - fraction of the commanded lit path with light near it (gaps, missing ends);
//!    * stray    - light further than a few pixels from the commanded path (overshoot,
//!                 rounding, tails), as a fraction of all light, and its 95th percentile distance.
//! 3. Colour-delay comb: where the dashes actually light up versus where they were commanded,
//!    separately for left-to-right and right-to-left rows. The delay at which both directions
//!    agree is the right colour delay.

use anyhow::{bail, Context};
use base64::Engine as _;
use flate2::read::GzDecoder;
use photon_core::keystone::Homography;
use photon_core::Vec2;
use serde_json::Value;
use std::io::{BufRead, BufReader, Write};

struct Img {
    w: usize,
    h: usize,
    /// Brightness 0..1 (max channel), background subtracted.
    v: Vec<f32>,
}

fn decode(v: &Value, key: &str, w: usize, h: usize) -> anyhow::Result<Vec<u8>> {
    let raw = base64::engine::general_purpose::STANDARD.decode(v[key].as_str().context("image")?)?;
    if raw.len() != w * h * 3 {
        bail!("image size mismatch");
    }
    Ok(raw)
}

/// Laser light only: each channel minus its local background (a morphological opening with a
/// radius wider than a laser line, which removes the lines and keeps the lit wall). This copes
/// with ambient light and with the webcam changing exposure / white balance between steps,
/// which a plain "minus the dark frame" does not.
fn to_img(rgb: &[u8], dark: Option<&[u8]>, w: usize, h: usize) -> Img {
    const R: usize = 12;
    let tophat = |img: &[u8], k: usize| -> Vec<f32> {
        let c: Vec<f32> = (0..w * h).map(|i| img[i * 3 + k] as f32 / 255.0).collect();
        let bg = opening(&c, w, h, R);
        c.iter().zip(&bg).map(|(v, b)| (v - b).max(0.0)).collect()
    };
    let median = |img: &[u8], k: usize| -> f32 {
        let mut v: Vec<u8> = (0..w * h).step_by(7).map(|i| img[i * 3 + k]).collect();
        v.sort_unstable();
        v[v.len() / 2] as f32
    };
    let mut chans = [vec![0.0f32; w * h], vec![0.0f32; w * h], vec![0.0f32; w * h]];
    for k in 0..3 {
        let mut t = tophat(rgb, k);
        // Fine static detail (railings, edges) is in the laser-off frame too: remove it,
        // scaled for the camera's exposure change and dilated a little for camera shake.
        if let Some(d) = dark {
            let gain = (median(rgb, k) + 1.0) / (median(d, k) + 1.0);
            let td = dilate(&tophat(d, k), w, h, 2);
            for (v, s) in t.iter_mut().zip(&td) {
                *v = (*v - s * gain * 1.2).max(0.0);
            }
        }
        chans[k] = t;
    }
    let v = (0..w * h).map(|i| chans[0][i].max(chans[1][i]).max(chans[2][i])).collect();
    Img { w, h, v }
}

fn dilate(c: &[f32], w: usize, h: usize, r: usize) -> Vec<f32> {
    morph(&morph(c, w, h, r, true, false), w, h, r, false, false)
}

fn morph(src: &[f32], w: usize, h: usize, r: usize, horizontal: bool, take_min: bool) -> Vec<f32> {
    let mut out = vec![0.0f32; w * h];
    let (n, m) = if horizontal { (h, w) } else { (w, h) };
    let idx = |line: usize, j: usize| if horizontal { line * w + j } else { j * w + line };
    let mut buf = vec![0.0f32; m];
    for line in 0..n {
        for j in 0..m {
            buf[j] = src[idx(line, j)];
        }
        for j in 0..m {
            let (lo, hi) = (j.saturating_sub(r), (j + r).min(m - 1));
            let mut v = buf[lo];
            for &x in &buf[lo + 1..=hi] {
                v = if take_min { v.min(x) } else { v.max(x) };
            }
            out[idx(line, j)] = v;
        }
    }
    out
}

/// Grey-scale opening (erode then dilate) with a square of radius r.
fn opening(c: &[f32], w: usize, h: usize, r: usize) -> Vec<f32> {
    let e = morph(&morph(c, w, h, r, true, true), w, h, r, false, true);
    morph(&morph(&e, w, h, r, true, false), w, h, r, false, false)
}

/// Threshold for "lit": a fraction of the bright end of the image.
fn lit_threshold(img: &Img) -> f32 {
    let mut vals: Vec<f32> = img.v.iter().copied().filter(|&x| x > 0.02).collect();
    if vals.is_empty() {
        return 1.0;
    }
    vals.sort_by(|a, b| a.total_cmp(b));
    (vals[(vals.len() * 99 / 100).min(vals.len() - 1)] * 0.25).max(0.04)
}

struct Step {
    name: String,
    mean: Vec<u8>,
    sent: Vec<[f32; 5]>,
}

pub fn run(input: &str, out_dir: &str) -> anyhow::Result<()> {
    std::fs::create_dir_all(out_dir)?;
    let reader = BufReader::new(GzDecoder::new(std::fs::File::open(input)?));
    let (mut w, mut h) = (0usize, 0usize);
    let mut steps: Vec<Step> = Vec::new();
    for line in reader.lines() {
        let line = line?;
        if line.trim().is_empty() {
            continue;
        }
        let v: Value = serde_json::from_str(&line)?;
        match v["type"].as_str() {
            Some("header") => {
                w = v["cam_w"].as_u64().unwrap_or(0) as usize;
                h = v["cam_h"].as_u64().unwrap_or(0) as usize;
            }
            Some("step") => {
                let sent = v["sent_xyrgb"]
                    .as_array()
                    .map(|a| {
                        a.chunks(5)
                            .map(|c| {
                                let f = |i: usize| c.get(i).and_then(|x| x.as_f64()).unwrap_or(0.0) as f32;
                                [f(0), f(1), f(2), f(3), f(4)]
                            })
                            .collect()
                    })
                    .unwrap_or_default();
                steps.push(Step { name: v["name"].as_str().unwrap_or("").to_string(), mean: decode(&v, "mean_rgb", w, h)?, sent });
            }
            _ => {}
        }
    }
    let reg_i = steps.iter().position(|s| s.name == "registration").context("no registration step")?;
    let dark = steps.iter().position(|s| s.name == "dark").map(|i| steps[i].mean.clone());
    let reg = to_img(&steps[reg_i].mean, dark.as_deref(), w, h);
    let hmg = register(&reg, &steps[reg_i].sent).context("could not find the registration border in the camera image")?;
    let corners = [Vec2::new(-1.0, 1.0), Vec2::new(1.0, 1.0), Vec2::new(1.0, -1.0), Vec2::new(-1.0, -1.0)];
    println!(
        "registration: laser corners (-0.9,0.9) (0.9,0.9) (0.9,-0.9) (-0.9,-0.9) -> camera {:?}",
        corners.map(|c| { let p = hmg.apply(c * 0.9); (p.x.round(), p.y.round()) })
    );

    println!("{:<34} {:>9} {:>8} {:>10}", "step", "coverage", "stray", "stray p95");
    let mut summary = String::new();
    let mut delay_shift: Vec<(f32, f32)> = Vec::new();
    for (i, st) in steps.iter().enumerate() {
        if st.name == "dark" {
            continue;
        }
        let mut img = to_img(&st.mean, dark.as_deref(), w, h);
        // Only look inside (slightly beyond) the projection area: reflections elsewhere
        // (windows, shiny surfaces) aren't the scanner's fault.
        if let Some(inv) = Homography::from_points(corners.map(|c| hmg.apply(c)), corners) {
            for y in 0..h {
                for x in 0..w {
                    let q = inv.apply(Vec2::new(x as f32, y as f32));
                    if q.x.abs() > 1.05 || q.y.abs() > 1.05 {
                        img.v[y * w + x] = 0.0;
                    }
                }
            }
        }
        // Commanded lit segments in camera pixels.
        let segs: Vec<(Vec2, Vec2)> = st
            .sent
            .windows(2)
            .filter(|p| p[0][2] + p[0][3] + p[0][4] > 0.0 && p[1][2] + p[1][3] + p[1][4] > 0.0)
            .map(|p| (hmg.apply(Vec2::new(p[0][0], p[0][1])), hmg.apply(Vec2::new(p[1][0], p[1][1]))))
            .collect();
        let dist = distance_to_segments(&segs, w, h);
        // Threshold relative to how bright this step's lines are where light is expected
        // (faster drawing = dimmer lines; a global threshold would read that as gaps).
        let thr = path_threshold(&img, &segs).max(lit_threshold(&img) * 0.2);
        // Coverage: commanded samples with light within 3 px.
        let (mut covered, mut samples) = (0usize, 0usize);
        for &(a, b) in &segs {
            let n = (a.distance(b).ceil() as usize).max(1);
            for k in 0..n {
                let q = a.lerp(b, k as f32 / n as f32);
                samples += 1;
                if lit_near(&img, q, 3, thr) {
                    covered += 1;
                }
            }
        }
        // Stray: lit pixels far from the command.
        let mut far = Vec::new();
        let mut lit = 0usize;
        for i in 0..w * h {
            if img.v[i] >= thr {
                lit += 1;
                far.push(dist[i]);
            }
        }
        far.sort_by(|a, b| a.total_cmp(b));
        let stray = far.iter().filter(|&&d| d > 4.0).count() as f32 / lit.max(1) as f32;
        let p95 = far.get(far.len() * 95 / 100).copied().unwrap_or(0.0);
        let coverage = covered as f32 / samples.max(1) as f32;
        let row = format!("{:<34} {:>8.0}% {:>7.1}% {:>8.1}px", st.name, coverage * 100.0, stray * 100.0, p95);
        println!("{row}");
        summary += &row;
        summary.push('\n');
        if st.name.starts_with("delay") {
            if let Some((lr, rl)) = comb_offsets(&img, &hmg, thr) {
                // Blur widens both ends equally; a timing error moves both ends the same way.
                // Averaging the two directions cancels any leftover registration offset.
                let shift = ((lr.0 + lr.1) / 2.0 + (rl.0 + rl.1) / 2.0) / 2.0;
                if let Some(d) = st.name.rsplit('=').next().and_then(|v| v.parse::<f32>().ok()) {
                    delay_shift.push((d, shift));
                }
                let line = format!("    comb: lit start/end offset L->R {:+.3}/{:+.3}  R->L {:+.3}/{:+.3}  shift {:+.4} (laser units, + = light late)", lr.0, lr.1, rl.0, rl.1, shift);
                println!("{line}");
                summary += &line;
                summary.push('\n');
            }
        }
        save_overlay(&format!("{out_dir}/step{i:02}.ppm"), &img, &segs, w, h)?;
    }
    // Colour delay estimate: where the light's shift along the stroke crosses zero.
    if let Some(w) = delay_shift.windows(2).find(|w| w[0].1 <= 0.0 && w[1].1 > 0.0) {
        let ((d0, s0), (d1, s1)) = (w[0], w[1]);
        let best = d0 + (d1 - d0) * (-s0) / (s1 - s0);
        let line = format!("colour delay estimate: {best:.0} us (light neither early nor late)");
        println!("{line}");
        summary += &line;
        summary.push('\n');
    }
    std::fs::write(format!("{out_dir}/summary.txt"), summary)?;
    Ok(())
}

/// 30% of the median brightness found along the commanded path.
fn path_threshold(img: &Img, segs: &[(Vec2, Vec2)]) -> f32 {
    let mut vals = Vec::new();
    for &(a, b) in segs {
        let n = (a.distance(b).ceil() as usize).max(1);
        for k in 0..n {
            let q = a.lerp(b, k as f32 / n as f32);
            let (x, y) = (q.x.round() as isize, q.y.round() as isize);
            let mut m = 0.0f32;
            for dy in -2..=2 {
                for dx in -2..=2 {
                    let (px, py) = (x + dx, y + dy);
                    if px >= 0 && py >= 0 && (px as usize) < img.w && (py as usize) < img.h {
                        m = m.max(img.v[py as usize * img.w + px as usize]);
                    }
                }
            }
            vals.push(m);
        }
    }
    if vals.is_empty() {
        return 1.0;
    }
    vals.sort_by(|a, b| a.total_cmp(b));
    (vals[vals.len() / 2] * 0.3).max(0.01)
}

fn lit_near(img: &Img, q: Vec2, r: isize, thr: f32) -> bool {
    let (x, y) = (q.x.round() as isize, q.y.round() as isize);
    for dy in -r..=r {
        for dx in -r..=r {
            let (px, py) = (x + dx, y + dy);
            if px >= 0 && py >= 0 && (px as usize) < img.w && (py as usize) < img.h && img.v[py as usize * img.w + px as usize] >= thr {
                return true;
            }
        }
    }
    false
}

/// Homography laser -> camera from the registration image: the border's four corners
/// (commanded at +-0.9) and the red mark at the top-left corner to fix orientation.
fn register(img: &Img, sent: &[[f32; 5]]) -> Option<Homography> {
    let thr = lit_threshold(img);
    // The border is one closed loop: take the largest connected lit region, so reflections
    // and stray light elsewhere in the picture don't pull the corners.
    let pts: Vec<Vec2> = largest_component(img, thr)
        .into_iter()
        .map(|i| Vec2::new((i % img.w) as f32, (i / img.w) as f32))
        .collect();
    if pts.len() < 50 {
        return None;
    }
    // Extreme points along the diagonals: corners of a convex quadrilateral.
    let ext = |f: &dyn Fn(Vec2) -> f32| pts.iter().copied().max_by(|a, b| f(*a).total_cmp(&f(*b))).unwrap();
    let mut quad = [ext(&|p| -p.x - p.y), ext(&|p| p.x - p.y), ext(&|p| p.x + p.y), ext(&|p| -p.x + p.y)];
    // Refine: fit a line to the middle part of each side and intersect neighbours, so a
    // corner hidden behind something (or a stray blob near one) doesn't skew the result.
    let dim: Vec<Vec2> = (0..img.w * img.h)
        .filter(|&i| img.v[i] >= thr * 0.4)
        .map(|i| Vec2::new((i % img.w) as f32, (i / img.w) as f32))
        .collect();
    for band in [40.0f32, 15.0, 8.0] {
        let mut lines = Vec::new();
        for k in 0..4 {
            let (a, b) = (quad[k], quad[(k + 1) % 4]);
            let d = b - a;
            let len = d.length().max(1.0);
            let dir = d * (1.0 / len);
            let normal = Vec2::new(-dir.y, dir.x);
            let near: Vec<Vec2> = dim
                .iter()
                .copied()
                .filter(|&p| {
                    let t = (p - a).dot(dir) / len;
                    (0.15..0.85).contains(&t) && (p - a).dot(normal).abs() < band
                })
                .collect();
            lines.push(fit_line(&near)?);
        }
        let mut next = quad;
        for k in 0..4 {
            next[(k + 1) % 4] = intersect(lines[k], lines[(k + 1) % 4])?;
        }
        quad = next;
    }
    // The corner mark (drawn inside the top-left corner) breaks the square's symmetry: try
    // every rotation and both windings, and keep the one under which the commanded lit points
    // away from the border land on the most light.
    let mark: Vec<Vec2> = sent
        .iter()
        .filter(|p| p[2] + p[3] + p[4] > 0.0 && p[0].abs() < 0.85 && p[1].abs() < 0.85)
        .map(|p| Vec2::new(p[0], p[1]))
        .collect();
    let laser = [Vec2::new(-0.9, 0.9), Vec2::new(0.9, 0.9), Vec2::new(0.9, -0.9), Vec2::new(-0.9, -0.9)];
    let mut best: Option<(f32, Homography)> = None;
    // A camera looking at the projection surface never sees it mirrored (and the mark can't
    // tell a mirror across its diagonal apart), so only rotations are tried.
    for mirror in [false] {
        for rot in 0..4 {
            let mut cam = [Vec2::ZERO; 4];
            for k in 0..4 {
                let j = if mirror { (4 - k + rot) % 4 } else { (k + rot) % 4 };
                cam[k] = quad[j];
            }
            if let Some(h) = Homography::from_points(laser, cam) {
                let score: f32 = mark.iter().map(|&p| if lit_near(img, h.apply(p), 2, thr) { 1.0 } else { 0.0 }).sum();
                if best.as_ref().is_none_or(|b| score > b.0) {
                    best = Some((score, h));
                }
            }
        }
    }
    best.map(|b| b.1)
}

/// Robust line through points (RANSAC, then least squares on the inliers): (a point on it,
/// unit direction). Robust because lit clutter (leaves, reflections) can sit next to a side.
fn fit_line(pts: &[Vec2]) -> Option<(Vec2, Vec2)> {
    if pts.len() < 10 {
        return None;
    }
    let mut seed = 0x2545_f491_4f6c_dd1du64;
    let mut rand = |n: usize| {
        seed ^= seed << 13;
        seed ^= seed >> 7;
        seed ^= seed << 17;
        (seed % n as u64) as usize
    };
    let inliers = |p0: Vec2, dir: Vec2| -> Vec<Vec2> {
        let normal = Vec2::new(-dir.y, dir.x);
        pts.iter().copied().filter(|&p| (p - p0).dot(normal).abs() < 3.0).collect()
    };
    let mut best: Vec<Vec2> = Vec::new();
    for _ in 0..300 {
        let (a, b) = (pts[rand(pts.len())], pts[rand(pts.len())]);
        let d = b - a;
        if d.length() < 20.0 {
            continue;
        }
        let inl = inliers(a, d * (1.0 / d.length()));
        if inl.len() > best.len() {
            best = inl;
        }
    }
    if best.len() < 10 {
        return None;
    }
    let (c, dir) = least_squares_line(&best);
    Some(least_squares_line(&inliers(c, dir)))
}

fn least_squares_line(pts: &[Vec2]) -> (Vec2, Vec2) {
    let n = pts.len().max(1) as f32;
    let c = pts.iter().fold(Vec2::ZERO, |s, &p| s + p) * (1.0 / n);
    let (mut sxx, mut sxy, mut syy) = (0.0f32, 0.0f32, 0.0f32);
    for &p in pts {
        let d = p - c;
        sxx += d.x * d.x;
        sxy += d.x * d.y;
        syy += d.y * d.y;
    }
    let angle = 0.5 * (2.0 * sxy).atan2(sxx - syy);
    (c, Vec2::new(angle.cos(), angle.sin()))
}

fn intersect(l1: (Vec2, Vec2), l2: (Vec2, Vec2)) -> Option<Vec2> {
    let (p, r) = l1;
    let (q, s) = l2;
    let den = r.x * s.y - r.y * s.x;
    if den.abs() < 1e-6 {
        return None;
    }
    let t = ((q.x - p.x) * s.y - (q.y - p.y) * s.x) / den;
    Some(p + r * t)
}

fn largest_component(img: &Img, thr: f32) -> Vec<usize> {
    let (w, h) = (img.w, img.h);
    let mut seen = vec![false; w * h];
    let mut best = Vec::new();
    for start in 0..w * h {
        if seen[start] || img.v[start] < thr {
            continue;
        }
        let mut comp = vec![start];
        seen[start] = true;
        let mut k = 0;
        while k < comp.len() {
            let i = comp[k];
            k += 1;
            let (x, y) = ((i % w) as isize, (i / w) as isize);
            for dy in -2..=2isize {
                for dx in -2..=2isize {
                    let (nx, ny) = (x + dx, y + dy);
                    if nx < 0 || ny < 0 || nx >= w as isize || ny >= h as isize {
                        continue;
                    }
                    let j = ny as usize * w + nx as usize;
                    if !seen[j] && img.v[j] >= thr {
                        seen[j] = true;
                        comp.push(j);
                    }
                }
            }
        }
        if comp.len() > best.len() {
            best = comp;
        }
    }
    best
}

/// Chamfer distance (pixels) from every pixel to the nearest commanded segment.
fn distance_to_segments(segs: &[(Vec2, Vec2)], w: usize, h: usize) -> Vec<f32> {
    let mut d = vec![f32::MAX; w * h];
    for &(a, b) in segs {
        let n = (a.distance(b).ceil() as usize).max(1);
        for k in 0..=n {
            let q = a.lerp(b, k as f32 / n as f32);
            let (x, y) = (q.x.round() as isize, q.y.round() as isize);
            if x >= 0 && y >= 0 && (x as usize) < w && (y as usize) < h {
                d[y as usize * w + x as usize] = 0.0;
            }
        }
    }
    let (a, b) = (1.0f32, std::f32::consts::SQRT_2);
    for y in 0..h {
        for x in 0..w {
            let i = y * w + x;
            let mut v = d[i];
            if x > 0 { v = v.min(d[i - 1] + a); }
            if y > 0 {
                v = v.min(d[i - w] + a);
                if x > 0 { v = v.min(d[i - w - 1] + b); }
                if x + 1 < w { v = v.min(d[i - w + 1] + b); }
            }
            d[i] = v;
        }
    }
    for y in (0..h).rev() {
        for x in (0..w).rev() {
            let i = y * w + x;
            let mut v = d[i];
            if x + 1 < w { v = v.min(d[i + 1] + a); }
            if y + 1 < h {
                v = v.min(d[i + w] + a);
                if x + 1 < w { v = v.min(d[i + w + 1] + b); }
                if x > 0 { v = v.min(d[i + w - 1] + b); }
            }
            d[i] = v;
        }
    }
    d
}

/// For the delay comb (rows y = 0.7 - 0.2 * r, dashes x0 = -0.8 + 0.28 k, length 0.16; even
/// rows left-to-right, odd rows right-to-left): average offset of the lit start and end from
/// the commanded start and end, along the drawing direction, per direction.
fn comb_offsets(img: &Img, h: &Homography, thr: f32) -> Option<((f32, f32), (f32, f32))> {
    let (mut lr, mut rl) = (Vec::new(), Vec::new());
    for row in 0..8 {
        let y = 0.7 - row as f32 * 0.2;
        for k in 0..6 {
            let x0 = -0.8 + k as f32 * 0.28;
            // Scan along the dash with margin, find lit extent.
            let (lo, hi) = (x0 - 0.06, x0 + 0.22);
            let n = 200;
            let lit: Vec<bool> = (0..=n)
                .map(|i| {
                    let x = lo + (hi - lo) * i as f32 / n as f32;
                    lit_near(img, h.apply(Vec2::new(x, y)), 1, thr)
                })
                .collect();
            let first = lit.iter().position(|&b| b)?;
            let last = lit.iter().rposition(|&b| b)?;
            let x_first = lo + (hi - lo) * first as f32 / n as f32;
            let x_last = lo + (hi - lo) * last as f32 / n as f32;
            if row % 2 == 0 {
                // Drawn from x0 to x0+0.16: start offset = x_first - x0 (later = +).
                lr.push((x_first - x0, x_last - (x0 + 0.16)));
            } else {
                // Drawn from x0+0.16 to x0: start is the right end; "later" = further left.
                rl.push(((x0 + 0.16) - x_last, x0 - x_first));
            }
        }
    }
    let avg = |v: &[(f32, f32)]| {
        let n = v.len().max(1) as f32;
        (v.iter().map(|p| p.0).sum::<f32>() / n, v.iter().map(|p| p.1).sum::<f32>() / n)
    };
    Some((avg(&lr), avg(&rl)))
}

fn save_overlay(path: &str, img: &Img, segs: &[(Vec2, Vec2)], w: usize, h: usize) -> anyhow::Result<()> {
    let mut rgb = vec![0u8; w * h * 3];
    let peak = img.v.iter().copied().fold(0.0f32, f32::max).max(1e-3);
    for i in 0..w * h {
        let g = ((img.v[i] / peak).powf(0.6) * 255.0) as u8;
        rgb[i * 3..i * 3 + 3].copy_from_slice(&[g, g, g]);
    }
    for &(a, b) in segs {
        let n = (a.distance(b).ceil() as usize).max(1);
        for k in 0..=n {
            let q = a.lerp(b, k as f32 / n as f32);
            let (x, y) = (q.x.round() as isize, q.y.round() as isize);
            if x >= 0 && y >= 0 && (x as usize) < w && (y as usize) < h {
                let i = (y as usize * w + x as usize) * 3;
                rgb[i] = 255;
                rgb[i + 1] = rgb[i + 1] / 3;
                rgb[i + 2] = rgb[i + 2] / 3;
            }
        }
    }
    let mut f = std::fs::File::create(path)?;
    write!(f, "P6 {w} {h} 255\n")?;
    f.write_all(&rgb)?;
    Ok(())
}
