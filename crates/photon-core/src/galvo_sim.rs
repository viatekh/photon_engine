//! Physical preview: simulates galvo mirrors following a point stream, and renders what the
//! beam would look like as a long exposure (what a camera or the eye sees).
//!
//! Each mirror axis is modelled as a damped second-order system (inertia + spring + damping),
//! the standard small-signal model of a closed-loop galvo, with a slew-rate limit. The
//! natural frequency scales with the scanner's kpps rating. The numbers are approximate - the
//! point is to make overshoot, ringing, lag, bright dwell spots and seam gaps visible when
//! comparing settings, not to predict a specific scanner exactly.

use crate::geom::{LaserPoint, Rgb, Vec2};

#[derive(Clone, Copy, Debug)]
pub struct GalvoModel {
    /// Undamped natural frequency (Hz). ~3 kHz is typical for a "30K" scanner.
    pub natural_hz: f32,
    /// Damping ratio (<1 rings, 1 = critically damped).
    pub damping: f32,
    /// Max mirror speed, field units per second.
    pub max_speed: f32,
}

impl GalvoModel {
    pub fn for_rating(kpps: f32) -> Self {
        let f = (kpps / 30.0).max(0.1);
        Self { natural_hz: 3000.0 * f, damping: 0.55, max_speed: 4000.0 * f }
    }
}

/// Beam position and colour at `substeps` samples per point, looping the frame `loops` times
/// (the first loop is discarded so the mirrors start in steady state).
pub fn simulate(frame: &[LaserPoint], pps: u32, model: GalvoModel, substeps: usize, loops: usize) -> Vec<(Vec2, Rgb)> {
    if frame.is_empty() {
        return Vec::new();
    }
    let dt = 1.0 / (pps as f32 * substeps as f32);
    let w = 2.0 * std::f32::consts::PI * model.natural_hz;
    let (mut p, mut v) = (frame[0].pos(), Vec2::ZERO);
    let mut out = Vec::with_capacity(frame.len() * substeps * loops.saturating_sub(1));
    for lap in 0..loops {
        for pt in frame {
            let target = pt.pos();
            let c = Rgb::new(pt.r, pt.g, pt.b);
            for _ in 0..substeps {
                // x'' = w^2 (target - x) - 2 zeta w x'
                let acc = (target - p) * (w * w) - v * (2.0 * model.damping * w);
                v = v + acc * dt;
                let speed = v.length();
                if speed > model.max_speed {
                    v = v * (model.max_speed / speed);
                }
                p = p + v * dt;
                if lap > 0 {
                    out.push((p, c));
                }
            }
        }
    }
    out
}

/// Long-exposure render: square RGB8 image of the laser field, `size` pixels a side.
/// Brightness is proportional to time spent per pixel (so slow spots glow), tone-mapped.
pub fn expose(samples: &[(Vec2, Rgb)], size: usize) -> Vec<u8> {
    let mut acc = vec![[0.0f32; 3]; size * size];
    for (p, c) in samples {
        if c.is_black() {
            continue;
        }
        let x = ((p.x + 1.0) * 0.5 * (size - 1) as f32).round();
        let y = ((1.0 - p.y) * 0.5 * (size - 1) as f32).round();
        if x < 0.0 || y < 0.0 || x >= size as f32 || y >= size as f32 {
            continue;
        }
        let i = y as usize * size + x as usize;
        acc[i][0] += c.r;
        acc[i][1] += c.g;
        acc[i][2] += c.b;
    }
    // Normalise so a line drawn at a typical speed sits mid-range; hot spots saturate.
    let mut lit: Vec<f32> = acc.iter().map(|a| a[0].max(a[1]).max(a[2])).filter(|&v| v > 0.0).collect();
    lit.sort_by(|a, b| a.total_cmp(b));
    let reference = lit.get(lit.len() / 2).copied().unwrap_or(1.0).max(1e-6) * 2.0;
    let mut img = vec![0u8; size * size * 3];
    for (i, a) in acc.iter().enumerate() {
        for ch in 0..3 {
            let v = (a[ch] / reference).min(4.0);
            let tone = 1.0 - (-v * 1.2).exp(); // soft saturation
            img[i * 3 + ch] = (tone * 255.0) as u8;
        }
    }
    img
}

/// How far the lit beam strays from the intended drawing: RMS and max distance (field units)
/// from each lit simulated sample to the nearest lit segment of the commanded stream.
/// Lag *along* a line doesn't count; overshoot, ringing and rounded corners do.
pub fn path_deviation(frame: &[LaserPoint], pps: u32, model: GalvoModel) -> (f32, f32) {
    let sim = simulate(frame, pps, model, 4, 2);
    let segs: Vec<(Vec2, Vec2)> = (0..frame.len())
        .filter_map(|i| {
            let (a, b) = (frame[i], frame[(i + 1) % frame.len()]);
            (a.is_lit() && b.is_lit()).then_some((a.pos(), b.pos()))
        })
        .collect();
    if segs.is_empty() {
        return (0.0, 0.0);
    }
    let (mut sum, mut max, mut n) = (0.0f32, 0.0f32, 0usize);
    for (k, (p, c)) in sim.iter().enumerate() {
        if c.is_black() || k % 2 == 1 {
            continue;
        }
        let d = segs
            .iter()
            .map(|&(a, b)| crate::vectorise::point_segment_distance(*p, a, b))
            .fold(f32::MAX, f32::min);
        sum += d * d;
        max = max.max(d);
        n += 1;
    }
    ((sum / n.max(1) as f32).sqrt(), max)
}
