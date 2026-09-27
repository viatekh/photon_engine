//! Final per-point processing: colour correction, colour delay and safety checks.

use crate::geom::LaserPoint;
use serde::{Deserialize, Serialize};

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct ColourParams {
    /// Master brightness 0..1.
    pub brightness: f32,
    pub red: f32,
    pub green: f32,
    pub blue: f32,
    /// Level a lit channel starts at, for diodes that don't emit below a threshold.
    /// A lit value v becomes min + v * (1 - min); zero stays zero.
    pub min_level: f32,
    /// Delay colour relative to position, so colour changes line up with where the (lagging)
    /// mirrors actually are. Wicked Lasers' LaserCube default is 4 samples at 30k = ~133 us.
    pub colour_delay_us: f32,
}

impl Default for ColourParams {
    fn default() -> Self {
        Self { brightness: 0.3, red: 1.0, green: 1.0, blue: 1.0, min_level: 0.0, colour_delay_us: 133.0 }
    }
}

impl ColourParams {
    pub fn delay_points(&self, pps: u32) -> usize {
        (self.colour_delay_us.max(0.0) * 1e-6 * pps as f32).round() as usize
    }

    /// Apply to a loopable frame. Delay rotates colour through the loop, so it is seamless.
    pub fn apply(&self, frame: &[LaserPoint], pps: u32) -> Vec<LaserPoint> {
        let n = frame.len();
        if n == 0 {
            return Vec::new();
        }
        let delay = self.delay_points(pps) % n;
        let level = |v: f32, gain: f32| {
            let v = (v * gain * self.brightness).clamp(0.0, 1.0);
            if v <= 0.0 { 0.0 } else { self.min_level + v * (1.0 - self.min_level) }
        };
        (0..n)
            .map(|i| {
                let pos = frame[i];
                let col = frame[(i + n - delay) % n];
                LaserPoint {
                    x: pos.x.clamp(-1.0, 1.0),
                    y: pos.y.clamp(-1.0, 1.0),
                    r: level(col.r, self.red),
                    g: level(col.g, self.green),
                    b: level(col.b, self.blue),
                }
            })
            .collect()
    }
}

/// True if every lit point in the frame is within `min_extent` of the others' bounding box
/// diagonal, i.e. the frame would concentrate the beam on (nearly) one spot.
pub fn is_static_beam(frame: &[LaserPoint], min_extent: f32) -> bool {
    let mut lit = frame.iter().filter(|p| p.is_lit()).peekable();
    if lit.peek().is_none() {
        return false;
    }
    let (mut x0, mut y0, mut x1, mut y1) = (f32::MAX, f32::MAX, f32::MIN, f32::MIN);
    for p in lit {
        x0 = x0.min(p.x);
        y0 = y0.min(p.y);
        x1 = x1.max(p.x);
        y1 = y1.max(p.y);
    }
    ((x1 - x0).powi(2) + (y1 - y0).powi(2)).sqrt() < min_extent
}

pub fn blank_frame(frame: &[LaserPoint]) -> Vec<LaserPoint> {
    frame.iter().map(|p| LaserPoint::blank(p.pos())).collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geom::{Rgb, Vec2};

    #[test]
    fn colour_delay_rotates_colours() {
        let frame: Vec<_> = (0..4)
            .map(|i| {
                let c = if i == 0 { Rgb::WHITE } else { Rgb::BLACK };
                LaserPoint::lit(Vec2::new(i as f32 * 0.1, 0.0), c)
            })
            .collect();
        // 1 point at 30k pps.
        let p = ColourParams { brightness: 1.0, colour_delay_us: 1e6 / 30_000.0, ..Default::default() };
        let out = p.apply(&frame, 30_000);
        assert!(!out[0].is_lit());
        assert!(out[1].is_lit());
        assert_eq!(out[1].x, 0.1);
    }

    #[test]
    fn brightness_and_min_level() {
        let frame = vec![LaserPoint::lit(Vec2::ZERO, Rgb::new(1.0, 0.0, 0.5))];
        let p = ColourParams { brightness: 0.5, min_level: 0.2, colour_delay_us: 0.0, ..Default::default() };
        let out = p.apply(&frame, 30_000)[0];
        assert!((out.r - (0.2 + 0.5 * 0.8)).abs() < 1e-6);
        assert_eq!(out.g, 0.0);
    }

    #[test]
    fn static_beam_detection() {
        let dot = vec![LaserPoint::lit(Vec2::ZERO, Rgb::WHITE); 100];
        assert!(is_static_beam(&dot, 0.05));
        let line = vec![
            LaserPoint::lit(Vec2::new(-0.5, 0.0), Rgb::WHITE),
            LaserPoint::lit(Vec2::new(0.5, 0.0), Rgb::WHITE),
        ];
        assert!(!is_static_beam(&line, 0.05));
        assert!(!is_static_beam(&blank_frame(&dot), 0.05));
    }
}
