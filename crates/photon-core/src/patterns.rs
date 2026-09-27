//! Built-in test patterns (content space, before keystone).

use crate::geom::{Path, Rgb, Vec2};
use serde::{Deserialize, Serialize};

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum TestPattern {
    /// Full-field border with centre cross: use this to set the keystone corners.
    Frame,
    Circle,
    Grid,
    /// Red, green, blue and white lines to check colour channels and balance.
    ColourBars,
    /// Star (sharp corners), small circles (tight curves) and a big circle (speed): tune
    /// lit speed / acceleration / colour delay until all are crisp and closed.
    ScannerTest,
}

impl TestPattern {
    pub const ALL: [TestPattern; 5] = [
        TestPattern::Frame,
        TestPattern::Circle,
        TestPattern::Grid,
        TestPattern::ColourBars,
        TestPattern::ScannerTest,
    ];

    pub fn label(self) -> &'static str {
        match self {
            TestPattern::Frame => "Frame + cross",
            TestPattern::Circle => "Circle",
            TestPattern::Grid => "Grid",
            TestPattern::ColourBars => "Colour bars",
            TestPattern::ScannerTest => "Scanner test (tuning)",
        }
    }

    pub fn paths(self) -> Vec<Path> {
        let line = |a: (f32, f32), b: (f32, f32), c: Rgb| {
            Path::new(vec![Vec2::new(a.0, a.1), Vec2::new(b.0, b.1)], false, c)
        };
        match self {
            TestPattern::Frame => vec![
                Path::new(
                    vec![Vec2::new(-1.0, 1.0), Vec2::new(1.0, 1.0), Vec2::new(1.0, -1.0), Vec2::new(-1.0, -1.0)],
                    true,
                    Rgb::WHITE,
                ),
                line((-0.2, 0.0), (0.2, 0.0), Rgb::new(0.0, 1.0, 0.0)),
                line((0.0, -0.2), (0.0, 0.2), Rgb::new(0.0, 1.0, 0.0)),
                // Arrow pointing up so flips are obvious.
                Path::new(
                    vec![Vec2::new(-0.1, 0.6), Vec2::new(0.0, 0.75), Vec2::new(0.1, 0.6)],
                    false,
                    Rgb::new(1.0, 0.0, 0.0),
                ),
            ],
            TestPattern::Circle => {
                let n = 72;
                let pts = (0..n)
                    .map(|i| {
                        let a = i as f32 / n as f32 * std::f32::consts::TAU;
                        Vec2::new(0.8 * a.cos(), 0.8 * a.sin())
                    })
                    .collect();
                vec![Path::new(pts, true, Rgb::new(0.0, 0.4, 1.0))]
            }
            TestPattern::Grid => {
                let mut v = Vec::new();
                for i in 0..5 {
                    let t = -1.0 + i as f32 * 0.5;
                    v.push(line((t, -1.0), (t, 1.0), Rgb::new(0.0, 1.0, 0.0)));
                    v.push(line((-1.0, t), (1.0, t), Rgb::new(0.0, 1.0, 0.0)));
                }
                v
            }
            TestPattern::ScannerTest => {
                let circle = |cx: f32, cy: f32, r: f32, n: usize, c: Rgb| {
                    let pts = (0..n)
                        .map(|i| {
                            let a = i as f32 / n as f32 * std::f32::consts::TAU;
                            Vec2::new(cx + r * a.cos(), cy + r * a.sin())
                        })
                        .collect();
                    Path::new(pts, true, c)
                };
                let star = (0..10)
                    .map(|i| {
                        let a = i as f32 / 10.0 * std::f32::consts::TAU + std::f32::consts::FRAC_PI_2;
                        let r = if i % 2 == 0 { 0.45 } else { 0.18 };
                        Vec2::new(-0.4 + r * a.cos(), 0.35 + r * a.sin())
                    })
                    .collect();
                let mut v = vec![
                    Path::new(star, true, Rgb::new(1.0, 0.8, 0.0)),
                    circle(0.45, 0.35, 0.4, 72, Rgb::new(0.0, 0.4, 1.0)),
                ];
                for i in 0..5 {
                    v.push(circle(-0.64 + i as f32 * 0.32, -0.55, 0.08, 24, Rgb::new(0.0, 1.0, 0.2)));
                }
                v
            }
            TestPattern::ColourBars => vec![
                line((-0.8, 0.6), (0.8, 0.6), Rgb::new(1.0, 0.0, 0.0)),
                line((-0.8, 0.2), (0.8, 0.2), Rgb::new(0.0, 1.0, 0.0)),
                line((-0.8, -0.2), (0.8, -0.2), Rgb::new(0.0, 0.0, 1.0)),
                line((-0.8, -0.6), (0.8, -0.6), Rgb::WHITE),
            ],
        }
    }
}
