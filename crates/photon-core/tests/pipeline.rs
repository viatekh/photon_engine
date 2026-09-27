//! End-to-end: a synthetic 1080p frame through the whole core pipeline.

use photon_core::image::{PixelOrder, WorkImage};
use photon_core::keystone::OutputGeometry;
use photon_core::output::ColourParams;
use photon_core::planner::{Planner, PlannerParams, Strategy};
use photon_core::scan::ScanParams;
use photon_core::vectorise::{vectorise, TraceMode, VectoriseParams};
use photon_core::Vec2;
use std::time::Instant;

const W: usize = 1920;
const H: usize = 1080;

/// Thin anti-aliased-ish rings and lines on black, like typical laser-oriented VJ content.
fn frame(rings: usize) -> Vec<u8> {
    let mut data = vec![0u8; W * H * 4];
    let mut plot = |x: i32, y: i32, c: [u8; 3]| {
        for dy in -2..=2 {
            for dx in -2..=2 {
                let (px, py) = (x + dx, y + dy);
                if px >= 0 && py >= 0 && (px as usize) < W && (py as usize) < H {
                    let i = (py as usize * W + px as usize) * 4;
                    data[i..i + 3].copy_from_slice(&c);
                    data[i + 3] = 255;
                }
            }
        }
    };
    for k in 0..rings {
        let cx = 200.0 + (k % 8) as f32 * 220.0;
        let cy = 150.0 + (k / 8) as f32 * 220.0;
        let r = 40.0 + (k % 3) as f32 * 25.0;
        for s in 0..720 {
            let a = s as f32 / 720.0 * std::f32::consts::TAU;
            plot((cx + r * a.cos()) as i32, (cy + r * a.sin()) as i32, [0, 255, 80]);
        }
    }
    for x in 100..1820 {
        plot(x, 1000, [255, 0, 0]);
    }
    data
}

fn run(rings: usize, mode: TraceMode, strategy: Strategy) -> (usize, usize, usize, usize) {
    let data = frame(rings);
    let vp = VectoriseParams { mode, ..Default::default() };
    let img = WorkImage::from_frame(&data, W, H, W * 4, PixelOrder::Rgba, vp.resolution, false);
    let paths = vectorise(&img, &vp);
    let geometry = OutputGeometry::default();
    let paths = geometry.apply(&paths);
    let scan = ScanParams::default();
    let params = PlannerParams { strategy, ..Default::default() };
    let plan = Planner::new().plan(paths, &scan, &params);
    for f in &plan.frames {
        assert!(f.points.len() <= plan.stats.budget);
        let out = ColourParams::default().apply(&f.points);
        assert!(out.iter().all(|p| p.x.abs() <= 1.0 && p.y.abs() <= 1.0));
    }
    (plan.stats.input_paths, plan.stats.drawn_paths, plan.stats.points, plan.stats.budget)
}

#[test]
fn simple_frame_is_drawn_completely() {
    for mode in [TraceMode::Centreline, TraceMode::Outline] {
        let (input, drawn, _, _) = run(3, mode, Strategy::WholeShapes);
        assert!(input >= 4, "{mode:?}: {input}");
        assert_eq!(input, drawn, "{mode:?}");
    }
}

#[test]
fn busy_frame_is_culled_within_budget() {
    for strategy in Strategy::ALL {
        let (input, drawn, points, budget) = run(32, TraceMode::Centreline, strategy);
        assert!(drawn > 0 && points <= budget, "{strategy:?}");
        if strategy != Strategy::TakeTurns {
            assert!(drawn < input, "{strategy:?}: expected culling ({drawn}/{input})");
        }
    }
}

#[test]
fn keystone_keeps_everything_in_field() {
    let geometry = OutputGeometry {
        corners: [Vec2::new(-0.6, 0.9), Vec2::new(0.6, 0.9), Vec2::new(1.0, -1.0), Vec2::new(-1.0, -1.0)],
        ..Default::default()
    };
    let paths = geometry.apply(&photon_core::patterns::TestPattern::Grid.paths());
    for p in &paths {
        assert!(p.points.iter().all(|v| v.x.abs() <= 1.0 + 1e-5 && v.y.abs() <= 1.0 + 1e-5));
    }
}

#[test]
#[ignore = "timing; run with --release -- --ignored --nocapture"]
fn timing() {
    let data = frame(32);
    for mode in [TraceMode::Centreline, TraceMode::Outline] {
        let vp = VectoriseParams { mode, ..Default::default() };
        let t = Instant::now();
        let n = 20;
        for _ in 0..n {
            let img = WorkImage::from_frame(&data, W, H, W * 4, PixelOrder::Rgba, vp.resolution, false);
            let paths = vectorise(&img, &vp);
            let _ = Planner::new().plan(paths, &ScanParams::default(), &PlannerParams::default());
        }
        println!("{mode:?}: {:.2} ms/frame", t.elapsed().as_secs_f64() * 1000.0 / n as f64);
    }
}
