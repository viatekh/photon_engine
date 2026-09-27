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

// ------------------------------------------------------------------------------------------------
// Arbitrary content: a Mandelbrot render and a soft "photo" through Edges + auto detail.

use photon_core::detail::{AutoDetail, AutoDetailParams};

fn mandelbrot(w: usize, h: usize, zoom: f32) -> Vec<u8> {
    let mut data = vec![0u8; w * h * 4];
    let (cx, cy) = (-0.743_643_9, 0.131_825_9);
    let scale = 3.0 / (w as f32 * zoom);
    for py in 0..h {
        for px in 0..w {
            let x0 = cx + (px as f32 - w as f32 / 2.0) * scale;
            let y0 = cy + (py as f32 - h as f32 / 2.0) * scale;
            let (mut x, mut y, mut i) = (0.0f32, 0.0f32, 0);
            while x * x + y * y < 4.0 && i < 200 {
                let t = x * x - y * y + x0;
                y = 2.0 * x * y + y0;
                x = t;
                i += 1;
            }
            let k = (px + py * w) * 4;
            let v = if i == 200 { 0 } else { ((i * 13) % 256) as u8 };
            data[k..k + 4].copy_from_slice(&[v, v / 2, 255 - v, 255]);
        }
    }
    data
}

fn soft_photo(w: usize, h: usize) -> Vec<u8> {
    let mut data = vec![0u8; w * h * 4];
    for y in 0..h {
        for x in 0..w {
            let (fx, fy) = (x as f32 / w as f32, y as f32 / h as f32);
            let v = 0.5 + 0.25 * (fx * 9.0).sin() * (fy * 7.0).cos() + 0.25 * ((fx - 0.5).hypot(fy - 0.5) * 20.0).sin();
            let b = (v.clamp(0.0, 1.0) * 255.0) as u8;
            let k = (y * w + x) * 4;
            data[k..k + 4].copy_from_slice(&[b, b, (b as f32 * 0.8) as u8, 255]);
        }
    }
    data
}

/// Run a static frame through auto detail for a while; return final (demand, capacity, drawn, input).
fn settle(data: &[u8], w: usize, h: usize, strategy: Strategy) -> (usize, usize, usize, usize) {
    let base = VectoriseParams { mode: TraceMode::Edges, ..Default::default() };
    let auto = AutoDetailParams::default();
    let mut detail = AutoDetail::default();
    let mut planner = Planner::new();
    let scan = ScanParams::default();
    let params = PlannerParams { strategy, ..Default::default() };
    let img = WorkImage::from_frame(data, w, h, w * 4, PixelOrder::Rgba, base.resolution, false);
    let mut last = (0, 0, 0, 0);
    for _ in 0..40 {
        let vp = detail.apply(&base, &auto);
        let paths = vectorise(&img, &vp);
        let plan = planner.plan(paths, &scan, &params);
        for f in &plan.frames {
            assert!(f.points.len() <= plan.stats.budget);
        }
        detail.update(&plan.stats, &auto);
        if std::env::var("DBG").is_ok() {
            eprintln!("level {:.2} demand {} ctl {} cap {} drawn {}/{}", detail.level, plan.stats.demand, plan.stats.demand_controlled, plan.stats.capacity, plan.stats.drawn_paths, plan.stats.input_paths);
        }
        last = (plan.stats.demand, plan.stats.capacity, plan.stats.drawn_paths, plan.stats.input_paths);
    }
    last
}

#[test]
fn fractal_settles_near_budget() {
    let data = mandelbrot(960, 540, 40.0);
    for strategy in [Strategy::Combined, Strategy::WholeShapes] {
        let (demand, capacity, drawn, input) = settle(&data, 960, 540, strategy);
        assert!(drawn > 0, "{strategy:?}");
        // Settled within a factor of ~2 of capacity rather than 10x over.
        assert!(demand < capacity * 2, "{strategy:?}: demand {demand} capacity {capacity} ({drawn}/{input})");
    }
}

#[test]
fn soft_photo_produces_lines() {
    let data = soft_photo(640, 360);
    let (_, _, drawn, _) = settle(&data, 640, 360, Strategy::Combined);
    assert!(drawn >= 3, "{drawn}");
}

#[test]
#[ignore = "timing; run with --release -- --ignored --nocapture"]
fn timing_fractal() {
    let data = mandelbrot(1920, 1080, 40.0);
    let vp = VectoriseParams { mode: TraceMode::Edges, ..Default::default() };
    let t = Instant::now();
    let n = 10;
    let mut paths_n = 0;
    for _ in 0..n {
        let img = WorkImage::from_frame(&data, W, H, W * 4, PixelOrder::Rgba, vp.resolution, false);
        let paths = vectorise(&img, &vp);
        paths_n = paths.len();
        let _ = Planner::new().plan(paths, &ScanParams::default(), &PlannerParams::default());
    }
    println!("Edges/fractal: {paths_n} paths, {:.2} ms/frame", t.elapsed().as_secs_f64() * 1000.0 / n as f64);
}

// ------------------------------------------------------------------------------------------------
// Auto mode on laser-style content (the case from the first on-device screenshot).

fn rings_frame(rings: &[(f32, f32, f32)], line_y: Option<f32>) -> Vec<u8> {
    let (w, h) = (960usize, 540usize);
    let mut data = vec![0u8; w * h * 4];
    let mut plot = |x: f32, y: f32| {
        for dy in -1..=1 {
            for dx in -1..=1 {
                let (px, py) = (x as i32 + dx, y as i32 + dy);
                if px >= 0 && py >= 0 && (px as usize) < w && (py as usize) < h {
                    let i = (py as usize * w + px as usize) * 4;
                    data[i..i + 4].copy_from_slice(&[0, 255, 60, 255]);
                }
            }
        }
    };
    for &(cx, cy, r) in rings {
        let steps = (r * 8.0) as usize;
        for s in 0..steps {
            let a = s as f32 / steps as f32 * std::f32::consts::TAU;
            plot(cx + r * a.cos(), cy + r * a.sin());
        }
    }
    if let Some(y) = line_y {
        for x in 60..900 {
            plot(x as f32, y);
        }
    }
    data
}

fn auto_paths(data: &[u8]) -> Vec<photon_core::Path> {
    let vp = VectoriseParams::default();
    assert_eq!(vp.mode, TraceMode::Auto);
    let img = WorkImage::from_frame(data, 960, 540, 960 * 4, PixelOrder::Rgba, vp.resolution, false);
    vectorise(&img, &vp)
}

#[test]
fn auto_traces_separate_rings_once_each() {
    let data = rings_frame(&[(200.0, 200.0, 40.0), (500.0, 300.0, 60.0), (750.0, 150.0, 30.0)], None);
    let paths = auto_paths(&data);
    assert_eq!(paths.len(), 3, "{:#?}", paths.iter().map(|p| (p.closed, p.points.len())).collect::<Vec<_>>());
    assert!(paths.iter().all(|p| p.closed));
    // Each ring is its own shape.
    let mut groups: Vec<u32> = paths.iter().map(|p| p.group).collect();
    groups.dedup();
    assert_eq!(groups.len(), 3);
}

#[test]
fn auto_line_is_single_stroke_not_a_loop() {
    let data = rings_frame(&[], Some(270.0));
    let paths = auto_paths(&data);
    assert_eq!(paths.len(), 1);
    assert!(!paths[0].closed);
}

#[test]
fn overlapping_rings_form_one_shape() {
    let data = rings_frame(&[(400.0, 270.0, 60.0), (470.0, 270.0, 60.0)], None);
    let paths = auto_paths(&data);
    assert!(paths.len() >= 2);
    let g = paths[0].group;
    assert!(paths.iter().all(|p| p.group == g), "{:?}", paths.iter().map(|p| p.group).collect::<Vec<_>>());
}

#[test]
fn auto_filled_shape_uses_its_outline() {
    // A solid disc (too wide to be a stroke) should come out as one closed outline.
    let (w, h) = (960usize, 540usize);
    let mut data = vec![0u8; w * h * 4];
    for y in 0..h {
        for x in 0..w {
            if ((x as f32 - 480.0).powi(2) + (y as f32 - 270.0).powi(2)).sqrt() < 120.0 {
                data[(y * w + x) * 4..(y * w + x) * 4 + 4].copy_from_slice(&[255, 255, 255, 255]);
            }
        }
    }
    let paths = auto_paths(&data);
    assert_eq!(paths.len(), 1, "{}", paths.len());
    assert!(paths[0].closed);
}

#[test]
#[ignore = "diagnostic"]
fn stroke_stats() {
    let vp = VectoriseParams::default();
    eprintln!("--- rings");
    let data = rings_frame(&[(400.0, 270.0, 60.0), (470.0, 270.0, 60.0), (430.0, 330.0, 50.0), (520.0, 300.0, 40.0), (360.0, 220.0, 35.0)], Some(250.0));
    let img = WorkImage::from_frame(&data, 960, 540, 960 * 4, PixelOrder::Rgba, vp.resolution, false);
    vectorise(&img, &vp);
    for zoom in [40.0, 400.0, 4000.0] {
        eprintln!("--- fractal zoom {zoom}");
        let data = mandelbrot(960, 540, zoom);
        let img = WorkImage::from_frame(&data, 960, 540, 960 * 4, PixelOrder::Rgba, vp.resolution, false);
        vectorise(&img, &vp);
    }
}
