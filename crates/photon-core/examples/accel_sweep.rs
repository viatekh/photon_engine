//! Sweep lit acceleration / corner tolerance through the galvo simulator: points needed vs
//! how far the beam strays. cargo run --release -p photon-core --example accel_sweep
use photon_core::galvo_sim::{path_deviation, GalvoModel};
use photon_core::geom::{Path, Rgb, Vec2};
use photon_core::output::ColourParams;
use photon_core::scan::{render, ScanParams};

fn circle(r: f32, n: usize) -> Path {
    Path::new((0..n).map(|i| { let a = i as f32 / n as f32 * std::f32::consts::TAU; Vec2::new(r * a.cos(), r * a.sin()) }).collect(), true, Rgb::WHITE)
}
fn star() -> Path {
    Path::new((0..10).map(|i| { let a = i as f32 / 10.0 * std::f32::consts::TAU; let r = if i % 2 == 0 { 0.8 } else { 0.35 }; Vec2::new(r * a.cos(), r * a.sin()) }).collect(), true, Rgb::WHITE)
}
fn main() {
    let model = GalvoModel::for_rating(30.0);
    let shapes: Vec<(&str, Vec<Path>)> = vec![
        ("small circles", (0..6).map(|i| { let mut c = circle(0.06, 24); for p in &mut c.points { p.x += -0.6 + i as f32 * 0.24; } c }).collect()),
        ("big circle", vec![circle(0.8, 72)]),
        ("star", vec![star()]),
    ];
    for accel in [0.6e6f32, 1.2e6, 2.5e6, 5e6, 1e7] {
        for tol in [0.003f32, 0.01] {
            let scan = ScanParams { lit_accel: accel, corner_tolerance: tol, ..Default::default() };
            let mut line = format!("accel {accel:>8.1e} tol {tol:.3} |");
            for (name, paths) in &shapes {
                let f = render(paths, &scan);
                let pts = ColourParams { colour_delay_us: 133.0, ..Default::default() }.apply(&f.points, scan.pps);
                let (rms, max) = path_deviation(&pts, scan.pps, model);
                line += &format!(" {name}: {:4} pts rms {:.4} max {:.4} |", pts.len(), rms, max);
            }
            println!("{line}");
        }
    }
}
