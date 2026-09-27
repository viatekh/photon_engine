//! Render test patterns through the galvo simulator as long-exposure images, comparing scan
//! settings. Usage: cargo run --release -p photon-core --example beam_preview -- OUT_DIR

use photon_core::galvo_sim::{expose, path_deviation, simulate, GalvoModel};
use photon_core::geom::{Path, Rgb, Vec2};
use photon_core::output::ColourParams;
use photon_core::patterns::TestPattern;
use photon_core::scan::{render, ScanParams};
use std::io::Write;

fn star() -> Vec<Path> {
    let pts = (0..10)
        .map(|i| {
            let a = i as f32 / 10.0 * std::f32::consts::TAU + std::f32::consts::FRAC_PI_2;
            let r = if i % 2 == 0 { 0.85 } else { 0.35 };
            Vec2::new(r * a.cos(), r * a.sin())
        })
        .collect();
    vec![Path::new(pts, true, Rgb::new(1.0, 0.8, 0.0))]
}

fn save(path: &str, img: &[u8], size: usize) {
    let mut f = std::fs::File::create(path).unwrap();
    write!(f, "P6 {size} {size} 255\n").unwrap();
    f.write_all(img).unwrap();
}

fn main() {
    let out = std::env::args().nth(1).unwrap_or_else(|| ".".into());
    // "Constant speed": acceleration effectively unlimited (like the previous engine).
    let constant = ScanParams { lit_accel: 1e12, blank_accel: 1e12, lit_speed: 450.0, path_dwell_us: 70.0, ..Default::default() };
    let variants: Vec<(&str, ScanParams, f32)> = vec![
        ("constant_speed_no_colour_delay", constant.clone(), 0.0),
        ("accel_limited", ScanParams::default(), ColourParams::default().colour_delay_us),
    ];
    let shapes: Vec<(&str, Vec<Path>)> = vec![
        ("circle", TestPattern::Circle.paths()),
        ("frame", TestPattern::Frame.paths()),
        ("star", star()),
        ("grid", TestPattern::Grid.paths()),
    ];
    let model = GalvoModel::for_rating(30.0);
    for (sname, paths) in &shapes {
        for (vname, scan, delay) in &variants {
            let frame = render(paths, scan);
            let colour = ColourParams { brightness: 1.0, colour_delay_us: *delay, ..Default::default() };
            let pts = colour.apply(&frame.points, scan.pps);
            let (rms, max) = path_deviation(&pts, scan.pps, model);
            let sim = simulate(&pts, scan.pps, model, 16, 4);
            let img = expose(&sim, 420);
            let file = format!("{out}/{sname}_{vname}.ppm");
            save(&file, &img, 420);
            println!(
                "{sname:7} {vname:32} points {:5}  refresh {:5.0} Hz  deviation rms {:.4} max {:.4}",
                pts.len(),
                scan.pps as f32 / pts.len() as f32,
                rms,
                max
            );
        }
    }
}
