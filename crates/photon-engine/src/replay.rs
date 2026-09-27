//! Headless replay: re-run a recording's input frames through the current tracing and planning
//! code, with the recording's settings, and write the result as a new recording. Used to test a
//! change against real content:
//!
//!     photon-engine --replay recordings/rec-123.jsonl.gz /tmp/after.jsonl.gz
//!     python3 tools/analyse_recording.py /tmp/after.jsonl.gz

use crate::engine::OutputStatus;
use crate::recorder::Recorder;
use crate::settings::Settings;
use anyhow::Context;
use base64::Engine as _;
use flate2::read::GzDecoder;
use photon_core::detail::AutoDetail;
use photon_core::image::WorkImage;
use photon_core::planner::Planner;
use photon_core::vectorise::vectorise;
use photon_core::Rgb;
use serde_json::Value;
use std::io::{BufRead, BufReader};
use std::path::PathBuf;
use std::time::Instant;

pub fn run(input: &str, output: &str) -> anyhow::Result<()> {
    let file = std::fs::File::open(input).with_context(|| format!("open {input}"))?;
    let reader = BufReader::new(GzDecoder::new(file));
    let mut rec = Recorder::create(PathBuf::from(output))?;
    let mut settings = Settings::default();
    let mut planner = Planner::new();
    let mut detail = AutoDetail::default();
    let out_status = OutputStatus::default();
    let mut n = 0;
    for line in reader.lines() {
        let line = line?;
        if line.trim().is_empty() {
            continue;
        }
        let v: Value = serde_json::from_str(&line)?;
        match v["type"].as_str() {
            Some("settings") => {
                settings = serde_json::from_value(v["settings"].clone()).context("settings")?;
            }
            Some("frame") => {
                let Some(img) = v.get("image") else { continue };
                let (w, h) = (img["w"].as_u64().unwrap_or(0) as usize, img["h"].as_u64().unwrap_or(0) as usize);
                let rgb = base64::engine::general_purpose::STANDARD.decode(img["rgb"].as_str().unwrap_or(""))?;
                let mut image = WorkImage::new(w, h);
                for (px, c) in image.pixels.iter_mut().zip(rgb.chunks_exact(3)) {
                    *px = Rgb::new(c[0] as f32 / 255.0, c[1] as f32 / 255.0, c[2] as f32 / 255.0);
                }
                let t0 = Instant::now();
                let level = detail.level;
                let vp = detail.apply(&settings.vectorise, &settings.auto_detail);
                let paths = settings.geometry.apply(&vectorise(&image, &vp));
                let plan = planner.plan(paths, &settings.scan, &settings.planner);
                detail.update(&plan.stats, &settings.auto_detail);
                let ms = t0.elapsed().as_secs_f32() * 1000.0;
                let armed = v["armed"].as_bool().unwrap_or(false);
                rec.set_clock(v["t"].as_f64().unwrap_or(0.0));
                rec.record(&settings, Some(&image), &plan, level, ms, armed, &out_status)?;
                n += 1;
            }
            _ => {}
        }
    }
    let path = rec.finish()?;
    eprintln!("replayed {n} frames -> {}", path.display());
    Ok(())
}
