//! Calibration session: draws a sequence of test patterns with different scanner / colour
//! settings, and for each step captures what the camera saw next to exactly what was sent to
//! the laser. The result (recordings/calib-<time>.jsonl.gz) is for offline analysis: fitting the
//! galvo model and measuring colour delay, overshoot and settle time on the real hardware.
//!
//! The session temporarily changes the live settings (pattern, scan, colour, planner) and
//! restores them afterwards.

use crate::camera::{CAM_H, CAM_W};
use crate::engine::Shared;
use crate::settings::Settings;
use base64::Engine as _;
use flate2::write::GzEncoder;
use flate2::Compression;
use photon_core::patterns::TestPattern;
use photon_core::planner::Strategy;
use serde_json::json;
use std::io::{BufWriter, Write};
use std::sync::atomic::Ordering;
use std::sync::Arc;
use std::thread;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

const SETTLE: Duration = Duration::from_millis(700);
const CAPTURE: Duration = Duration::from_millis(900);

#[derive(Clone, Default)]
pub struct SessionStatus {
    pub running: bool,
    pub step: usize,
    pub total: usize,
    pub name: String,
    pub message: String,
    /// The user's settings while a session is running (restored afterwards; saved instead of
    /// the session's temporary ones if the app is closed mid-session).
    pub original: Option<Settings>,
}

struct Step {
    name: String,
    apply: Box<dyn Fn(&mut Settings) + Send>,
}

fn step(name: impl Into<String>, apply: impl Fn(&mut Settings) + Send + 'static) -> Step {
    Step { name: name.into(), apply: Box::new(apply) }
}

fn steps() -> Vec<Step> {
    let mut v = Vec::new();
    // Background (laser dark) and registration (slow, accurate) first.
    v.push(step("dark", |s| {
        s.test_pattern = TestPattern::Registration;
        s.colour.brightness = 0.0;
    }));
    v.push(step("registration", |s| {
        s.test_pattern = TestPattern::Registration;
        s.scan.lit_speed = 150.0;
        s.scan.lit_accel = 3e5;
    }));
    // Mirror dynamics: speed x acceleration on sharp corners and tight curves.
    for speed in [400.0f32, 600.0, 900.0] {
        for accel in [0.6e6f32, 1.2e6, 2.5e6, 5e6] {
            v.push(step(format!("scanner speed={speed} accel={accel:.1e}"), move |s| {
                s.test_pattern = TestPattern::ScannerTest;
                s.scan.lit_speed = speed;
                s.scan.lit_accel = accel;
            }));
        }
    }
    // Colour delay: alternating-direction dashes.
    for delay in [0.0f32, 66.0, 133.0, 200.0, 266.0, 333.0, 400.0] {
        v.push(step(format!("delay colour_delay_us={delay}"), move |s| {
            s.test_pattern = TestPattern::DelayComb;
            s.colour.colour_delay_us = delay;
        }));
    }
    // Settle after blanked jumps.
    for post in [0.0f32, 65.0, 130.0, 260.0] {
        v.push(step(format!("jumps blank_post_us={post}"), move |s| {
            s.test_pattern = TestPattern::JumpGrid;
            s.scan.blank_post_us = post;
        }));
    }
    // Big circle at increasing speed: lag / shrink.
    for speed in [300.0f32, 600.0, 1200.0] {
        v.push(step(format!("circle speed={speed}"), move |s| {
            s.test_pattern = TestPattern::Circle;
            s.scan.lit_speed = speed;
        }));
    }
    v
}

pub fn start(shared: Arc<Shared>) {
    {
        let mut st = shared.session.lock();
        if st.running {
            return;
        }
        *st = SessionStatus { running: true, original: Some(shared.settings.read().clone()), ..Default::default() };
    }
    shared.session_abort.store(false, Ordering::SeqCst);
    thread::Builder::new()
        .name("calibration".into())
        .spawn(move || {
            let result = run(&shared);
            let mut st = shared.session.lock();
            if let Some(orig) = st.original.take() {
                *shared.settings.write() = orig;
            }
            st.running = false;
            st.message = match result {
                Ok(msg) => msg,
                Err(e) => format!("Calibration failed: {e:#}"),
            };
            log::info!("{}", st.message);
        })
        .unwrap();
}

fn run(shared: &Arc<Shared>) -> anyhow::Result<String> {
    if shared.camera.lock().latest.is_none() {
        anyhow::bail!("no camera frames - select a camera first");
    }
    if !shared.armed.load(Ordering::SeqCst) {
        anyhow::bail!("arm the laser first (the session draws test patterns)");
    }
    let base = shared.session.lock().original.clone().unwrap();
    let steps = steps();
    std::fs::create_dir_all("recordings")?;
    let secs = SystemTime::now().duration_since(UNIX_EPOCH).map(|d| d.as_secs()).unwrap_or(0);
    let path = std::path::PathBuf::from(format!("recordings/calib-{secs}.jsonl.gz"));
    let mut out = GzEncoder::new(BufWriter::new(std::fs::File::create(&path)?), Compression::default());
    let mut line = |v: serde_json::Value| -> anyhow::Result<()> {
        out.write_all(serde_json::to_string(&v)?.as_bytes())?;
        out.write_all(b"\n")?;
        Ok(())
    };
    line(json!({
        "type": "header",
        "format": 1,
        "kind": "calibration",
        "app": concat!("photon-engine ", env!("CARGO_PKG_VERSION")),
        "unix_time": secs,
        "camera": shared.settings.read().camera,
        "cam_w": CAM_W, "cam_h": CAM_H,
        "base_settings": base,
    }))?;
    let total = steps.len();
    for (i, st) in steps.iter().enumerate() {
        if shared.session_abort.load(Ordering::SeqCst) {
            anyhow::bail!("aborted");
        }
        if !shared.armed.load(Ordering::SeqCst) {
            anyhow::bail!("laser was disarmed - session stopped");
        }
        {
            let mut ss = shared.session.lock();
            ss.step = i + 1;
            ss.total = total;
            ss.name = st.name.clone();
        }
        // Session base: the user's settings, drawing a test pattern with everything visible
        // (no culling, no tracking delays), then this step's changes.
        let mut s = base.clone();
        s.test_pattern_on = true;
        s.planner.strategy = Strategy::AdaptiveRefresh;
        s.planner.min_hz = 5.0;
        s.planner.tracking.enabled = false;
        (st.apply)(&mut s);
        *shared.settings.write() = s.clone();

        thread::sleep(SETTLE);
        // Accumulate camera frames: mean (long exposure) and per-pixel max.
        let mut sum = vec![0u32; CAM_W * CAM_H * 3];
        let mut max = vec![0u8; CAM_W * CAM_H * 3];
        let mut frames = 0u32;
        let mut last_at: Option<Instant> = None;
        let t0 = Instant::now();
        while t0.elapsed() < CAPTURE {
            let f = shared.camera.lock().latest.clone();
            if let Some(f) = f {
                if last_at != Some(f.at) {
                    last_at = Some(f.at);
                    for (k, &v) in f.rgb.iter().enumerate() {
                        sum[k] += v as u32;
                        max[k] = max[k].max(v);
                    }
                    frames += 1;
                }
            }
            thread::sleep(Duration::from_millis(5));
        }
        let mean: Vec<u8> = sum.iter().map(|&v| (v / frames.max(1)) as u8).collect();
        // Exactly what was being sent to the DAC (after colour delay etc.).
        let sent = shared.monitor.lock().clone();
        let pts: Vec<f32> = sent
            .iter()
            .flat_map(|p| {
                let r = |v: f32| (v * 1e4).round() / 1e4;
                [r(p.x), r(p.y), r(p.r), r(p.g), r(p.b)]
            })
            .collect();
        let b64 = |v: &[u8]| base64::engine::general_purpose::STANDARD.encode(v);
        line(json!({
            "type": "step",
            "index": i,
            "name": st.name,
            "settings": s,
            "camera_frames": frames,
            "mean_rgb": b64(&mean),
            "max_rgb": b64(&max),
            "sent_xyrgb": pts,
            "output": {
                "sent_pps": shared.output_status.lock().sent_pps,
                "underruns": shared.output_status.lock().underruns,
            },
        }))?;
    }
    drop(line);
    out.finish()?.flush()?;
    let full = std::fs::canonicalize(&path).unwrap_or(path);
    Ok(format!("Calibration saved: {}", full.display()))
}
