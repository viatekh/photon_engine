//! Session recorder: captures exactly what the pipeline saw and decided, frame by frame, so a
//! problem seen on the laser (flicker, jumping, wrong shapes) can be analysed and replayed
//! offline. Output is gzipped JSON lines:
//!
//! * `{"type":"header", ...}` once
//! * `{"type":"settings", "t":.., "settings":{..}}` at start and whenever settings change
//! * `{"type":"frame", "t":.., ...}` per processed frame: the working image fed to the tracer
//!   (RGB8, base64), every traced path (drawn or dropped), planner stats, auto-detail level
//!   and output state.

use crate::engine::OutputStatus;
use crate::settings::Settings;
use base64::Engine as _;
use flate2::write::GzEncoder;
use flate2::Compression;
use photon_core::image::WorkImage;
use photon_core::planner::Plan;
use photon_core::Path;
use serde_json::{json, Value};
use std::fs::File;
use std::io::{BufWriter, Write};
use std::path::PathBuf;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

pub const MAX_DURATION: Duration = Duration::from_secs(20);

#[derive(Clone, Default)]
pub struct RecordingStatus {
    pub active: bool,
    pub seconds: f32,
    pub frames: u64,
    pub megabytes: f32,
    /// Last finished recording (or error).
    pub message: String,
}

pub struct Recorder {
    enc: GzEncoder<BufWriter<File>>,
    path: PathBuf,
    start: Instant,
    frames: u64,
    raw_bytes: u64,
    last_settings: Option<Settings>,
    /// Replay: use the original recording's timestamps instead of the wall clock.
    clock: Option<f64>,
}

impl Recorder {
    /// Start a new recording in `recordings/`.
    pub fn start() -> anyhow::Result<Self> {
        let dir = PathBuf::from("recordings");
        std::fs::create_dir_all(&dir)?;
        let secs = SystemTime::now().duration_since(UNIX_EPOCH).map(|d| d.as_secs()).unwrap_or(0);
        Self::create(dir.join(format!("rec-{secs}.jsonl.gz")))
    }

    pub fn create(path: PathBuf) -> anyhow::Result<Self> {
        let secs = SystemTime::now().duration_since(UNIX_EPOCH).map(|d| d.as_secs()).unwrap_or(0);
        let file = File::create(&path)?;
        let mut rec = Self {
            enc: GzEncoder::new(BufWriter::new(file), Compression::fast()),
            path,
            start: Instant::now(),
            frames: 0,
            raw_bytes: 0,
            last_settings: None,
            clock: None,
        };
        rec.line(&json!({
            "type": "header",
            "format": 1,
            "app": concat!("photon-engine ", env!("CARGO_PKG_VERSION")),
            "unix_time": secs,
            "os": std::env::consts::OS,
            "arch": std::env::consts::ARCH,
        }))?;
        Ok(rec)
    }

    pub fn set_clock(&mut self, t: f64) {
        self.clock = Some(t);
    }

    pub fn elapsed(&self) -> Duration {
        self.start.elapsed()
    }

    pub fn frames(&self) -> u64 {
        self.frames
    }

    /// Uncompressed bytes written so far (compressed size is typically much smaller).
    pub fn raw_megabytes(&self) -> f32 {
        self.raw_bytes as f32 / 1e6
    }

    fn line(&mut self, v: &Value) -> anyhow::Result<()> {
        let s = serde_json::to_string(v)?;
        self.raw_bytes += s.len() as u64 + 1;
        self.enc.write_all(s.as_bytes())?;
        self.enc.write_all(b"\n")?;
        Ok(())
    }

    #[allow(clippy::too_many_arguments)]
    pub fn record(
        &mut self,
        settings: &Settings,
        image: Option<&WorkImage>,
        plan: &Plan,
        detail_level: f32,
        process_ms: f32,
        armed: bool,
        output: &OutputStatus,
    ) -> anyhow::Result<()> {
        let t = self.clock.unwrap_or_else(|| self.start.elapsed().as_secs_f64());
        if self.last_settings.as_ref() != Some(settings) {
            self.line(&json!({ "type": "settings", "t": t, "settings": settings }))?;
            self.last_settings = Some(settings.clone());
        }
        let st = &plan.stats;
        let mut frame = json!({
            "type": "frame",
            "t": t,
            "level": detail_level,
            "ms": process_ms,
            "armed": armed,
            "out_blank": output.blanked_reason,
            "out_passes": output.frames_per_sec,
            "stats": {
                "input_paths": st.input_paths, "drawn_paths": st.drawn_paths,
                "input_shapes": st.input_shapes, "drawn_shapes": st.drawn_shapes,
                "points": st.points, "budget": st.budget, "refresh_hz": st.refresh_hz,
                "simplify": st.simplify_used, "groups": st.groups,
                "demand": st.demand, "demand_controlled": st.demand_controlled, "capacity": st.capacity,
            },
            "frame_points": plan.frames.iter().map(|f| f.points.len()).collect::<Vec<_>>(),
            "drawn": plan.drawn.iter().map(path_json).collect::<Vec<_>>(),
            "dropped": plan.dropped.iter().map(path_json).collect::<Vec<_>>(),
        });
        if let Some(img) = image {
            let mut rgb = Vec::with_capacity(img.pixels.len() * 3);
            for c in &img.pixels {
                for v in [c.r, c.g, c.b] {
                    rgb.push((v.clamp(0.0, 1.0) * 255.0).round() as u8);
                }
            }
            frame["image"] = json!({
                "w": img.width,
                "h": img.height,
                "rgb": base64::engine::general_purpose::STANDARD.encode(&rgb),
            });
        }
        self.line(&frame)?;
        self.frames += 1;
        Ok(())
    }

    pub fn finish(mut self) -> anyhow::Result<PathBuf> {
        self.enc.flush()?;
        self.enc.finish()?.flush()?;
        Ok(self.path)
    }
}

fn round(v: f32, digits: i32) -> f32 {
    let m = 10f32.powi(digits);
    (v * m).round() / m
}

fn path_json(p: &Path) -> Value {
    let pts: Vec<f32> = p.points.iter().flat_map(|v| [round(v.x, 4), round(v.y, 4)]).collect();
    json!({
        "g": p.group,
        "c": [round(p.color.r, 2), round(p.color.g, 2), round(p.color.b, 2)],
        "wt": round(p.weight, 3),
        "cl": p.closed,
        "p": pts,
    })
}
