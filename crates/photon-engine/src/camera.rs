//! Webcam capture, for measuring the real laser output (development / calibration).
//!
//! Real cameras are read through the `ffmpeg` command-line tool (macOS: AVFoundation), which is
//! the most dependable way to get frames from USB webcams without platform-specific bindings:
//! `brew install ffmpeg`. A simulated camera renders the galvo model of what is being sent to the
//! laser, so the whole calibration flow can be tested without hardware.

use crate::engine::Shared;
use parking_lot::Mutex;
use photon_core::galvo_sim::{simulate, GalvoModel};
use photon_core::keystone::Homography;
use photon_core::Vec2;
use serde::{Deserialize, Serialize};
use std::io::Read;
use std::process::{Child, Command, Stdio};
use std::sync::atomic::Ordering;
use std::sync::Arc;
use std::thread;
use std::time::{Duration, Instant};

/// Frames are delivered at this size (the camera's own aspect is squashed to fit; calibration
/// maps camera pixels to laser coordinates with a homography, so that doesn't matter).
pub const CAM_W: usize = 960;
pub const CAM_H: usize = 540;

#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub enum CameraSelection {
    #[default]
    None,
    /// Renders the galvo simulation of the laser output (no hardware needed).
    Simulated,
    /// AVFoundation device index and name, as listed by ffmpeg.
    Device { index: usize, name: String },
}

impl CameraSelection {
    pub fn label(&self) -> String {
        match self {
            CameraSelection::None => "None".into(),
            CameraSelection::Simulated => "Simulated (galvo model)".into(),
            CameraSelection::Device { index, name } => format!("[{index}] {name}"),
        }
    }
}

/// One RGB8 camera frame, CAM_W x CAM_H.
pub struct CamFrame {
    pub rgb: Vec<u8>,
    pub at: Instant,
}

#[derive(Default)]
pub struct CameraState {
    pub latest: Option<Arc<CamFrame>>,
    pub message: String,
    pub frames: u64,
}

/// Video devices ffmpeg can see (macOS AVFoundation). Empty with a note if ffmpeg is missing.
pub fn list_devices() -> (Vec<CameraSelection>, String) {
    let out = Command::new("ffmpeg")
        .args(["-hide_banner", "-f", "avfoundation", "-list_devices", "true", "-i", ""])
        .stdin(Stdio::null())
        .output();
    let out = match out {
        Ok(o) => o,
        Err(_) => return (Vec::new(), "ffmpeg not found - install with: brew install ffmpeg".into()),
    };
    let text = String::from_utf8_lossy(&out.stderr);
    let mut devices = Vec::new();
    let mut in_video = false;
    for line in text.lines() {
        if line.contains("AVFoundation video devices") {
            in_video = true;
            continue;
        }
        if line.contains("AVFoundation audio devices") {
            break;
        }
        if !in_video {
            continue;
        }
        // "[AVFoundation indev @ 0x...] [0] FaceTime HD Camera"
        if let Some(rest) = line.rsplit_once("] [").map(|(_, r)| r) {
            if let Some((idx, name)) = rest.split_once("] ") {
                if let Ok(index) = idx.trim().parse() {
                    if !name.starts_with("Capture screen") {
                        devices.push(CameraSelection::Device { index, name: name.trim().to_string() });
                    }
                }
            }
        }
    }
    let note = if devices.is_empty() { "No cameras found by ffmpeg".to_string() } else { String::new() };
    (devices, note)
}

/// Background thread: keeps `shared.camera.latest` fresh for the selected camera.
pub fn start(shared: Arc<Shared>) -> thread::JoinHandle<()> {
    thread::Builder::new().name("camera".into()).spawn(move || run(shared)).unwrap()
}

fn set_msg(state: &Mutex<CameraState>, msg: impl Into<String>) {
    let msg = msg.into();
    let mut st = state.lock();
    if st.message != msg {
        log::info!("camera: {msg}");
        st.message = msg;
    }
}

fn run(shared: Arc<Shared>) {
    let mut current = CameraSelection::None;
    let mut child: Option<Child> = None;
    let mut reader: Option<thread::JoinHandle<()>> = None;
    let sim_warp = Homography::from_points(
        [Vec2::new(-1.0, 1.0), Vec2::new(1.0, 1.0), Vec2::new(1.0, -1.0), Vec2::new(-1.0, -1.0)],
        // A slightly off-axis camera view, in camera pixels.
        [Vec2::new(250.0, 40.0), Vec2::new(735.0, 70.0), Vec2::new(720.0, 500.0), Vec2::new(230.0, 480.0)],
    )
    .unwrap();

    while !shared.shutdown.load(Ordering::Relaxed) {
        let wanted = shared.settings.read().camera.clone();
        if wanted != current {
            if let Some(mut c) = child.take() {
                let _ = c.kill();
                let _ = c.wait();
            }
            if let Some(r) = reader.take() {
                let _ = r.join();
            }
            shared.camera.lock().latest = None;
            current = wanted.clone();
            if let CameraSelection::Device { index, .. } = &current {
                match spawn_ffmpeg(*index) {
                    Ok((c, r)) => {
                        child = Some(c);
                        let sh = shared.clone();
                        reader = Some(thread::spawn(move || read_frames(sh, r)));
                        set_msg(&shared.camera, format!("Capturing {}", current.label()));
                    }
                    Err(e) => set_msg(&shared.camera, format!("{e:#}")),
                }
            } else if current == CameraSelection::None {
                set_msg(&shared.camera, "");
            }
        }
        match &current {
            CameraSelection::Simulated => {
                let frame = simulated_frame(&shared, &sim_warp);
                let mut st = shared.camera.lock();
                st.latest = Some(Arc::new(frame));
                st.frames += 1;
                st.message = "Simulated camera".into();
                drop(st);
                thread::sleep(Duration::from_millis(33));
            }
            CameraSelection::Device { .. } => {
                if let Some(c) = child.as_mut() {
                    if let Ok(Some(status)) = c.try_wait() {
                        set_msg(&shared.camera, format!("ffmpeg stopped ({status}); check the camera and permissions"));
                        child = None;
                    }
                }
                thread::sleep(Duration::from_millis(100));
            }
            CameraSelection::None => thread::sleep(Duration::from_millis(100)),
        }
    }
    if let Some(mut c) = child {
        let _ = c.kill();
    }
}

fn spawn_ffmpeg(index: usize) -> anyhow::Result<(Child, std::process::ChildStdout)> {
    // Webcams only accept frame rates they support; 30 is near universal, and ffmpeg's
    // AVFoundation input picks the device's default size. Output is scaled to CAM_W x CAM_H.
    let mut child = Command::new("ffmpeg")
        .args([
            "-hide_banner",
            "-loglevel",
            "error",
            "-f",
            "avfoundation",
            "-framerate",
            "30",
            "-i",
            &format!("{index}:none"),
            "-vf",
            &format!("scale={CAM_W}:{CAM_H}"),
            "-pix_fmt",
            "rgb24",
            "-f",
            "rawvideo",
            "-",
        ])
        .stdin(Stdio::null())
        .stdout(Stdio::piped())
        .stderr(Stdio::inherit())
        .spawn()
        .map_err(|e| anyhow::anyhow!("could not start ffmpeg ({e}); install with: brew install ffmpeg"))?;
    let out = child.stdout.take().unwrap();
    Ok((child, out))
}

fn read_frames(shared: Arc<Shared>, mut out: std::process::ChildStdout) {
    let size = CAM_W * CAM_H * 3;
    loop {
        let mut buf = vec![0u8; size];
        if out.read_exact(&mut buf).is_err() {
            return;
        }
        let mut st = shared.camera.lock();
        st.latest = Some(Arc::new(CamFrame { rgb: buf, at: Instant::now() }));
        st.frames += 1;
    }
}

/// What a camera would see: the galvo simulation of the frame currently being sent, as a
/// ~1/30 s exposure, seen through a slightly off-axis perspective.
fn simulated_frame(shared: &Shared, warp: &Homography) -> CamFrame {
    let monitor = shared.monitor.lock().clone();
    let settings = shared.settings.read().clone();
    let mut acc = vec![0.0f32; CAM_W * CAM_H * 3];
    if shared.armed.load(Ordering::Relaxed) && !monitor.is_empty() {
        let model = GalvoModel::for_rating(settings.scan.scanner_kpps);
        let samples = simulate(&monitor, settings.scan.pps, model, 4, 3);
        for (p, c) in samples {
            if c.is_black() {
                continue;
            }
            let q = warp.apply(p);
            let (x, y) = (q.x.round() as isize, q.y.round() as isize);
            if x < 0 || y < 0 || x >= CAM_W as isize || y >= CAM_H as isize {
                continue;
            }
            let i = (y as usize * CAM_W + x as usize) * 3;
            acc[i] += c.r;
            acc[i + 1] += c.g;
            acc[i + 2] += c.b;
        }
    }
    let rgb = acc.iter().map(|&v| (((v * 0.35).min(1.0) * 255.0) as u8).saturating_add(6)).collect();
    CamFrame { rgb, at: Instant::now() }
}
