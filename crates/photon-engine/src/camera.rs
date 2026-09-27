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
    /// Frames per second actually arriving.
    pub fps: f32,
    /// Last lines ffmpeg printed (errors / warnings).
    pub ffmpeg_log: String,
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

/// Capture settings tried in turn until the camera delivers frames. AVFoundation rejects any
/// pixel format / frame rate / size the device doesn't list (ffmpeg's default pixel format,
/// yuv420p, is rejected by most Mac webcams).
const CONFIGS: &[(&str, &str, Option<&str>)] = &[
    ("uyvy422", "30", None),
    ("nv12", "30", None),
    ("uyvy422", "30", Some("1280x720")),
    ("nv12", "30", Some("1280x720")),
    ("0rgb", "30", None),
    ("bgr0", "30", None),
    ("uyvy422", "15", Some("640x480")),
    ("nv12", "15", Some("640x480")),
];

fn run(shared: Arc<Shared>) {
    let mut current = CameraSelection::None;
    let mut child: Option<Child> = None;
    let mut reader: Option<thread::JoinHandle<()>> = None;
    let mut attempt = 0usize;
    let mut frames_at_spawn = 0u64;
    let sim_warp = Homography::from_points(
        [Vec2::new(-1.0, 1.0), Vec2::new(1.0, 1.0), Vec2::new(1.0, -1.0), Vec2::new(-1.0, -1.0)],
        // A slightly off-axis camera view, in camera pixels.
        [Vec2::new(250.0, 40.0), Vec2::new(735.0, 70.0), Vec2::new(720.0, 500.0), Vec2::new(230.0, 480.0)],
    )
    .unwrap();

    while !shared.shutdown.load(Ordering::Relaxed) {
        let wanted = shared.settings.read().camera.clone();
        let mut respawn = false;
        if wanted != current {
            stop(&mut child, &mut reader);
            {
                let mut st = shared.camera.lock();
                st.latest = None;
                st.fps = 0.0;
            }
            current = wanted.clone();
            attempt = 0;
            respawn = matches!(current, CameraSelection::Device { .. });
            if current == CameraSelection::None {
                set_msg(&shared.camera, "");
                shared.camera.lock().ffmpeg_log.clear();
            }
        }
        if let (CameraSelection::Device { .. }, Some(c)) = (&current, child.as_mut()) {
            if let Ok(Some(status)) = c.try_wait() {
                stop(&mut child, &mut reader);
                let got_frames = shared.camera.lock().frames > frames_at_spawn;
                if !got_frames && attempt + 1 < CONFIGS.len() {
                    attempt += 1;
                    respawn = true;
                } else {
                    set_msg(
                        &shared.camera,
                        format!("ffmpeg stopped ({status}); check the camera and its permission (see messages below)"),
                    );
                }
            }
        }
        if respawn {
            if let CameraSelection::Device { index, .. } = &current {
                let (pix, fps, size) = CONFIGS[attempt];
                shared.camera.lock().ffmpeg_log.clear();
                frames_at_spawn = shared.camera.lock().frames;
                match spawn_ffmpeg(*index, pix, fps, size) {
                    Ok((mut c, r)) => {
                        if let Some(err) = c.stderr.take() {
                            let sh = shared.clone();
                            thread::spawn(move || read_log(sh, err));
                        }
                        child = Some(c);
                        let sh = shared.clone();
                        reader = Some(thread::spawn(move || read_frames(sh, r)));
                        set_msg(
                            &shared.camera,
                            format!(
                                "Capturing {} ({pix}, {fps} fps, {})",
                                current.label(),
                                size.unwrap_or("default size")
                            ),
                        );
                    }
                    Err(e) => set_msg(&shared.camera, format!("{e:#}")),
                }
            }
        }
        match &current {
            CameraSelection::Simulated => {
                let frame = simulated_frame(&shared, &sim_warp);
                let mut st = shared.camera.lock();
                st.latest = Some(Arc::new(frame));
                st.frames += 1;
                st.fps = 30.0;
                st.message = "Simulated camera".into();
                drop(st);
                thread::sleep(Duration::from_millis(33));
            }
            _ => thread::sleep(Duration::from_millis(100)),
        }
    }
    stop(&mut child, &mut reader);
}

fn stop(child: &mut Option<Child>, reader: &mut Option<thread::JoinHandle<()>>) {
    if let Some(mut c) = child.take() {
        let _ = c.kill();
        let _ = c.wait();
    }
    if let Some(r) = reader.take() {
        let _ = r.join();
    }
}

fn spawn_ffmpeg(
    index: usize,
    pix: &str,
    fps: &str,
    size: Option<&str>,
) -> anyhow::Result<(Child, std::process::ChildStdout)> {
    // Input options must match a mode the device lists; output is scaled to CAM_W x CAM_H RGB.
    let mut args: Vec<String> = ["-hide_banner", "-loglevel", "error", "-f", "avfoundation"]
        .iter()
        .map(|s| s.to_string())
        .collect();
    args.extend(["-pixel_format".into(), pix.into(), "-framerate".into(), fps.into()]);
    if let Some(size) = size {
        args.extend(["-video_size".into(), size.into()]);
    }
    args.extend([
        "-i".into(),
        format!("{index}:none"),
        "-vf".into(),
        format!("scale={CAM_W}:{CAM_H}"),
        "-pix_fmt".into(),
        "rgb24".into(),
        "-f".into(),
        "rawvideo".into(),
        "-".into(),
    ]);
    log::info!("camera: ffmpeg {}", args.join(" "));
    let mut child = Command::new("ffmpeg")
        .args(&args)
        .stdin(Stdio::null())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()
        .map_err(|e| anyhow::anyhow!("could not start ffmpeg ({e}); install with: brew install ffmpeg"))?;
    let out = child.stdout.take().unwrap();
    Ok((child, out))
}

/// Forward ffmpeg's messages to the log and the camera panel.
fn read_log(shared: Arc<Shared>, err: std::process::ChildStderr) {
    use std::io::BufRead;
    for line in std::io::BufReader::new(err).lines().map_while(Result::ok) {
        log::warn!("ffmpeg: {line}");
        // Keep the last few lines: ffmpeg's errors span several (e.g. the supported-mode list).
        let mut st = shared.camera.lock();
        let mut lines: Vec<&str> = st.ffmpeg_log.lines().collect();
        lines.push(line.trim());
        let keep = lines.len().saturating_sub(12);
        st.ffmpeg_log = lines[keep..].join("\n");
    }
}

fn read_frames(shared: Arc<Shared>, mut out: std::process::ChildStdout) {
    let size = CAM_W * CAM_H * 3;
    let mut since = Instant::now();
    let mut count = 0u32;
    loop {
        let mut buf = vec![0u8; size];
        if out.read_exact(&mut buf).is_err() {
            log::warn!("camera: ffmpeg stream ended");
            shared.camera.lock().fps = 0.0;
            return;
        }
        count += 1;
        let mut st = shared.camera.lock();
        st.latest = Some(Arc::new(CamFrame { rgb: buf, at: Instant::now() }));
        st.frames += 1;
        if since.elapsed() >= Duration::from_secs(1) {
            st.fps = count as f32 / since.elapsed().as_secs_f32();
            count = 0;
            since = Instant::now();
        }
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
