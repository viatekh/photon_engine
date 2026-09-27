//! The two worker threads:
//! * **pipeline**: source frame -> vectorise -> keystone -> plan (anti-breakup)
//! * **output**: streams the latest plan to the DAC, applying colour and safety checks

use crate::dac::{Dac, DacSelection};
use crate::input::{SourceSelection, VideoSource};
use crate::recorder::{Recorder, RecordingStatus, MAX_DURATION};
use crate::settings::Settings;
use parking_lot::{Mutex, RwLock};
use photon_core::detail::AutoDetail;
use photon_core::image::WorkImage;
use photon_core::output::{blank_frame, is_static_beam};
use photon_core::planner::{Plan, Planner};
use photon_core::scan::emit_blank;
use photon_core::vectorise::vectorise;
use photon_core::{LaserPoint, Vec2};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::Arc;
use std::thread;
use std::time::{Duration, Instant};

pub struct PlanPacket {
    pub plan: Plan,
    pub created: Instant,
    pub seq: u64,
}

pub struct Preview {
    pub image: WorkImage,
    pub seq: u64,
}

#[derive(Clone, Default)]
pub struct InputStatus {
    pub message: String,
    pub fps: f32,
    pub process_ms: f32,
    /// Auto-detail level (1.0 = settings as set; higher = detail being reduced).
    pub detail_level: f32,
}

#[derive(Clone, Default)]
pub struct OutputStatus {
    pub message: String,
    pub connected: bool,
    pub max_pps: u32,
    pub blanked_reason: Option<&'static str>,
    pub frames_per_sec: f32,
}

pub struct Shared {
    pub settings: RwLock<Settings>,
    pub plan: Mutex<Option<Arc<PlanPacket>>>,
    pub preview: Mutex<Option<Arc<Preview>>>,
    /// The frame currently being sent to the DAC (after colour + safety).
    pub monitor: Mutex<Arc<Vec<LaserPoint>>>,
    pub input_status: Mutex<InputStatus>,
    pub output_status: Mutex<OutputStatus>,
    /// Set by the UI to start / stop recording; the pipeline thread does the work.
    pub record_requested: AtomicBool,
    pub recording: Mutex<RecordingStatus>,
    /// Never persisted: the app always starts disarmed.
    pub armed: AtomicBool,
    pub shutdown: AtomicBool,
    seq: AtomicU64,
}

impl Shared {
    pub fn new(settings: Settings) -> Arc<Self> {
        Arc::new(Self {
            settings: RwLock::new(settings),
            plan: Mutex::new(None),
            preview: Mutex::new(None),
            monitor: Mutex::new(Arc::new(Vec::new())),
            input_status: Mutex::new(InputStatus::default()),
            output_status: Mutex::new(OutputStatus::default()),
            record_requested: AtomicBool::new(false),
            recording: Mutex::new(RecordingStatus::default()),
            armed: AtomicBool::new(false),
            shutdown: AtomicBool::new(false),
            seq: AtomicU64::new(0),
        })
    }

    fn next_seq(&self) -> u64 {
        self.seq.fetch_add(1, Ordering::Relaxed) + 1
    }
}

pub fn start(shared: Arc<Shared>) -> Vec<thread::JoinHandle<()>> {
    let s1 = shared.clone();
    let s2 = shared;
    vec![
        thread::Builder::new().name("pipeline".into()).spawn(move || pipeline(s1)).unwrap(),
        thread::Builder::new().name("output".into()).spawn(move || output(s2)).unwrap(),
    ]
}

// ------------------------------------------------------------------------------------------------

fn pipeline(shared: Arc<Shared>) {
    let mut planner = Planner::new();
    let mut detail = AutoDetail::default();
    let mut previous: Option<WorkImage> = None;
    let mut source: Option<Box<dyn VideoSource>> = None;
    let mut source_sel = SourceSelection::None;
    let mut retry_at = Instant::now();
    let mut frames = 0u32;
    let mut fps_since = Instant::now();
    let mut recorder: Option<Recorder> = None;

    while !shared.shutdown.load(Ordering::Relaxed) {
        let settings = shared.settings.read().clone();
        update_recorder(&shared, &mut recorder);

        // (Re)connect the source when the selection changes or after an error.
        if settings.source != source_sel {
            source = None;
            previous = None;
            detail = AutoDetail::default();
            source_sel = settings.source.clone();
            retry_at = Instant::now();
        }
        if source.is_none() && source_sel != SourceSelection::None && Instant::now() >= retry_at {
            match source_sel.open() {
                Ok(s) => {
                    source = s;
                    set_input_msg(&shared, format!("Connected to {}", source_sel.label()));
                }
                Err(e) => {
                    set_input_msg(&shared, format!("{}: {e:#}", source_sel.label()));
                    retry_at = Instant::now() + Duration::from_secs(2);
                }
            }
        }

        if settings.test_pattern_on {
            let paths = settings.geometry.apply(&settings.test_pattern.paths());
            let plan = planner.plan(paths, &settings.scan, &settings.planner);
            record(&shared, &mut recorder, &settings, None, &plan, 1.0, 0.0);
            publish(&shared, plan);
            thread::sleep(Duration::from_millis(30));
            continue;
        }

        let Some(src) = source.as_mut() else {
            if source_sel == SourceSelection::None {
                set_input_msg(&shared, "No source selected".into());
            }
            thread::sleep(Duration::from_millis(50));
            continue;
        };

        let mut result = None;
        let received = src.receive(Duration::from_millis(100), &mut |f| {
            let t0 = Instant::now();
            let mut image = WorkImage::from_frame(
                f.data,
                f.width,
                f.height,
                f.stride,
                f.order,
                settings.vectorise.resolution,
                settings.flip_input_y,
            );
            // Temporal smoothing: blend with the previous frame to steady edges on video.
            let a = settings.vectorise.temporal_smoothing.clamp(0.0, 0.95);
            if let Some(prev) = &previous {
                if a > 0.0 && prev.width == image.width && prev.height == image.height {
                    for (c, p) in image.pixels.iter_mut().zip(&prev.pixels) {
                        c.r = c.r * (1.0 - a) + p.r * a;
                        c.g = c.g * (1.0 - a) + p.g * a;
                        c.b = c.b * (1.0 - a) + p.b * a;
                    }
                }
            }
            let level_used = detail.level;
            let vp = detail.apply(&settings.vectorise, &settings.auto_detail);
            let paths = vectorise(&image, &vp);
            let paths = settings.geometry.apply(&paths);
            let plan = planner.plan(paths, &settings.scan, &settings.planner);
            detail.update(&plan.stats, &settings.auto_detail);
            previous = Some(image.clone());
            result = Some((image, plan, t0.elapsed(), level_used));
        });
        match received {
            Err(e) => {
                set_input_msg(&shared, format!("{}: {e:#}", source_sel.label()));
                source = None;
                retry_at = Instant::now() + Duration::from_secs(1);
            }
            Ok(_) => {
                if let Some((image, plan, took, level_used)) = result {
                    let ms = took.as_secs_f32() * 1000.0;
                    record(&shared, &mut recorder, &settings, Some(&image), &plan, level_used, ms);
                    publish(&shared, plan);
                    let seq = shared.next_seq();
                    *shared.preview.lock() = Some(Arc::new(Preview { image, seq }));
                    frames += 1;
                    let el = fps_since.elapsed().as_secs_f32();
                    let mut st = shared.input_status.lock();
                    st.process_ms = st.process_ms * 0.9 + took.as_secs_f32() * 1000.0 * 0.1;
                    st.detail_level = detail.level;
                    if el >= 1.0 {
                        st.fps = frames as f32 / el;
                        frames = 0;
                        fps_since = Instant::now();
                    }
                }
            }
        }
    }
}

fn set_input_msg(shared: &Shared, msg: String) {
    let mut st = shared.input_status.lock();
    if st.message != msg {
        log::info!("input: {msg}");
        st.message = msg;
    }
}

/// Start/stop the recorder to match the UI's request, and enforce the length limit.
fn update_recorder(shared: &Shared, recorder: &mut Option<Recorder>) {
    let want = shared.record_requested.load(Ordering::Relaxed);
    let over = recorder.as_ref().is_some_and(|r| r.elapsed() >= MAX_DURATION);
    if want && recorder.is_none() {
        match Recorder::start() {
            Ok(r) => *recorder = Some(r),
            Err(e) => {
                shared.record_requested.store(false, Ordering::Relaxed);
                shared.recording.lock().message = format!("Could not start recording: {e:#}");
            }
        }
    } else if (!want || over) && recorder.is_some() {
        shared.record_requested.store(false, Ordering::Relaxed);
        let r = recorder.take().unwrap();
        let frames = r.frames();
        let msg = match r.finish() {
            Ok(p) => {
                let size = std::fs::metadata(&p).map(|m| m.len() as f32 / 1e6).unwrap_or(0.0);
                let full = std::fs::canonicalize(&p).unwrap_or(p);
                format!("Saved {frames} frames, {size:.1} MB: {}", full.display())
            }
            Err(e) => format!("Recording failed: {e:#}"),
        };
        log::info!("{msg}");
        let mut st = shared.recording.lock();
        st.active = false;
        st.message = msg;
    }
    if let Some(r) = recorder {
        let mut st = shared.recording.lock();
        st.active = true;
        st.seconds = r.elapsed().as_secs_f32();
        st.frames = r.frames();
        st.megabytes = r.raw_megabytes();
    }
}

fn record(
    shared: &Shared,
    recorder: &mut Option<Recorder>,
    settings: &Settings,
    image: Option<&WorkImage>,
    plan: &Plan,
    level: f32,
    ms: f32,
) {
    let Some(r) = recorder.as_mut() else { return };
    let armed = shared.armed.load(Ordering::Relaxed);
    let output = shared.output_status.lock().clone();
    if let Err(e) = r.record(settings, image, plan, level, ms, armed, &output) {
        log::error!("recording error: {e:#}");
        shared.record_requested.store(false, Ordering::Relaxed);
    }
}

fn publish(shared: &Shared, plan: Plan) {
    let seq = shared.next_seq();
    *shared.plan.lock() = Some(Arc::new(PlanPacket { plan, created: Instant::now(), seq }));
}

// ------------------------------------------------------------------------------------------------

/// Plays plan frames back-to-back, switching to a new plan only at a frame boundary.
struct Player {
    frame: Vec<LaserPoint>,
    idx: usize,
    packet: Option<Arc<PlanPacket>>,
    group: usize,
    pos: Vec2,
    frames_played: u32,
    blanked: Option<&'static str>,
}

impl Player {
    fn new() -> Self {
        Self {
            frame: Vec::new(),
            idx: 0,
            packet: None,
            group: 0,
            pos: Vec2::ZERO,
            frames_played: 0,
            blanked: Some("starting"),
        }
    }

    fn fill(&mut self, shared: &Shared, settings: &Settings, n: usize, out: &mut Vec<LaserPoint>) {
        // Disarming cuts the current frame short instead of waiting for it to finish.
        if !shared.armed.load(Ordering::Relaxed) && self.blanked != Some("disarmed") {
            self.idx = self.frame.len();
        }
        while out.len() < n {
            if self.idx >= self.frame.len() {
                self.next_frame(shared, settings);
            }
            let take = (n - out.len()).min(self.frame.len() - self.idx);
            out.extend_from_slice(&self.frame[self.idx..self.idx + take]);
            self.idx += take;
        }
        if let Some(p) = out.last() {
            self.pos = p.pos();
        }
    }

    fn next_frame(&mut self, shared: &Shared, settings: &Settings) {
        self.frames_played += 1;
        let latest = shared.plan.lock().clone();
        let switched = match (&latest, &self.packet) {
            (Some(a), Some(b)) => a.seq != b.seq,
            (Some(_), None) => true,
            _ => false,
        };
        if switched {
            self.packet = latest;
            self.group = 0;
        } else if let Some(p) = &self.packet {
            self.group = (self.group + 1) % p.plan.frames.len().max(1);
        }

        let stale = self
            .packet
            .as_ref()
            .is_none_or(|p| p.created.elapsed() > Duration::from_millis(settings.signal_timeout_ms as u64));
        let points: &[LaserPoint] = self
            .packet
            .as_ref()
            .and_then(|p| p.plan.frames.get(self.group))
            .map(|f| f.points.as_slice())
            .unwrap_or(&[]);

        let reason = if !shared.armed.load(Ordering::Relaxed) {
            Some("disarmed")
        } else if stale {
            Some("no signal")
        } else if points.is_empty() {
            Some("nothing to draw")
        } else {
            None
        };

        let mut frame = match reason {
            Some(_) => Vec::new(),
            None => {
                let f = settings.colour.apply(points);
                if is_static_beam(&f, settings.static_beam_min_extent) {
                    self.blanked = Some("static beam blocked");
                    blank_frame(&f)
                } else {
                    f
                }
            }
        };
        if reason.is_some() {
            self.blanked = reason;
        } else if !frame.iter().all(|p| !p.is_lit()) {
            self.blanked = None;
        }
        if frame.is_empty() {
            // Park the beam, dark, for ~5 ms.
            let hold = (settings.scan.pps as usize / 200).max(16);
            frame = vec![LaserPoint::blank(self.pos); hold];
        }

        // Blanked jump from wherever the beam is to where the new frame starts.
        let start = frame[0].pos();
        let mut transit = Vec::new();
        emit_blank(self.pos, start, &settings.scan, &mut transit);
        if !transit.is_empty() {
            transit.extend(frame);
            frame = transit;
        }
        *shared.monitor.lock() = Arc::new(frame.clone());
        self.frame = frame;
        self.idx = 0;
    }
}

fn output(shared: Arc<Shared>) {
    let mut dac: Option<Box<dyn Dac>> = None;
    let mut dac_sel: Option<DacSelection> = None;
    let mut retry_at = Instant::now();
    let mut player = Player::new();
    let mut applied_pps = 0u32;
    let mut enabled: Option<bool> = None;
    let mut buf = Vec::new();
    let mut stats_since = Instant::now();

    while !shared.shutdown.load(Ordering::Relaxed) {
        let settings = shared.settings.read().clone();

        if dac_sel != Some(settings.dac) {
            dac = None;
            dac_sel = Some(settings.dac);
            retry_at = Instant::now();
        }
        if dac.is_none() {
            if Instant::now() < retry_at {
                thread::sleep(Duration::from_millis(50));
                continue;
            }
            match settings.dac.open() {
                Ok(d) => {
                    let mut st = shared.output_status.lock();
                    st.message = d.description();
                    st.connected = true;
                    st.max_pps = d.max_pps();
                    log::info!("output: {}", st.message);
                    dac = Some(d);
                    applied_pps = 0;
                    enabled = None;
                }
                Err(e) => {
                    let mut st = shared.output_status.lock();
                    st.message = format!("{e:#}");
                    st.connected = false;
                    retry_at = Instant::now() + Duration::from_secs(2);
                    continue;
                }
            }
        }
        let d = dac.as_mut().unwrap();

        let result = (|| -> anyhow::Result<()> {
            if applied_pps != settings.scan.pps {
                d.set_pps(settings.scan.pps)?;
                applied_pps = settings.scan.pps;
            }
            let want = shared.armed.load(Ordering::Relaxed);
            if enabled != Some(want) {
                d.set_enabled(want)?;
                enabled = Some(want);
            }
            let free = d.free_space()?;
            // Small chunks keep latency (and the time to go dark) low.
            let chunk = (settings.scan.pps as usize / 100).clamp(64, 1000);
            if free < chunk.min(256) {
                thread::sleep(Duration::from_millis(1));
                return Ok(());
            }
            buf.clear();
            player.fill(&shared, &settings, free.min(chunk), &mut buf);
            d.write(&buf)?;
            Ok(())
        })();

        if let Err(e) = result {
            log::error!("output error: {e:#}");
            let mut st = shared.output_status.lock();
            st.message = format!("{e:#}");
            st.connected = false;
            dac = None;
            retry_at = Instant::now() + Duration::from_secs(1);
            continue;
        }

        let el = stats_since.elapsed();
        if el >= Duration::from_millis(250) {
            let mut st = shared.output_status.lock();
            st.frames_per_sec = st.frames_per_sec * 0.8 + 0.2 * player.frames_played as f32 / el.as_secs_f32();
            st.blanked_reason = player.blanked;
            player.frames_played = 0;
            stats_since = Instant::now();
        }
    }

    // Shutting down: go dark.
    if let Some(d) = dac.as_mut() {
        let _ = d.write(&vec![LaserPoint::blank(player.pos); 256]);
        let _ = d.set_enabled(false);
    }
}
