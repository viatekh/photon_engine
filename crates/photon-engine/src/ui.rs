//! The control window.

use crate::dac::DacSelection;
use crate::engine::Shared;
use crate::input::{self, SourceSelection};
use crate::settings::Settings;
use egui::{Color32, Pos2, Rect, Sense, Stroke, Vec2 as EVec2};
use photon_core::keystone::{CORNER_NAMES, DEFAULT_CORNERS};
use photon_core::patterns::TestPattern;
use photon_core::planner::{Priority, Strategy};
use photon_core::vectorise::{FitMode, TraceMode};
use photon_core::Vec2;
use std::sync::atomic::Ordering;
use std::sync::Arc;
use std::time::{Duration, Instant};

const SETTINGS_KEY: &str = "photon_settings";

pub struct App {
    shared: Arc<Shared>,
    threads: Vec<std::thread::JoinHandle<()>>,
    sources: Vec<SourceSelection>,
    source_notes: Vec<String>,
    sources_at: Instant,
    preview_tex: Option<egui::TextureHandle>,
    preview_seq: u64,
    show_threshold: bool,
    edit_keystone: bool,
    dragging: Option<usize>,
}

impl App {
    pub fn new(cc: &eframe::CreationContext<'_>) -> Self {
        let settings: Settings = cc
            .storage
            .and_then(|s| eframe::get_value::<Settings>(s, SETTINGS_KEY))
            .map(Settings::migrate)
            .unwrap_or_default();
        let shared = Shared::new(settings);
        let threads = crate::engine::start(shared.clone());
        Self {
            shared,
            threads,
            sources: Vec::new(),
            source_notes: Vec::new(),
            sources_at: Instant::now() - Duration::from_secs(60),
            preview_tex: None,
            preview_seq: 0,
            show_threshold: true,
            edit_keystone: false,
            dragging: None,
        }
    }

    fn disarm(&self) {
        self.shared.armed.store(false, Ordering::SeqCst);
    }
}

impl eframe::App for App {
    fn save(&mut self, storage: &mut dyn eframe::Storage) {
        eframe::set_value(storage, SETTINGS_KEY, &*self.shared.settings.read());
    }

    fn on_exit(&mut self, _gl: Option<&eframe::glow::Context>) {
        self.disarm();
        self.shared.shutdown.store(true, Ordering::SeqCst);
        for t in self.threads.drain(..) {
            let _ = t.join();
        }
    }

    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        let ctx = ui.ctx().clone();
        // Space / Escape: instant blackout, from anywhere in the app.
        if ctx.input(|i| i.key_pressed(egui::Key::Escape) || i.key_pressed(egui::Key::Space)) {
            self.disarm();
        }
        if self.sources_at.elapsed() > Duration::from_secs(2) {
            (self.sources, self.source_notes) = input::available_sources();
            self.sources_at = Instant::now();
        }

        let mut settings = self.shared.settings.read().clone();
        let before = settings.clone();

        self.top_bar(ui);
        egui::Panel::left("controls").resizable(true).default_size(340.0).show(ui, |ui| {
            egui::ScrollArea::vertical().show(ui, |ui| self.controls(ui, &mut settings));
        });
        let auto = settings.auto_detail.enabled;
        egui::Panel::bottom("stats").show(ui, |ui| self.stats(ui, auto));
        egui::CentralPanel::default().show(ui, |ui| self.previews(ui, &mut settings));

        if settings != before {
            *self.shared.settings.write() = settings;
        }
        ctx.request_repaint_after(Duration::from_millis(33));
    }
}

impl App {
    fn top_bar(&mut self, ui: &mut egui::Ui) {
        egui::Panel::top("top").show(ui, |ui| {
            ui.horizontal(|ui| {
                let armed = self.shared.armed.load(Ordering::SeqCst);
                let (text, fill) = if armed {
                    ("● ARMED — click or Space/Esc to blackout", Color32::from_rgb(200, 30, 30))
                } else {
                    ("○ Disarmed — click to ARM laser output", Color32::from_rgb(60, 60, 60))
                };
                let btn = egui::Button::new(egui::RichText::new(text).strong().color(Color32::WHITE))
                    .fill(fill)
                    .min_size(EVec2::new(320.0, 32.0));
                if ui.add(btn).clicked() {
                    self.shared.armed.store(!armed, Ordering::SeqCst);
                }
                ui.separator();
                let rec = self.shared.recording.lock().clone();
                let requested = self.shared.record_requested.load(Ordering::SeqCst);
                let (label, fill) = if rec.active || requested {
                    (
                        format!("■ Stop recording ({:.0}s / {}s)", rec.seconds, crate::recorder::MAX_DURATION.as_secs()),
                        Color32::from_rgb(150, 20, 60),
                    )
                } else {
                    ("● Record".to_string(), Color32::from_rgb(60, 60, 60))
                };
                let rec_btn = egui::Button::new(egui::RichText::new(label).color(Color32::WHITE)).fill(fill);
                if ui
                    .add(rec_btn)
                    .on_hover_text(if rec.message.is_empty() {
                        "Capture up to 20 s of what the engine sees and decides, for analysis.".to_string()
                    } else {
                        rec.message.clone()
                    })
                    .clicked()
                {
                    self.shared.record_requested.store(!(rec.active || requested), Ordering::SeqCst);
                }
                if !rec.active && !rec.message.is_empty() {
                    ui.small(&rec.message);
                }
                ui.separator();
                let out = self.shared.output_status.lock().clone();
                let dot = if out.connected { Color32::GREEN } else { Color32::RED };
                ui.colored_label(dot, "⏺");
                ui.label(&out.message);
                if let Some(r) = out.blanked_reason {
                    ui.separator();
                    ui.colored_label(Color32::YELLOW, format!("Output dark: {r}"));
                }
            });
        });
    }

    fn controls(&mut self, ui: &mut egui::Ui, s: &mut Settings) {
        egui::CollapsingHeader::new("Input").default_open(true).show(ui, |ui| {
            egui::ComboBox::from_label("Source")
                .selected_text(s.source.label())
                .width(220.0)
                .show_ui(ui, |ui| {
                    ui.selectable_value(&mut s.source, SourceSelection::None, "None");
                    for src in &self.sources {
                        ui.selectable_value(&mut s.source, src.clone(), src.label());
                    }
                });
            for n in &self.source_notes {
                ui.small(n);
            }
            let st = self.shared.input_status.lock().clone();
            ui.small(&st.message);
            ui.checkbox(&mut s.flip_input_y, "Flip input vertically");
            ui.separator();
            ui.checkbox(&mut s.test_pattern_on, "Test pattern (overrides input)");
            ui.add_enabled_ui(s.test_pattern_on, |ui| {
                egui::ComboBox::from_label("Pattern")
                    .selected_text(s.test_pattern.label())
                    .show_ui(ui, |ui| {
                        for p in TestPattern::ALL {
                            ui.selectable_value(&mut s.test_pattern, p, p.label());
                        }
                    });
            });
        });

        egui::CollapsingHeader::new("Tracing").default_open(true).show(ui, |ui| {
            let v = &mut s.vectorise;
            egui::ComboBox::from_label("Mode")
                .selected_text(v.mode.label())
                .width(220.0)
                .show_ui(ui, |ui| {
                    for m in TraceMode::ALL {
                        ui.selectable_value(&mut v.mode, m, m.label());
                    }
                });
            ui.small(match v.mode {
                TraceMode::Auto => "Thin lines traced once down the middle; everything else (filled shapes, film, fractals) by its edges.",
                TraceMode::Edges => "Contrast edges only. Thin lines give two edges.",
                TraceMode::Centreline => "Follows the middle of bright lines. Best for laser-style line content.",
                TraceMode::Outline => "Traces the edges of bright areas above the threshold.",
            });
            ui.horizontal(|ui| {
                ui.selectable_value(&mut v.fit, FitMode::Fit, "Fit (keep aspect)");
                ui.selectable_value(&mut v.fit, FitMode::Stretch, "Stretch");
            });
            if v.mode == TraceMode::Auto {
                ui.add(egui::Slider::new(&mut v.stroke_width_px, 1.0..=8.0).text("Max stroke width (px)"))
                    .on_hover_text("Lines up to this wide (in working pixels) are traced once along their centre. Wider areas are traced by their edges.");
                ui.add(egui::Slider::new(&mut v.stroke_threshold, 0.02..=0.8).text("Stroke threshold"));
            }
            if v.mode.uses_edges() {
                ui.add(egui::Slider::new(&mut v.edge_threshold, 0.02..=0.8).text("Edge threshold"));
                ui.add(egui::Slider::new(&mut v.blur_px, 0.0..=4.0).text("Blur (px)"))
                    .on_hover_text("Higher ignores fine texture and keeps bold structure.");
            } else {
                ui.add(egui::Slider::new(&mut v.threshold, 0.02..=0.98).text("Threshold"));
            }
            ui.add(egui::Slider::new(&mut v.temporal_smoothing, 0.0..=0.9).text("Temporal smoothing"))
                .on_hover_text("Blend with previous frames to reduce flicker on video. Too high smears motion.");
            ui.add(egui::Slider::new(&mut v.resolution, 64..=480).text("Resolution (px)"));
            ui.add(egui::Slider::new(&mut v.smoothing, 0..=8).text("Smoothing"));
            ui.add(egui::Slider::new(&mut v.simplify_px, 0.0..=4.0).text("Simplify (px)"));
            ui.add(egui::Slider::new(&mut v.min_length_px, 0.0..=40.0).text("Min length (px)"));
            ui.separator();
            let a = &mut s.auto_detail;
            ui.checkbox(&mut a.enabled, "Auto detail (fit content to scan budget)")
                .on_hover_text("Continuously raises/lowers edge threshold and min length so busy content keeps its strongest structure instead of being culled at random.");
            ui.add_enabled_ui(a.enabled, |ui| {
                ui.add(egui::Slider::new(&mut a.target_fill, 0.3..=1.0).text("Target fill"));
                ui.add(egui::Slider::new(&mut a.speed, 0.05..=1.0).text("Response speed"));
            });
            let v = &mut s.vectorise;
            ui.checkbox(&mut v.normalise_colour, "Full-brightness colours");
        });

        egui::CollapsingHeader::new("Anti-breakup").default_open(true).show(ui, |ui| {
            let p = &mut s.planner;
            egui::ComboBox::from_label("Strategy")
                .selected_text(p.strategy.label())
                .width(220.0)
                .show_ui(ui, |ui| {
                    for st in Strategy::ALL {
                        ui.selectable_value(&mut p.strategy, st, st.label());
                    }
                });
            ui.small(match p.strategy {
                Strategy::WholeShapes => "Draws complete shapes, most important first, until the frame is full at the target rate.",
                Strategy::Simplify => "Simplifies geometry (up to the limit) to fit; then drops whole shapes if still too much.",
                Strategy::AdaptiveRefresh => "Lets the refresh rate fall to the minimum before dropping whole shapes.",
                Strategy::TakeTurns => "Splits shapes into groups drawn on alternate passes. Shows more, flickers more.",
                Strategy::Combined => "Adaptive refresh, then simplify, then drop whole shapes.",
            });
            egui::ComboBox::from_label("Priority")
                .selected_text(format!("{:?}", p.priority))
                .show_ui(ui, |ui| {
                    for pr in Priority::ALL {
                        ui.selectable_value(&mut p.priority, pr, format!("{pr:?}"));
                    }
                });
            ui.add(egui::Slider::new(&mut p.target_hz, 15.0..=120.0).text("Target refresh (Hz)"));
            ui.add(egui::Slider::new(&mut p.min_hz, 10.0..=60.0).text("Min refresh (Hz)"));
            ui.add(egui::Slider::new(&mut p.max_simplify, 0.0..=0.1).text("Max simplify"));
            ui.add(egui::Slider::new(&mut p.max_groups, 1..=6).text("Max groups (D)"));
            ui.add(egui::Slider::new(&mut p.stickiness, 0.0..=2.0).text("Selection stickiness"));
            ui.checkbox(&mut p.split_oversized, "Split very large shapes into strokes")
                .on_hover_text("A connected shape needing over half the frame is split into its separate strokes (each still drawn whole). Off: shapes are only drawn whole, so very large ones may not be drawn at all.");
        });

        egui::CollapsingHeader::new("Output").default_open(true).show(ui, |ui| {
            egui::ComboBox::from_label("Device")
                .selected_text(s.dac.label())
                .width(220.0)
                .show_ui(ui, |ui| {
                    for d in DacSelection::ALL {
                        ui.selectable_value(&mut s.dac, d, d.label());
                    }
                });
            let max_pps = self.shared.output_status.lock().max_pps.max(1000);
            let pps_max = max_pps.max(s.scan.pps);
            ui.add(egui::Slider::new(&mut s.scan.pps, 1000..=pps_max).text("DAC point rate (pps)"))
                .on_hover_text("Points per second sent to the DAC. Sets the per-frame point budget. Limited to what the device reports.");
            ui.add(egui::Slider::new(&mut s.scan.scanner_kpps, 5.0..=60.0).text("Scanner rating (kpps)"))
                .on_hover_text("Your galvos' rated speed at ILDA 8 degrees (e.g. 30 for 30K). Faster scanners can move further per point and need shorter dwells.");
            let c = &mut s.colour;
            ui.add(egui::Slider::new(&mut c.brightness, 0.0..=1.0).text("Brightness"));
            ui.add(egui::Slider::new(&mut c.red, 0.0..=1.0).text("Red"));
            ui.add(egui::Slider::new(&mut c.green, 0.0..=1.0).text("Green"));
            ui.add(egui::Slider::new(&mut c.blue, 0.0..=1.0).text("Blue"));
            ui.add(egui::Slider::new(&mut c.min_level, 0.0..=0.5).text("Min level"));
            ui.add(egui::Slider::new(&mut c.colour_delay, 0..=20).text("Colour delay (pts)"));
            ui.horizontal(|ui| {
                ui.checkbox(&mut s.geometry.flip_x, "Flip X");
                ui.checkbox(&mut s.geometry.flip_y, "Flip Y");
            });
        });

        egui::CollapsingHeader::new("Keystone").default_open(true).show(ui, |ui| {
            ui.checkbox(&mut self.edit_keystone, "Edit corners (drag in the output preview)");
            for (i, name) in CORNER_NAMES.iter().enumerate() {
                ui.horizontal(|ui| {
                    ui.label(format!("{name:>12}"));
                    let c = &mut s.geometry.corners[i];
                    ui.add(egui::DragValue::new(&mut c.x).speed(0.002).range(-1.0..=1.0).prefix("x "));
                    ui.add(egui::DragValue::new(&mut c.y).speed(0.002).range(-1.0..=1.0).prefix("y "));
                });
            }
            if ui.button("Reset keystone").clicked() {
                s.geometry.corners = DEFAULT_CORNERS;
            }
        });

        egui::CollapsingHeader::new("Scanner tuning").default_open(false).show(ui, |ui| {
            let sc = &mut s.scan;
            ui.small("Speeds and dwells are for a 30K scanner; the scanner rating above scales them.");
            ui.add(egui::Slider::new(&mut sc.lit_speed, 50.0..=2000.0).text("Lit speed"));
            ui.add(egui::Slider::new(&mut sc.blank_speed, 100.0..=5000.0).text("Blank speed"));
            ui.add(egui::Slider::new(&mut sc.corner_dwell_us, 0.0..=500.0).text("Corner dwell (µs)"));
            ui.add(egui::Slider::new(&mut sc.corner_min_angle, 0.0..=90.0).text("Corner angle (°)"));
            ui.add(egui::Slider::new(&mut sc.path_dwell_us, 0.0..=500.0).text("Path end dwell (µs)"));
            ui.add(egui::Slider::new(&mut sc.blank_dwell_us, 0.0..=500.0).text("Blank dwell (µs)"));
            ui.separator();
            ui.add(egui::Slider::new(&mut s.signal_timeout_ms, 100..=3000).text("Signal-loss blackout (ms)"));
            ui.add(egui::Slider::new(&mut s.static_beam_min_extent, 0.0..=0.3).text("Static beam guard"));
            if ui.button("Reset scanner tuning").clicked() {
                s.scan = Default::default();
            }
        });

        ui.separator();
        if ui
            .button("Reset all settings")
            .on_hover_text("Back to defaults. Keeps source, device, keystone and flips.")
            .clicked()
        {
            *s = Settings {
                source: s.source.clone(),
                dac: s.dac,
                geometry: s.geometry.clone(),
                ..Default::default()
            };
        }
    }

    fn stats(&self, ui: &mut egui::Ui, s_auto: bool) {
        let plan = self.shared.plan.lock().clone();
        let inp = self.shared.input_status.lock().clone();
        let out = self.shared.output_status.lock().clone();
        ui.horizontal_wrapped(|ui| {
            ui.label(format!("Input {:.0} fps, {:.1} ms/frame", inp.fps, inp.process_ms));
            if s_auto {
                ui.separator();
                ui.label(format!("Detail level {:.2}", inp.detail_level))
                    .on_hover_text("1.0 = your settings. Above 1 = auto detail is reducing detail to fit.");
            }
            if let Some(p) = plan {
                let st = &p.plan.stats;
                ui.separator();
                ui.label(format!("Shapes {} / {} drawn", st.drawn_shapes, st.input_shapes))
                    .on_hover_text(format!("{} / {} paths. A shape is everything that touches; it is drawn whole or not at all.", st.drawn_paths, st.input_paths));
                ui.separator();
                let over = st.points > st.budget;
                ui.colored_label(
                    if over { Color32::RED } else { ui.visuals().text_color() },
                    format!("Points {} / budget {}", st.points, st.budget),
                );
                ui.separator();
                ui.label(format!("Refresh {:.0} Hz", st.refresh_hz));
                if st.groups > 1 {
                    ui.label(format!("({} groups)", st.groups));
                }
                if st.simplify_used > 0.0 {
                    ui.separator();
                    ui.label(format!("Simplified {:.3}", st.simplify_used));
                }
            }
            ui.separator();
            ui.label(format!("Output {:.0} passes/s", out.frames_per_sec));
        });
    }

    fn previews(&mut self, ui: &mut egui::Ui, s: &mut Settings) {
        let avail = ui.available_size();
        let side = (avail.x / 2.0 - 12.0).min(avail.y - 24.0).max(100.0);
        ui.horizontal(|ui| {
            ui.vertical(|ui| {
                ui.horizontal(|ui| {
                    ui.strong("Input");
                    ui.checkbox(&mut self.show_threshold, "show threshold");
                });
                self.input_preview(ui, side, s);
            });
            ui.vertical(|ui| {
                ui.strong("Laser output (what is sent to the DAC; grey = dropped by anti-breakup)");
                self.output_preview(ui, side, s);
            });
        });
    }

    fn input_preview(&mut self, ui: &mut egui::Ui, side: f32, s: &Settings) {
        let preview = self.shared.preview.lock().clone();
        if let Some(p) = &preview {
            if p.seq != self.preview_seq {
                self.preview_seq = p.seq;
                let img = &p.image;
                let thr = if s.vectorise.mode.uses_edges() { 0.0 } else { s.vectorise.threshold };
                let pixels = img
                    .pixels
                    .iter()
                    .map(|c| {
                        let dim = if self.show_threshold && c.max_channel() < thr { 0.25 } else { 1.0 };
                        let f = |v: f32| (v * dim * 255.0).clamp(0.0, 255.0) as u8;
                        Color32::from_rgb(f(c.r), f(c.g), f(c.b))
                    })
                    .collect();
                let ci = egui::ColorImage::new([img.width, img.height], pixels);
                match &mut self.preview_tex {
                    Some(t) => t.set(ci, egui::TextureOptions::NEAREST),
                    None => {
                        self.preview_tex =
                            Some(ui.ctx().load_texture("input", ci, egui::TextureOptions::NEAREST))
                    }
                }
            }
        }
        let (rect, _) = ui.allocate_exact_size(EVec2::splat(side), Sense::hover());
        ui.painter().rect_filled(rect, 0.0, Color32::from_gray(10));
        match (&self.preview_tex, preview) {
            (Some(t), Some(_)) if !s.test_pattern_on => {
                let [w, h] = t.size();
                let scale = (side / w as f32).min(side / h as f32);
                let r = Rect::from_center_size(rect.center(), EVec2::new(w as f32, h as f32) * scale);
                ui.painter().image(t.id(), r, Rect::from_min_max(Pos2::ZERO, Pos2::new(1.0, 1.0)), Color32::WHITE);
            }
            _ => {
                let msg = if s.test_pattern_on { "Test pattern active" } else { "No input" };
                ui.painter().text(rect.center(), egui::Align2::CENTER_CENTER, msg, egui::FontId::proportional(16.0), Color32::GRAY);
            }
        }
    }

    fn output_preview(&mut self, ui: &mut egui::Ui, side: f32, s: &mut Settings) {
        let (rect, resp) = ui.allocate_exact_size(EVec2::splat(side), Sense::click_and_drag());
        let painter = ui.painter_at(rect);
        painter.rect_filled(rect, 0.0, Color32::BLACK);
        painter.rect_stroke(rect, 0.0, Stroke::new(1.0, Color32::from_gray(50)), egui::StrokeKind::Inside);
        // Inset so lines at the very edge of the field (and corner handles) stay visible.
        let half = side / 2.0 - 10.0;
        let to_screen = |p: Vec2| Pos2::new(rect.center().x + p.x * half, rect.center().y - p.y * half);
        let to_laser = |p: Pos2| {
            Vec2::new(
                ((p.x - rect.center().x) / half).clamp(-1.0, 1.0),
                (-(p.y - rect.center().y) / half).clamp(-1.0, 1.0),
            )
        };
        let field = Rect::from_center_size(rect.center(), EVec2::splat(half * 2.0));
        painter.rect_stroke(field, 0.0, Stroke::new(1.0, Color32::from_gray(35)), egui::StrokeKind::Outside);

        if let Some(p) = self.shared.plan.lock().clone() {
            for path in &p.plan.dropped {
                let mut pts: Vec<Pos2> = path.points.iter().map(|&v| to_screen(v)).collect();
                if path.closed {
                    pts.push(pts[0]);
                }
                painter.add(egui::Shape::line(pts, Stroke::new(1.0, Color32::from_gray(70))));
            }
        }

        let monitor = self.shared.monitor.lock().clone();
        for w in monitor.windows(2) {
            let (a, b) = (w[0], w[1]);
            if a.is_lit() && b.is_lit() {
                let c = |v: f32| (v.clamp(0.0, 1.0).powf(0.5) * 255.0) as u8;
                let col = Color32::from_rgb(c(b.r), c(b.g), c(b.b));
                painter.line_segment([to_screen(a.pos()), to_screen(b.pos())], Stroke::new(1.5, col));
            }
        }
        if !self.shared.armed.load(Ordering::SeqCst) {
            // Still show the plan when disarmed so it can be set up safely.
            if let Some(p) = self.shared.plan.lock().clone() {
                for path in &p.plan.drawn {
                    let mut pts: Vec<Pos2> = path.points.iter().map(|&v| to_screen(v)).collect();
                    if path.closed {
                        pts.push(pts[0]);
                    }
                    let c = path.color;
                    let col = Color32::from_rgba_unmultiplied((c.r * 255.0) as u8, (c.g * 255.0) as u8, (c.b * 255.0) as u8, 110);
                    painter.add(egui::Shape::line(pts, Stroke::new(1.0, col)));
                }
            }
        }

        if self.edit_keystone {
            let corners = s.geometry.corners;
            let screen: Vec<Pos2> = corners.iter().map(|&c| to_screen(c)).collect();
            let mut outline = screen.clone();
            outline.push(screen[0]);
            painter.add(egui::Shape::line(outline, Stroke::new(1.0, Color32::from_rgb(255, 200, 0))));
            if resp.drag_started() {
                if let Some(pos) = resp.interact_pointer_pos() {
                    self.dragging = screen
                        .iter()
                        .enumerate()
                        .map(|(i, p)| (i, p.distance(pos)))
                        .filter(|(_, d)| *d < 20.0)
                        .min_by(|a, b| a.1.total_cmp(&b.1))
                        .map(|(i, _)| i);
                }
            }
            if let (Some(i), Some(pos)) = (self.dragging, resp.interact_pointer_pos()) {
                if resp.dragged() {
                    s.geometry.corners[i] = to_laser(pos);
                }
            }
            if resp.drag_stopped() {
                self.dragging = None;
            }
            for (i, p) in screen.iter().enumerate() {
                let active = self.dragging == Some(i);
                painter.circle_filled(*p, if active { 8.0 } else { 6.0 }, Color32::from_rgb(255, 200, 0));
            }
        }
    }
}
