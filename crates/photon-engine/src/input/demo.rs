//! Built-in animated source: a varying number of rings and lines, to exercise tracing and
//! anti-breakup without Resolume running.

use super::{FrameRef, VideoSource};
use photon_core::image::PixelOrder;
use std::time::{Duration, Instant};

const W: usize = 960;
const H: usize = 540;

#[derive(Clone, Copy)]
pub enum DemoKind {
    Rings,
    /// Continuous Mandelbrot zoom: endlessly intricate detail.
    Fractal,
}

pub struct DemoSource {
    kind: DemoKind,
    start: Instant,
    next: Instant,
    buf: Vec<u8>,
}

impl DemoSource {
    pub fn new(kind: DemoKind) -> Self {
        Self { kind, start: Instant::now(), next: Instant::now(), buf: vec![0; W * H * 4] }
    }

    fn draw_fractal(&mut self, t: f32) {
        // Zoom in for 30 s then start again. Rendered at half resolution to keep it cheap.
        let zoom = 1.25f64.powf((t % 30.0) as f64);
        let (cx, cy) = (-0.743_643_887_037_151f64, 0.131_825_904_205_33f64);
        let (hw, hh) = (W / 2, H / 2);
        let scale = 3.0 / (hw as f64 * zoom);
        let max_iter = 120 + (zoom.log2() * 12.0) as u32;
        for py in 0..hh {
            for px in 0..hw {
                let x0 = cx + (px as f64 - hw as f64 / 2.0) * scale;
                let y0 = cy + (py as f64 - hh as f64 / 2.0) * scale;
                let (mut x, mut y, mut i) = (0.0f64, 0.0f64, 0u32);
                while x * x + y * y < 4.0 && i < max_iter {
                    let tmp = x * x - y * y + x0;
                    y = 2.0 * x * y + y0;
                    x = tmp;
                    i += 1;
                }
                let c = if i == max_iter {
                    [0, 0, 0]
                } else {
                    let f = i as f32 * 0.1 + t * 0.5;
                    let ch = |o: f32| ((0.5 + 0.5 * (f + o).sin()) * 255.0) as u8;
                    [ch(0.0), ch(2.1), ch(4.2)]
                };
                for (dy, dx) in [(0, 0), (0, 1), (1, 0), (1, 1)] {
                    let k = ((py * 2 + dy) * W + px * 2 + dx) * 4;
                    self.buf[k..k + 4].copy_from_slice(&[c[0], c[1], c[2], 255]);
                }
            }
        }
    }

    fn draw(&mut self, t: f32) {
        if let DemoKind::Fractal = self.kind {
            return self.draw_fractal(t);
        }
        self.buf.fill(0);
        let buf = &mut self.buf;
        let mut plot = |x: f32, y: f32, c: [u8; 3]| {
            let (x, y) = (x as i32, y as i32);
            for dy in -1..=1 {
                for dx in -1..=1 {
                    let (px, py) = (x + dx, y + dy);
                    if px >= 0 && py >= 0 && (px as usize) < W && (py as usize) < H {
                        let i = (py as usize * W + px as usize) * 4;
                        buf[i..i + 3].copy_from_slice(&c);
                        buf[i + 3] = 255;
                    }
                }
            }
        };
        // Ring count swells from 1 to 24 and back every 20 s, so culling kicks in and out.
        let n = 1 + ((1.0 - (t * std::f32::consts::TAU / 20.0).cos()) * 11.5) as usize;
        for k in 0..n {
            let a0 = k as f32 * 2.399 + t * 0.3;
            let rad = 40.0 + 180.0 * ((k as f32 * 0.37).fract());
            let cx = W as f32 / 2.0 + rad * a0.cos();
            let cy = H as f32 / 2.0 + rad * 0.8 * a0.sin();
            let r = 18.0 + 22.0 * (0.5 + 0.5 * (t * 1.3 + k as f32).sin());
            let hue = [[0, 255, 60], [0, 120, 255], [255, 40, 40], [255, 0, 200]][k % 4];
            let steps = (r * 8.0) as usize;
            for s in 0..steps {
                let a = s as f32 / steps as f32 * std::f32::consts::TAU;
                plot(cx + r * a.cos(), cy + r * a.sin(), hue);
            }
        }
        // A sweeping line.
        let y = H as f32 * (0.5 + 0.4 * (t * 0.7).sin());
        for x in 60..W - 60 {
            plot(x as f32, y, [255, 255, 255]);
        }
    }
}

impl VideoSource for DemoSource {
    fn receive(&mut self, timeout: Duration, f: &mut dyn FnMut(FrameRef)) -> anyhow::Result<bool> {
        let now = Instant::now();
        if self.next > now {
            let wait = self.next - now;
            if wait > timeout {
                std::thread::sleep(timeout);
                return Ok(false);
            }
            std::thread::sleep(wait);
        }
        self.next = Instant::now() + Duration::from_millis(33);
        let t = self.start.elapsed().as_secs_f32();
        self.draw(t);
        f(FrameRef { data: &self.buf, width: W, height: H, stride: W * 4, order: PixelOrder::Rgba });
        Ok(true)
    }
}
