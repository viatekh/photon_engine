//! Built-in animated source: a varying number of rings and lines, to exercise tracing and
//! anti-breakup without Resolume running.

use super::{FrameRef, VideoSource};
use photon_core::image::PixelOrder;
use std::time::{Duration, Instant};

const W: usize = 960;
const H: usize = 540;

pub struct DemoSource {
    start: Instant,
    next: Instant,
    buf: Vec<u8>,
}

impl DemoSource {
    pub fn new() -> Self {
        Self { start: Instant::now(), next: Instant::now(), buf: vec![0; W * H * 4] }
    }

    fn draw(&mut self, t: f32) {
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
