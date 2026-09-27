//! Downsampled working image the vectoriser runs on.

use crate::geom::Rgb;

/// Byte order of incoming 8-bit, 4-channel pixels.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PixelOrder {
    Rgba,
    Bgra,
}

/// Small float RGB image. Row-major, top row first.
#[derive(Clone, Debug)]
pub struct WorkImage {
    pub width: usize,
    pub height: usize,
    pub pixels: Vec<Rgb>,
}

impl WorkImage {
    pub fn new(width: usize, height: usize) -> Self {
        Self { width, height, pixels: vec![Rgb::BLACK; width * height] }
    }

    pub fn get(&self, x: usize, y: usize) -> Rgb {
        self.pixels[y * self.width + x]
    }

    /// Brightness used for tracing. Max channel, so saturated blue counts as much as white.
    pub fn value(&self, x: usize, y: usize) -> f32 {
        self.get(x, y).max_channel()
    }

    /// Box-filter an 8-bit 4-channel frame down so neither side exceeds `max_size`.
    /// Alpha is ignored (Resolume sends opaque frames).
    pub fn from_frame(
        data: &[u8],
        width: usize,
        height: usize,
        stride: usize,
        order: PixelOrder,
        max_size: usize,
        flip_y: bool,
    ) -> Self {
        assert!(stride >= width * 4 && data.len() >= stride * height.saturating_sub(1) + width * 4);
        let max_size = max_size.max(1);
        let scale = (width.max(height) as f32 / max_size as f32).max(1.0);
        let ow = ((width as f32 / scale).round() as usize).max(1);
        let oh = ((height as f32 / scale).round() as usize).max(1);

        let mut sums = vec![[0u32; 4]; ow * oh];
        let (ri, bi) = match order {
            PixelOrder::Rgba => (0, 2),
            PixelOrder::Bgra => (2, 0),
        };
        let x_map: Vec<usize> = (0..width).map(|x| (x * ow / width).min(ow - 1)).collect();
        for y in 0..height {
            let oy = (y * oh / height).min(oh - 1);
            let oy = if flip_y { oh - 1 - oy } else { oy };
            let row = &data[y * stride..y * stride + width * 4];
            let out_row = &mut sums[oy * ow..(oy + 1) * ow];
            for (x, px) in row.chunks_exact(4).enumerate() {
                let s = &mut out_row[x_map[x]];
                s[0] += px[ri] as u32;
                s[1] += px[1] as u32;
                s[2] += px[bi] as u32;
                s[3] += 1;
            }
        }
        let pixels = sums
            .iter()
            .map(|s| {
                let n = (s[3].max(1) * 255) as f32;
                Rgb::new(s[0] as f32 / n, s[1] as f32 / n, s[2] as f32 / n)
            })
            .collect();
        Self { width: ow, height: oh, pixels }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn downsample_averages_and_swaps_channels() {
        // 4x2 BGRA: left half pure blue, right half black.
        let mut data = vec![0u8; 4 * 2 * 4];
        for y in 0..2 {
            for x in 0..2 {
                let i = (y * 4 + x) * 4;
                data[i] = 255; // B
            }
        }
        let img = WorkImage::from_frame(&data, 4, 2, 16, PixelOrder::Bgra, 2, false);
        assert_eq!((img.width, img.height), (2, 1));
        assert!((img.get(0, 0).b - 1.0).abs() < 1e-6);
        assert_eq!(img.get(0, 0).r, 0.0);
        assert_eq!(img.get(1, 0), Rgb::BLACK);
    }
}
