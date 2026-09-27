//! Output geometry: size, flips and four-corner ("hot corner") keystone, plus clipping.

use crate::geom::{Path, Vec2};
use serde::{Deserialize, Serialize};

/// Corner order everywhere: top-left, top-right, bottom-right, bottom-left.
pub const CORNER_NAMES: [&str; 4] = ["Top left", "Top right", "Bottom right", "Bottom left"];
pub const DEFAULT_CORNERS: [Vec2; 4] =
    [Vec2::new(-1.0, 1.0), Vec2::new(1.0, 1.0), Vec2::new(1.0, -1.0), Vec2::new(-1.0, -1.0)];

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct OutputGeometry {
    /// Where the corners of the content square land in laser space.
    pub corners: [Vec2; 4],
    pub flip_x: bool,
    pub flip_y: bool,
}

impl Default for OutputGeometry {
    fn default() -> Self {
        Self { corners: DEFAULT_CORNERS, flip_x: false, flip_y: false }
    }
}

impl OutputGeometry {
    pub fn transform(&self) -> Homography {
        Homography::from_square(self.corners)
    }

    /// Flip, keystone and clip content paths into the laser field.
    pub fn apply(&self, paths: &[Path]) -> Vec<Path> {
        let h = self.transform();
        let mut out = Vec::with_capacity(paths.len());
        for p in paths {
            let pts: Vec<Vec2> = p
                .points
                .iter()
                .map(|&v| {
                    let v = Vec2::new(
                        if self.flip_x { -v.x } else { v.x },
                        if self.flip_y { -v.y } else { v.y },
                    );
                    h.apply(v)
                })
                .collect();
            let mut q = Path::new(pts, p.closed, p.color);
            q.weight = p.weight;
            q.group = p.group;
            q.detail_controlled = p.detail_controlled;
            clip_path(&q, &mut out);
        }
        out
    }
}

/// Projective transform, row-major 3x3 with h[8] = 1.
#[derive(Clone, Copy, Debug)]
pub struct Homography([f32; 9]);

impl Homography {
    pub const IDENTITY: Homography = Homography([1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]);

    /// Maps the square's corners (see `DEFAULT_CORNERS`) to `dst`.
    pub fn from_square(dst: [Vec2; 4]) -> Homography {
        Self::from_points(DEFAULT_CORNERS, dst).unwrap_or(Self::IDENTITY)
    }

    pub fn from_points(src: [Vec2; 4], dst: [Vec2; 4]) -> Option<Homography> {
        // Solve A h = b for the 8 unknowns (standard DLT with h33 = 1).
        let mut a = [[0.0f64; 9]; 8];
        for i in 0..4 {
            let (x, y) = (src[i].x as f64, src[i].y as f64);
            let (u, v) = (dst[i].x as f64, dst[i].y as f64);
            a[2 * i] = [x, y, 1.0, 0.0, 0.0, 0.0, -u * x, -u * y, u];
            a[2 * i + 1] = [0.0, 0.0, 0.0, x, y, 1.0, -v * x, -v * y, v];
        }
        // Gaussian elimination with partial pivoting on the augmented matrix.
        for col in 0..8 {
            let pivot = (col..8).max_by(|&r1, &r2| a[r1][col].abs().total_cmp(&a[r2][col].abs()))?;
            if a[pivot][col].abs() < 1e-12 {
                return None;
            }
            a.swap(col, pivot);
            for row in 0..8 {
                if row != col {
                    let f = a[row][col] / a[col][col];
                    for k in col..9 {
                        a[row][k] -= f * a[col][k];
                    }
                }
            }
        }
        let mut h = [0.0f32; 9];
        for i in 0..8 {
            h[i] = (a[i][8] / a[i][i]) as f32;
        }
        h[8] = 1.0;
        Some(Homography(h))
    }

    pub fn apply(&self, p: Vec2) -> Vec2 {
        let h = &self.0;
        let w = h[6] * p.x + h[7] * p.y + h[8];
        let w = if w.abs() < 1e-9 { 1e-9 } else { w };
        Vec2::new((h[0] * p.x + h[1] * p.y + h[2]) / w, (h[3] * p.x + h[4] * p.y + h[5]) / w)
    }
}

/// Clip a path to the [-1, 1] square, pushing the visible pieces to `out`.
pub fn clip_path(path: &Path, out: &mut Vec<Path>) {
    let inside = |p: Vec2| p.x.abs() <= 1.0 && p.y.abs() <= 1.0;
    if path.points.iter().all(|&p| inside(p)) {
        out.push(path.clone());
        return;
    }
    let mut pts = path.points.clone();
    if path.closed {
        pts.push(pts[0]);
    }
    let mut current: Vec<Vec2> = Vec::new();
    let flush = |current: &mut Vec<Vec2>, out: &mut Vec<Path>| {
        if current.len() >= 2 {
            let mut piece = Path::new(std::mem::take(current), false, path.color);
            piece.weight = path.weight;
            piece.group = path.group;
            piece.detail_controlled = path.detail_controlled;
            out.push(piece);
        }
        current.clear();
    };
    for w in pts.windows(2) {
        match clip_segment(w[0], w[1]) {
            Some((a, b)) => {
                if current.last().is_none_or(|&l| l.distance(a) > 1e-6) {
                    flush(&mut current, out);
                    current.push(a);
                }
                current.push(b);
            }
            None => flush(&mut current, out),
        }
    }
    flush(&mut current, out);
}

/// Liang-Barsky against [-1, 1]^2.
fn clip_segment(a: Vec2, b: Vec2) -> Option<(Vec2, Vec2)> {
    let d = b - a;
    let (mut t0, mut t1) = (0.0f32, 1.0f32);
    for (p, q) in [(-d.x, a.x + 1.0), (d.x, 1.0 - a.x), (-d.y, a.y + 1.0), (d.y, 1.0 - a.y)] {
        if p.abs() < 1e-12 {
            if q < 0.0 {
                return None;
            }
        } else {
            let r = q / p;
            if p < 0.0 {
                t0 = t0.max(r);
            } else {
                t1 = t1.min(r);
            }
            if t0 > t1 {
                return None;
            }
        }
    }
    Some((a + d * t0, a + d * t1))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geom::Rgb;

    fn close(a: Vec2, b: Vec2) -> bool {
        a.distance(b) < 1e-4
    }

    #[test]
    fn identity_corners_are_identity() {
        let h = Homography::from_square(DEFAULT_CORNERS);
        let p = Vec2::new(0.3, -0.7);
        assert!(close(h.apply(p), p));
    }

    #[test]
    fn corners_map_exactly_and_centre_moves() {
        let dst = [
            Vec2::new(-0.5, 0.8),
            Vec2::new(0.5, 0.8),
            Vec2::new(1.0, -1.0),
            Vec2::new(-1.0, -1.0),
        ];
        let h = Homography::from_square(dst);
        for i in 0..4 {
            assert!(close(h.apply(DEFAULT_CORNERS[i]), dst[i]));
        }
        // Trapezoid narrower at top: centre gets pushed up by the perspective.
        let c = h.apply(Vec2::ZERO);
        assert!(c.x.abs() < 1e-4);
    }

    #[test]
    fn clip_splits_path_leaving_field() {
        let p = Path::new(
            vec![Vec2::new(-0.5, 0.0), Vec2::new(1.5, 0.0), Vec2::new(1.5, 0.5), Vec2::new(0.5, 0.5)],
            false,
            Rgb::WHITE,
        );
        let mut out = Vec::new();
        clip_path(&p, &mut out);
        assert_eq!(out.len(), 2);
        assert!(close(out[0].points[1], Vec2::new(1.0, 0.0)));
        assert!(close(out[1].points[0], Vec2::new(1.0, 0.5)));
    }
}
