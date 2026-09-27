//! Basic geometry types.
//!
//! Laser space is normalised: x and y run from -1.0 to 1.0, with +y pointing up.
//! (0, 0) is the centre of the projection.

use serde::{Deserialize, Serialize};
use std::ops::{Add, Mul, Sub};

#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Vec2 {
    pub x: f32,
    pub y: f32,
}

impl Vec2 {
    pub const ZERO: Vec2 = Vec2 { x: 0.0, y: 0.0 };

    pub const fn new(x: f32, y: f32) -> Self {
        Self { x, y }
    }

    pub fn length(self) -> f32 {
        (self.x * self.x + self.y * self.y).sqrt()
    }

    pub fn distance(self, other: Vec2) -> f32 {
        (self - other).length()
    }

    pub fn dot(self, other: Vec2) -> f32 {
        self.x * other.x + self.y * other.y
    }

    pub fn lerp(self, other: Vec2, t: f32) -> Vec2 {
        self + (other - self) * t
    }
}

impl Add for Vec2 {
    type Output = Vec2;
    fn add(self, o: Vec2) -> Vec2 {
        Vec2::new(self.x + o.x, self.y + o.y)
    }
}

impl Sub for Vec2 {
    type Output = Vec2;
    fn sub(self, o: Vec2) -> Vec2 {
        Vec2::new(self.x - o.x, self.y - o.y)
    }
}

impl Mul<f32> for Vec2 {
    type Output = Vec2;
    fn mul(self, s: f32) -> Vec2 {
        Vec2::new(self.x * s, self.y * s)
    }
}

/// Linear RGB, each channel 0.0..=1.0.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Rgb {
    pub r: f32,
    pub g: f32,
    pub b: f32,
}

impl Rgb {
    pub const BLACK: Rgb = Rgb { r: 0.0, g: 0.0, b: 0.0 };
    pub const WHITE: Rgb = Rgb { r: 1.0, g: 1.0, b: 1.0 };

    pub const fn new(r: f32, g: f32, b: f32) -> Self {
        Self { r, g, b }
    }

    pub fn max_channel(self) -> f32 {
        self.r.max(self.g).max(self.b)
    }

    pub fn is_black(self) -> bool {
        self.max_channel() <= 0.0
    }

    pub fn scale(self, s: f32) -> Rgb {
        Rgb::new(self.r * s, self.g * s, self.b * s)
    }
}

/// A polyline the laser should draw in a single colour, without blanking.
#[derive(Clone, Debug, PartialEq)]
pub struct Path {
    pub points: Vec<Vec2>,
    /// A closed path returns to its first point.
    pub closed: bool,
    pub color: Rgb,
    /// Importance of the path per unit length (edge strength or brightness), used for culling.
    pub weight: f32,
    /// Paths with the same group (other than NO_GROUP) are one shape: drawn together or not at all.
    pub group: u32,
    /// Whether auto detail can reduce this path (edge / threshold traces, not strokes or patterns).
    pub detail_controlled: bool,
}

/// A path that is a shape on its own.
pub const NO_GROUP: u32 = 0;

impl Path {
    pub fn new(points: Vec<Vec2>, closed: bool, color: Rgb) -> Self {
        Self { points, closed, color, weight: 1.0, group: NO_GROUP, detail_controlled: false }
    }

    /// Drawn length, including the closing segment of a closed path.
    pub fn length(&self) -> f32 {
        let mut len: f32 = self.points.windows(2).map(|w| w[0].distance(w[1])).sum();
        if self.closed && self.points.len() > 2 {
            len += self.points[self.points.len() - 1].distance(self.points[0]);
        }
        len
    }

    pub fn centroid(&self) -> Vec2 {
        if self.points.is_empty() {
            return Vec2::ZERO;
        }
        let sum = self.points.iter().fold(Vec2::ZERO, |a, &p| a + p);
        sum * (1.0 / self.points.len() as f32)
    }

    /// Axis-aligned bounds as (min, max).
    pub fn bounds(&self) -> (Vec2, Vec2) {
        let mut min = Vec2::new(f32::MAX, f32::MAX);
        let mut max = Vec2::new(f32::MIN, f32::MIN);
        for p in &self.points {
            min.x = min.x.min(p.x);
            min.y = min.y.min(p.y);
            max.x = max.x.max(p.x);
            max.y = max.y.max(p.y);
        }
        (min, max)
    }

    pub fn start(&self) -> Vec2 {
        self.points[0]
    }

    /// Where the beam ends up after drawing the path.
    pub fn end(&self) -> Vec2 {
        if self.closed {
            self.points[0]
        } else {
            self.points[self.points.len() - 1]
        }
    }
}

/// A single sample sent to the DAC. Position in laser space, colour 0..1.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct LaserPoint {
    pub x: f32,
    pub y: f32,
    pub r: f32,
    pub g: f32,
    pub b: f32,
}

impl LaserPoint {
    pub fn lit(p: Vec2, c: Rgb) -> Self {
        Self { x: p.x, y: p.y, r: c.r, g: c.g, b: c.b }
    }

    pub fn blank(p: Vec2) -> Self {
        Self { x: p.x, y: p.y, r: 0.0, g: 0.0, b: 0.0 }
    }

    pub fn pos(&self) -> Vec2 {
        Vec2::new(self.x, self.y)
    }

    pub fn is_lit(&self) -> bool {
        self.r > 0.0 || self.g > 0.0 || self.b > 0.0
    }
}
