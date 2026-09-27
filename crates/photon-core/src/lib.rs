//! Photon Engine core: everything between "a video frame arrived" and "points for the DAC",
//! with no platform or hardware dependencies.

pub mod detail;
pub mod galvo_sim;
pub mod geom;
pub mod image;
pub mod keystone;
pub mod output;
pub mod patterns;
pub mod planner;
pub mod scan;
pub mod vectorise;

pub use geom::{LaserPoint, Path, Rgb, Vec2};
