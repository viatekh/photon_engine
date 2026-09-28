//! Background scan for attached hardware (webcams, LaserCubes), so devices that are plugged in,
//! unplugged or re-plugged show up without restarting the app. The camera and output threads
//! use the latest scan to (re)connect.

use crate::camera::{self, CameraSelection};
use crate::engine::Shared;
use std::sync::atomic::Ordering;
use std::sync::Arc;
use std::thread;
use std::time::{Duration, Instant};

const INTERVAL: Duration = Duration::from_secs(2);

#[derive(Clone, Default)]
pub struct DeviceList {
    pub cameras: Vec<CameraSelection>,
    pub camera_note: String,
    /// LaserCubes currently on USB.
    pub lasercubes: usize,
    /// None until the first scan has finished.
    pub scanned: Option<Instant>,
}

pub fn start(shared: Arc<Shared>) -> thread::JoinHandle<()> {
    thread::Builder::new().name("devices".into()).spawn(move || run(shared)).unwrap()
}

fn run(shared: Arc<Shared>) {
    let mut next = Instant::now();
    while !shared.shutdown.load(Ordering::Relaxed) {
        if Instant::now() >= next || shared.rescan_devices.swap(false, Ordering::SeqCst) {
            let (cameras, camera_note) = camera::list_devices();
            let lasercubes = crate::dac::lasercube_usb::count();
            let mut d = shared.devices.lock();
            if d.scanned.is_some() && (d.cameras != cameras || d.lasercubes != lasercubes) {
                log::info!(
                    "devices changed: cameras [{}], {} LaserCube(s)",
                    cameras.iter().map(|c| c.label()).collect::<Vec<_>>().join(", "),
                    lasercubes
                );
            }
            *d = DeviceList { cameras, camera_note, lasercubes, scanned: Some(Instant::now()) };
            next = Instant::now() + INTERVAL;
        }
        thread::sleep(Duration::from_millis(100));
    }
}
