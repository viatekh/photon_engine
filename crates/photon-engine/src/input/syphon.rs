//! Syphon input via the Objective-C bridge in native/syphon_bridge.m.

use super::{FrameRef, SourceSelection, VideoSource};
use photon_core::image::PixelOrder;
use std::ffi::{c_char, c_int, c_void, CStr, CString};
use std::time::{Duration, Instant};

extern "C" {
    fn pe_syphon_server_count() -> c_int;
    fn pe_syphon_server_info(index: c_int, app: *mut c_char, name: *mut c_char, len: c_int) -> c_int;
    fn pe_syphon_open(app: *const c_char, name: *const c_char) -> *mut c_void;
    fn pe_syphon_read(h: *mut c_void, dst: *mut u8, cap: usize, w: *mut c_int, h: *mut c_int) -> c_int;
    fn pe_syphon_close(h: *mut c_void);
}

pub fn list() -> Vec<SourceSelection> {
    let mut out = Vec::new();
    unsafe {
        let n = pe_syphon_server_count();
        for i in 0..n {
            let mut app = [0 as c_char; 256];
            let mut name = [0 as c_char; 256];
            if pe_syphon_server_info(i, app.as_mut_ptr(), name.as_mut_ptr(), 256) != 0 {
                out.push(SourceSelection::Syphon {
                    app: CStr::from_ptr(app.as_ptr()).to_string_lossy().into_owned(),
                    name: CStr::from_ptr(name.as_ptr()).to_string_lossy().into_owned(),
                });
            }
        }
    }
    out
}

pub struct SyphonReceiver {
    handle: *mut c_void,
    buf: Vec<u8>,
}

// The bridge object is only touched from the thread that owns this receiver.
unsafe impl Send for SyphonReceiver {}

impl SyphonReceiver {
    pub fn connect(app: &str, name: &str) -> anyhow::Result<Self> {
        let (a, n) = (CString::new(app)?, CString::new(name)?);
        let handle = unsafe { pe_syphon_open(a.as_ptr(), n.as_ptr()) };
        if handle.is_null() {
            anyhow::bail!("Syphon server '{app} - {name}' not found");
        }
        Ok(Self { handle, buf: Vec::new() })
    }
}

impl VideoSource for SyphonReceiver {
    fn receive(&mut self, timeout: Duration, f: &mut dyn FnMut(FrameRef)) -> anyhow::Result<bool> {
        let deadline = Instant::now() + timeout;
        loop {
            let (mut w, mut h) = (0, 0);
            let r = unsafe { pe_syphon_read(self.handle, self.buf.as_mut_ptr(), self.buf.len(), &mut w, &mut h) };
            match r {
                1 => {
                    let (w, h) = (w as usize, h as usize);
                    f(FrameRef { data: &self.buf[..w * h * 4], width: w, height: h, stride: w * 4, order: PixelOrder::Bgra });
                    return Ok(true);
                }
                -2 => {
                    self.buf.resize(w as usize * h as usize * 4, 0);
                    continue;
                }
                -1 => anyhow::bail!("Syphon server went away"),
                _ => {}
            }
            if Instant::now() >= deadline {
                return Ok(false);
            }
            std::thread::sleep(Duration::from_millis(2));
        }
    }
}

impl Drop for SyphonReceiver {
    fn drop(&mut self) {
        unsafe { pe_syphon_close(self.handle) };
    }
}
