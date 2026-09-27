//! NDI input. The NDI runtime is loaded at run time, so the app builds and runs without it;
//! install the NDI SDK (or NDI Tools) to enable NDI sources.

use super::{FrameRef, VideoSource};
use libloading::Library;
use photon_core::image::PixelOrder;
use std::ffi::{c_char, c_void, CStr, CString};
use std::ptr;
use std::path::PathBuf;
use std::sync::{Mutex, OnceLock};
use std::time::Instant;
use std::time::Duration;

// --- Subset of Processing.NDI.Lib.h (NDI 5/6). Enums are 32-bit in the SDK. ---

#[repr(C)]
#[derive(Clone, Copy)]
struct NdiSource {
    p_ndi_name: *const c_char,
    p_url_address: *const c_char,
}

#[repr(C)]
struct NdiFindCreate {
    show_local_sources: bool,
    p_groups: *const c_char,
    p_extra_ips: *const c_char,
}

#[repr(C)]
struct NdiRecvCreateV3 {
    source_to_connect_to: NdiSource,
    color_format: i32,
    bandwidth: i32,
    allow_video_fields: bool,
    p_ndi_recv_name: *const c_char,
}

#[repr(C)]
struct NdiVideoFrameV2 {
    xres: i32,
    yres: i32,
    fourcc: u32,
    frame_rate_n: i32,
    frame_rate_d: i32,
    picture_aspect_ratio: f32,
    frame_format_type: i32,
    timecode: i64,
    p_data: *mut u8,
    line_stride_in_bytes: i32,
    p_metadata: *const c_char,
    timestamp: i64,
}

const COLOR_FORMAT_RGBX_RGBA: i32 = 2;
const BANDWIDTH_HIGHEST: i32 = 100;
const FRAME_TYPE_VIDEO: i32 = 1;
const FRAME_TYPE_ERROR: i32 = 4;

const fn fourcc(s: &[u8; 4]) -> u32 {
    s[0] as u32 | (s[1] as u32) << 8 | (s[2] as u32) << 16 | (s[3] as u32) << 24
}
const FOURCC_RGBA: u32 = fourcc(b"RGBA");
const FOURCC_RGBX: u32 = fourcc(b"RGBX");
const FOURCC_BGRA: u32 = fourcc(b"BGRA");
const FOURCC_BGRX: u32 = fourcc(b"BGRX");

type Instance = *mut c_void;

struct Api {
    _lib: Library,
    find_get_current_sources: unsafe extern "C" fn(Instance, *mut u32) -> *const NdiSource,
    recv_create_v3: unsafe extern "C" fn(*const NdiRecvCreateV3) -> Instance,
    recv_destroy: unsafe extern "C" fn(Instance),
    recv_capture_v2:
        unsafe extern "C" fn(Instance, *mut NdiVideoFrameV2, *mut c_void, *mut c_void, u32) -> i32,
    recv_free_video_v2: unsafe extern "C" fn(Instance, *const NdiVideoFrameV2),
    finder: Instance,
}

// The NDI API is thread-safe; the finder is only read via find_get_current_sources.
unsafe impl Send for Api {}
unsafe impl Sync for Api {}

/// Folders that may contain the NDI runtime library.
fn candidate_dirs() -> Vec<PathBuf> {
    let mut dirs: Vec<PathBuf> = Vec::new();
    for var in ["NDI_RUNTIME_DIR_V6", "NDI_RUNTIME_DIR_V5", "NDI_RUNTIME_DIR_V4"] {
        if let Ok(dir) = std::env::var(var) {
            dirs.push(dir.into());
        }
    }
    #[cfg(target_os = "macos")]
    {
        for d in [
            "/Library/NDI SDK for Apple/lib/macOS",
            "/Library/NDI Advanced SDK for Apple/lib/macOS",
            "/Library/NDI SDK for macOS/lib/macOS",
            "/usr/local/lib",
            "/opt/homebrew/lib",
            "/Library/Frameworks",
        ] {
            dirs.push(d.into());
        }
        // NDI Tools apps bundle the runtime inside themselves.
        for apps in ["/Applications", "/Applications/NDI Tools"] {
            if let Ok(entries) = std::fs::read_dir(apps) {
                for e in entries.flatten() {
                    let name = e.file_name().to_string_lossy().to_string();
                    if name.contains("NDI") && name.ends_with(".app") {
                        for sub in ["Contents/Frameworks", "Contents/MacOS", "Contents/Resources"] {
                            dirs.push(e.path().join(sub));
                        }
                    }
                }
            }
        }
    }
    #[cfg(target_os = "windows")]
    dirs.push("C:\\Program Files\\NDI\\NDI 6 Runtime\\v6".into());
    #[cfg(all(unix, not(target_os = "macos")))]
    dirs.extend(["/usr/lib".into(), "/usr/local/lib".into(), "/usr/lib/x86_64-linux-gnu".into()]);
    dirs
}

fn is_ndi_lib(name: &str) -> bool {
    #[cfg(target_os = "macos")]
    return name.starts_with("libndi") && name.ends_with(".dylib");
    #[cfg(target_os = "windows")]
    return name.starts_with("Processing.NDI.Lib") && name.ends_with(".dll");
    #[cfg(all(unix, not(target_os = "macos")))]
    return name.starts_with("libndi.so");
}

/// Every NDI library file found, plain SDK builds before "advanced" ones.
fn candidate_files() -> Vec<PathBuf> {
    let mut files = Vec::new();
    for dir in candidate_dirs() {
        let Ok(entries) = std::fs::read_dir(&dir) else { continue };
        let mut found: Vec<PathBuf> = entries
            .flatten()
            .map(|e| e.path())
            .filter(|p| p.file_name().is_some_and(|n| is_ndi_lib(&n.to_string_lossy())))
            .collect();
        found.sort_by_key(|p| p.to_string_lossy().contains("advanced"));
        files.extend(found);
    }
    files
}

fn api() -> Result<&'static Api, String> {
    // Only success is cached, so installing NDI while the app runs is picked up (retried every
    // few seconds rather than on every call).
    static API: OnceLock<&'static Api> = OnceLock::new();
    static LAST_TRY: Mutex<Option<(Instant, String)>> = Mutex::new(None);
    if let Some(api) = API.get() {
        return Ok(api);
    }
    let mut last = LAST_TRY.lock().unwrap();
    if let Some((at, err)) = last.as_ref() {
        if at.elapsed() < Duration::from_secs(5) {
            return Err(err.clone());
        }
    }
    match unsafe { load() } {
        Ok(api) => {
            let api: &'static Api = Box::leak(Box::new(api));
            let _ = API.set(api);
            log::info!("NDI runtime loaded");
            Ok(api)
        }
        Err(e) => {
            *last = Some((Instant::now(), e.clone()));
            Err(e)
        }
    }
}

unsafe fn load() -> Result<Api, String> {
    let files = candidate_files();
    let mut errors = Vec::new();
    let mut lib = None;
    for f in &files {
        match Library::new(f) {
            Ok(l) => {
                log::info!("loading NDI runtime from {}", f.display());
                lib = Some(l);
                break;
            }
            Err(e) => errors.push(format!("{}: {e}", f.display())),
        }
    }
    let Some(lib) = lib else {
        let msg = if errors.is_empty() {
            "NDI runtime not found. Run `find / -name \"libndi*\" 2>/dev/null` in Terminal to locate it, \
             then start the app with NDI_RUNTIME_DIR_V6=<that folder>"
                .to_string()
        } else {
            format!("NDI runtime found but failed to load: {}", errors.join("; "))
        };
        log::warn!("{msg}");
        return Err(msg);
    };
    macro_rules! sym {
        ($name:literal) => {
            *lib.get(concat!($name, "\0").as_bytes()).map_err(|e| format!("{}: {e}", $name))?
        };
    }
    let initialize: unsafe extern "C" fn() -> bool = sym!("NDIlib_initialize");
    let find_create_v2: unsafe extern "C" fn(*const NdiFindCreate) -> Instance = sym!("NDIlib_find_create_v2");
    if !initialize() {
        return Err("NDIlib_initialize failed (unsupported CPU?)".into());
    }
    let create = NdiFindCreate { show_local_sources: true, p_groups: ptr::null(), p_extra_ips: ptr::null() };
    let finder = find_create_v2(&create);
    if finder.is_null() {
        return Err("could not create NDI finder".into());
    }
    Ok(Api {
        find_get_current_sources: sym!("NDIlib_find_get_current_sources"),
        recv_create_v3: sym!("NDIlib_recv_create_v3"),
        recv_destroy: sym!("NDIlib_recv_destroy"),
        recv_capture_v2: sym!("NDIlib_recv_capture_v2"),
        recv_free_video_v2: sym!("NDIlib_recv_free_video_v2"),
        finder,
        _lib: lib,
    })
}

/// Names of NDI sources currently visible on the network (including this machine).
pub fn list() -> Result<Vec<String>, String> {
    let api = api()?;
    let mut n = 0u32;
    let sources = unsafe { (api.find_get_current_sources)(api.finder, &mut n) };
    if sources.is_null() {
        return Ok(Vec::new());
    }
    let sources = unsafe { std::slice::from_raw_parts(sources, n as usize) };
    Ok(sources
        .iter()
        .filter(|s| !s.p_ndi_name.is_null())
        .map(|s| unsafe { CStr::from_ptr(s.p_ndi_name) }.to_string_lossy().into_owned())
        .collect())
}

pub struct NdiReceiver {
    api: &'static Api,
    recv: Instance,
}

unsafe impl Send for NdiReceiver {}

impl NdiReceiver {
    pub fn connect(name: &str) -> anyhow::Result<Self> {
        let api = api().map_err(anyhow::Error::msg)?;
        let cname = CString::new(name)?;
        let recv_name = CString::new("Photon Engine")?;
        let create = NdiRecvCreateV3 {
            source_to_connect_to: NdiSource { p_ndi_name: cname.as_ptr(), p_url_address: ptr::null() },
            color_format: COLOR_FORMAT_RGBX_RGBA,
            bandwidth: BANDWIDTH_HIGHEST,
            allow_video_fields: false,
            p_ndi_recv_name: recv_name.as_ptr(),
        };
        let recv = unsafe { (api.recv_create_v3)(&create) };
        if recv.is_null() {
            anyhow::bail!("could not create NDI receiver for '{name}'");
        }
        Ok(Self { api, recv })
    }
}

impl VideoSource for NdiReceiver {
    fn receive(&mut self, timeout: Duration, f: &mut dyn FnMut(FrameRef)) -> anyhow::Result<bool> {
        let mut frame: NdiVideoFrameV2 = unsafe { std::mem::zeroed() };
        let t = unsafe {
            (self.api.recv_capture_v2)(
                self.recv,
                &mut frame,
                ptr::null_mut(),
                ptr::null_mut(),
                timeout.as_millis() as u32,
            )
        };
        if t == FRAME_TYPE_ERROR {
            anyhow::bail!("NDI connection lost");
        }
        if t != FRAME_TYPE_VIDEO {
            return Ok(false);
        }
        let order = match frame.fourcc {
            FOURCC_RGBA | FOURCC_RGBX => Some(PixelOrder::Rgba),
            FOURCC_BGRA | FOURCC_BGRX => Some(PixelOrder::Bgra),
            _ => None,
        };
        let ok = match order {
            Some(order) if !frame.p_data.is_null() && frame.xres > 0 && frame.yres > 0 => {
                let (w, h) = (frame.xres as usize, frame.yres as usize);
                let stride = if frame.line_stride_in_bytes > 0 { frame.line_stride_in_bytes as usize } else { w * 4 };
                let data = unsafe { std::slice::from_raw_parts(frame.p_data, stride * h) };
                f(FrameRef { data, width: w, height: h, stride, order });
                true
            }
            _ => {
                log::warn!("unsupported NDI pixel format {:#x}", frame.fourcc);
                false
            }
        };
        unsafe { (self.api.recv_free_video_v2)(self.recv, &frame) };
        Ok(ok)
    }
}

impl Drop for NdiReceiver {
    fn drop(&mut self) {
        unsafe { (self.api.recv_destroy)(self.recv) };
    }
}
