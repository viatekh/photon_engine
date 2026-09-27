//! Video inputs. Each source hands frames to a callback without copying where possible.

pub mod demo;
pub mod ndi;
#[cfg(all(target_os = "macos", has_syphon))]
pub mod syphon;

use photon_core::image::PixelOrder;
use serde::{Deserialize, Serialize};
use std::time::Duration;

/// A borrowed 8-bit, 4-channel frame.
pub struct FrameRef<'a> {
    pub data: &'a [u8],
    pub width: usize,
    pub height: usize,
    pub stride: usize,
    pub order: PixelOrder,
}

pub trait VideoSource: Send {
    /// Wait up to `timeout` for a new frame. Returns Ok(true) if `f` was called.
    fn receive(&mut self, timeout: Duration, f: &mut dyn FnMut(FrameRef)) -> anyhow::Result<bool>;
}

#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub enum SourceSelection {
    #[default]
    None,
    Syphon { app: String, name: String },
    Ndi { name: String },
    /// Built-in animations for testing without a video source.
    Demo,
    DemoFractal,
}

impl SourceSelection {
    pub fn label(&self) -> String {
        match self {
            SourceSelection::None => "None".into(),
            SourceSelection::Syphon { app, name } => {
                if name.is_empty() { format!("Syphon: {app}") } else { format!("Syphon: {app} - {name}") }
            }
            SourceSelection::Ndi { name } => format!("NDI: {name}"),
            SourceSelection::Demo => "Demo: rings (built in)".into(),
            SourceSelection::DemoFractal => "Demo: fractal zoom (built in)".into(),
        }
    }

    pub fn open(&self) -> anyhow::Result<Option<Box<dyn VideoSource>>> {
        match self {
            SourceSelection::None => Ok(None),
            SourceSelection::Demo => Ok(Some(Box::new(demo::DemoSource::new(demo::DemoKind::Rings)))),
            SourceSelection::DemoFractal => {
                Ok(Some(Box::new(demo::DemoSource::new(demo::DemoKind::Fractal))))
            }
            SourceSelection::Ndi { name } => Ok(Some(Box::new(ndi::NdiReceiver::connect(name)?))),
            #[cfg(all(target_os = "macos", has_syphon))]
            SourceSelection::Syphon { app, name } => {
                Ok(Some(Box::new(syphon::SyphonReceiver::connect(app, name)?)))
            }
            #[cfg(not(all(target_os = "macos", has_syphon)))]
            SourceSelection::Syphon { .. } => {
                anyhow::bail!("this build has no Syphon support (run scripts/setup_macos.sh and rebuild)")
            }
        }
    }
}

/// Everything the UI can offer to connect to right now.
pub fn available_sources() -> (Vec<SourceSelection>, Vec<String>) {
    let mut out = vec![SourceSelection::Demo, SourceSelection::DemoFractal];
    let mut notes = Vec::new();
    #[cfg(all(target_os = "macos", has_syphon))]
    out.extend(syphon::list());
    #[cfg(not(all(target_os = "macos", has_syphon)))]
    notes.push("Syphon not available in this build".to_string());
    match ndi::list() {
        Ok(names) => out.extend(names.into_iter().map(|name| SourceSelection::Ndi { name })),
        Err(e) => notes.push(format!("NDI unavailable: {e}")),
    }
    (out, notes)
}
