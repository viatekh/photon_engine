//! Photon Engine: laser output from Syphon / NDI video.

mod dac;
mod engine;
mod input;
mod recorder;
mod replay;
mod settings;
mod ui;

fn main() -> eframe::Result<()> {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info")).init();
    let args: Vec<String> = std::env::args().collect();
    if args.len() == 4 && args[1] == "--replay" {
        if let Err(e) = replay::run(&args[2], &args[3]) {
            eprintln!("replay failed: {e:#}");
            std::process::exit(1);
        }
        return Ok(());
    }
    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_title("Photon Engine")
            .with_inner_size([1280.0, 780.0]),
        ..Default::default()
    };
    eframe::run_native("Photon Engine", options, Box::new(|cc| Ok(Box::new(ui::App::new(cc)))))
}
