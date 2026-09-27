//! On macOS, compile the Syphon bridge if Syphon.framework is available.
//! Run `scripts/setup_macos.sh` to build the framework into `third_party/`,
//! or point `SYPHON_FRAMEWORK_DIR` at the directory containing `Syphon.framework`.

use std::path::PathBuf;

fn main() {
    println!("cargo::rustc-check-cfg=cfg(has_syphon)");
    println!("cargo:rerun-if-env-changed=SYPHON_FRAMEWORK_DIR");
    println!("cargo:rerun-if-changed=../../native/syphon_bridge.m");

    if std::env::var("CARGO_CFG_TARGET_OS").as_deref() != Ok("macos") {
        return;
    }
    let manifest = PathBuf::from(std::env::var("CARGO_MANIFEST_DIR").unwrap());
    let dir = std::env::var("SYPHON_FRAMEWORK_DIR")
        .map(PathBuf::from)
        .unwrap_or_else(|_| manifest.join("../../third_party"));
    println!("cargo:rerun-if-changed={}", dir.join("Syphon.framework").display());
    if !dir.join("Syphon.framework").exists() {
        println!(
            "cargo:warning=Syphon.framework not found in {} - building without Syphon input. \
             Run scripts/setup_macos.sh to enable it.",
            dir.display()
        );
        return;
    }
    let dir = dir.canonicalize().unwrap();
    cc::Build::new()
        .file("../../native/syphon_bridge.m")
        .flag("-fobjc-arc")
        .flag(format!("-F{}", dir.display()))
        .compile("syphon_bridge");
    println!("cargo:rustc-link-search=framework={}", dir.display());
    println!("cargo:rustc-link-lib=framework=Syphon");
    println!("cargo:rustc-link-lib=framework=Metal");
    println!("cargo:rustc-link-lib=framework=Foundation");
    println!("cargo:rustc-link-arg=-Wl,-rpath,{}", dir.display());
    println!("cargo:rustc-cfg=has_syphon");
}
