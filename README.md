# Photon Engine

Laser control software that takes a video feed (Syphon / NDI, e.g. from Resolume), traces it into
vector paths, decides what can be drawn **without beam breakup**, and streams it to a laser DAC.

Current output: **LaserCube / LaserDock over USB** (plus a simulator for working without hardware).

## Quick start (Apple Silicon Mac)

1. Install Rust: https://rustup.rs (needs Rust 1.95 or newer; `rustup update` if you already have it).
2. For **Syphon** input: install Xcode, then run once:
   ```sh
   ./scripts/setup_macos.sh
   ```
   This builds `Syphon.framework` into `third_party/`. Without it the app still builds, just without Syphon.
3. For **NDI** input: install the NDI SDK for Apple (https://ndi.video/for-developers/ndi-sdk/).
   The runtime is loaded at start-up, so nothing is needed at build time.
4. Run:
   ```sh
   cargo run --release
   ```
5. **Close LaserOS** (only one app can own the LaserCube's USB connection).

### First test with the laser

1. The app starts **disarmed**, with the **Frame + cross** test pattern on and brightness at 30%.
2. Output → Device → *LaserCube / LaserDock USB*. The top bar shows the firmware and max point rate.
3. Click **ARM**. **Space** or **Esc** blacks out immediately.
4. Use the Frame pattern to check orientation (the red arrow points up; use Flip X/Y if not), then
   tick *Edit corners* and drag the yellow handles in the output preview to keystone.
5. Untick *Test pattern*, pick your Syphon/NDI source (or a built-in demo: *rings* or *fractal zoom*).

In Resolume: Output → Syphon/Spout server (or NDI). Any content works in **Edges** mode (the default);
thin bright lines on black also work well in **Centreline** mode.

If a new version changes defaults, **Reset all settings** (bottom of the left panel) picks them up;
it keeps your source, device and keystone.

## How it works

```
source frame ─► downsample ─► trace (centreline | outline) ─► smooth/simplify
            ─► keystone + clip ─► anti-breakup planner ─► colour/safety ─► DAC
```

* **Tracing.** *Edges* (default) handles any content, including films and fractals: blur → gradient →
  thin edges to single lines → link weak edges onto strong ones (Canny) → follow the edges as line art. Each path is
  weighted by its edge contrast. *Centreline* thins bright areas to a 1px skeleton and follows it (best for line
  content). *Outline* traces region edges with marching squares (works for anything, but a line
  becomes a loop: twice the scan time).
* **Auto detail.** A feedback loop compares how many points the traced content would need with
  the budget, and every frame raises or lowers the edge threshold, minimum stroke length and
  (for very busy content) blur. A sparse frame gets more detail; a fractal keeps only its
  strongest, longest structures. *Temporal smoothing* blends frames to reduce edge flicker on video.
* **Scan model.** Paths are resampled so the beam never moves faster than the *lit speed* (blanked
  jumps use *blank speed*), with dwell at corners, path ends and blank transitions. Settings are in
  physical units (speed, µs), so they don't change meaning when you change the point rate.
* **Anti-breakup.** Budget = points/sec ÷ refresh rate. A shape is only ever drawn **whole**; every
  strategy ends with the whole-shapes pass, so a frame never exceeds the budget.
  * **A. Whole shapes:** greedily keep complete shapes in priority order (salient = length ×
    edge strength, the default; or largest / longest / brightest / most central) until the budget at the *target* refresh is full.
  * **B. Simplify:** find the smallest Douglas-Peucker simplification (≤ *max simplify*) that fits,
    then A.
  * **C. Adaptive refresh:** allow refresh to drop to *min refresh* before dropping shapes.
  * **D. Take turns:** split shapes into up to *max groups* drawn on alternate passes. More content,
    more flicker.
  * **E. Combined:** C, then B, then A.
  * *Stickiness* favours shapes drawn last frame so the selection doesn't flicker between frames.
* The output preview shows exactly what is sent to the DAC; dropped shapes show in grey.

## Safety behaviour

* Always starts disarmed; arming is never saved. Space/Esc blacks out at once (the current frame is cut short).
* Blackout if no new input frame arrives within the *signal-loss* timeout (default 500 ms).
* **Static beam guard:** a frame whose lit points all fall within a tiny area (a near-stationary
  beam) is blanked.
* On disarm the DAC's hardware output enable is switched off as well as sending dark points.

This is **not** a certified safety system. Don't scan audiences, and keep a physical interlock or
e-stop within reach.

## Layout

| Path | What |
|---|---|
| `crates/photon-core` | Platform-independent pipeline: tracing, scan rendering, planner, keystone, colour, safety. Unit tested. |
| `crates/photon-engine` | App: egui UI, worker threads, inputs (`input/`), DACs (`dac/`). |
| `native/syphon_bridge.m` | Small Objective-C bridge to Syphon (Metal client, CPU read-back). |
| `scripts/setup_macos.sh` | Builds Syphon.framework. |

Adding a DAC means implementing the `Dac` trait in `crates/photon-engine/src/dac/`. Adding an input
means implementing `VideoSource` in `crates/photon-engine/src/input/`.

```sh
cargo test                     # core tests
cargo test --release -p photon-core --test pipeline -- --ignored --nocapture   # timing
```

## Known unknowns (please report back)

* **Not yet run on a Mac or on real hardware.** It was developed on Linux: the core is tested, the
  macOS code type-checks for Apple Silicon, and the GUI was exercised with the simulator. The
  Objective-C Syphon bridge has **not** been compiled yet.
* **LaserCube USB protocol** has been checked against Wicked Lasers' libLaserdockCore source (VID 0x1fc9,
  PID 0x04d8; command endpoint 0x01, sample endpoint 0x03, 8-byte samples, X mirrored as in their
  code). Their optional host-side "security" handshake (which checks the unit is genuine) isn't sent; the
  cube shouldn't need it to accept output.
* **Interlock:** the LaserCube's interlock works in hardware, but over USB its state is not
  reported (only over WiFi), so the app can't show it. If the app is armed and streaming but the
  laser is dark, check the key/interlock.
* **Syphon texture orientation**: if the image is upside down, tick *Flip input vertically*.
* Scanner defaults (lit speed 450, blank speed 1500, dwells) are guesses for a LaserCube's
  galvos and need tuning by eye.
* Spout (Windows) is not implemented yet; the input layer is ready for it.
