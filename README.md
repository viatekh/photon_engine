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

* **Tracing.** *Auto* (default) handles any content. Thin bright strokes are found with a ridge
  detector and traced **once** down their middle; only line-like stroke components count (long
  enough, few branches), so texture isn't mistaken for lines. Everything else (filled shapes,
  film, fractals) is traced by its contrast edges, skipping edges that just border a stroke.
* **Shapes.** Everything traced from one connected piece of the image is one *shape* (e.g. two
  overlapping rings). The planner draws a shape completely or not at all. Minimum length applies
  per shape, so a shape never loses small pieces. A shape too big to fit in a frame on its own is
  split into its separate strokes (each still whole). This can be switched off.
* Other modes: *Edges only* (Canny: blur → gradient → thin edges → link weak edges onto strong
  ones → follow as line art; a thin line gives two edges). *Centreline* thins bright areas to a 1px skeleton and follows it (best for line
  content). *Outline* traces region edges with marching squares (works for anything, but a line
  becomes a loop: twice the scan time).
* **Auto detail.** A feedback loop compares how many points the traced content would need with
  the budget, and every frame raises or lowers the edge threshold, minimum shape size and
  (for very busy content) edge blur. It resets when you change source. A sparse frame gets more detail; a fractal keeps only its
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

## Recording a problem for analysis

If something looks wrong on the laser (flicker, shapes jumping, missing or wrong shapes):

1. Click **● Record** in the top bar while it happens (it stops by itself after 20 s).
2. The file lands in `recordings/rec-<time>.jsonl.gz` (the path is shown next to the button).
3. Send it: either commit it (`git add recordings/<file> && git commit -m "recording" && git push`)
   or put it in Google Drive. Say what you saw and roughly when.

A recording holds, per frame: the small working image the tracer saw, every traced shape (drawn or
dropped), the planner's numbers and the output state, plus your settings. It does not contain the
full-resolution video. Typical size is 5–30 MB.

Tools:

```sh
python3 tools/analyse_recording.py recordings/rec-123.jsonl.gz              # stability report
python3 tools/analyse_recording.py recordings/rec-123.jsonl.gz --sheet 100 16   # frames as an image
cargo run --release -- --replay recordings/rec-123.jsonl.gz /tmp/after.jsonl.gz # re-run with current code
```

## Camera calibration (measuring the real laser)

For development: a webcam watching the projection lets me measure what the real laser does
(mirror lag, overshoot, colour delay, settle time) instead of guessing.

1. `brew install ffmpeg` (used to read the webcam).
2. Point a USB webcam at the projection surface - **never into the beam**. Fixed on a stand,
   whole projection in view, nothing in front of it, room as dark as possible (the analysis copes
   with some ambient light, but the webcam's auto exposure then drops to ~6 fps).
3. Left panel → **Camera & calibration** → pick yours. The preview should show the laser, live,
   with a green "live - 30 fps" line. (macOS will ask for camera permission for the Terminal the
   first time.) If the feed freezes or ffmpeg refuses a capture mode, the app tries the next
   mode by itself; the panel shows the mode in use and ffmpeg's messages.
4. Arm the laser, click **Run calibration session**. It draws ~28 test patterns (≈1 minute),
   restores your settings, and saves `recordings/calib-<time>.jsonl.gz`.
5. Send that file (git push, as with recordings).

`--analyse-calibration FILE OUT_DIR` aligns the camera to laser coordinates, writes overlays
(commanded path in red over the camera image) and measures coverage / stray light per step.
"Simulated (galvo model)" camera runs the whole flow without hardware.

## Plugging devices in and out

Cameras and LaserCubes are rescanned every 2 s, so there's no need to restart the app. A camera is
remembered by name (macOS renumbers cameras when they're re-plugged) and capture resumes when it
comes back. The LaserCube reconnects by itself; Output shows whether one is on USB. Syphon / NDI
sources are also refreshed every 2 s and reconnect after a dropout.

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
* **Two different "kpps" numbers:** *DAC point rate* (points/sec sent; sets the per-frame budget,
  capped at what the cube reports) and *Scanner rating* (galvo speed, e.g. 30K at ILDA 8°, which
  scales the lit/blank speeds and dwells). Both are under Output. The LaserCube's galvo rating
  isn't published anywhere I could check: start at 30 and lower it if corners overshoot or
  lines wobble. Wicked Lasers' own software treats the USB cube's max rate as its current rate,
  so the max it reports may not be reliable.
* Spout (Windows) is not implemented yet; the input layer is ready for it.
