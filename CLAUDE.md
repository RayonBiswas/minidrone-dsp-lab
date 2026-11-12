# CLAUDE.md: Mini-Drone DSP Lab (handoff context)

## Who and what
- Owner: Rayon Biswas, T.Y. B.Tech EXTC, VJTI Mumbai. Team: Zaid Faruqui, Tanisha Nevrekar, Piyush Patil, Ananya Rane, Arpit Rajput.
- Course: DSP assignment "Digital Signal Processing in Today's Era". The results presentation is due in the 2nd–3rd week of November 2026. The project is also entered in the MathWorks Minidrone Competition India 2026.
- Project: **Noise-Robust Path Tracking for an Autonomous Mini-Drone**. It uses classical DSP only (no AI/ML) to clean each noisy sensor so a mini-drone can follow a coloured track using a downward camera and IMU.
- The aim, from proposal section 4, is to prove the chain works: inject noise on purpose, measure with metrics, and compare filter vs filter.
  - Metrics: detection accuracy %, SNR improvement (dB), RMSE of angle and height, tracking error, processing time per frame.
  - Comparisons: filter vs filter, FIR vs IIR (phase delay), gyro-only vs accel-only vs complementary vs Kalman.

## Live links and branches
- Live app (GitHub Pages): https://rayonbiswas.github.io/minidrone-dsp-lab/
- Repo: https://github.com/RayonBiswas/minidrone-dsp-lab
- Branches:
  - `main`: development.
  - `gh-pages`: **what the live site serves**. After every change, push to both: `git push origin main && git push origin main:gh-pages`.
  - `checkpoint-v1`: frozen first working version. Do not touch it.

## Files
- `index.html`: the entire phone web app in one file (vanilla JS and canvas, no build step).
- `matlab/minidrone_dsp_sim.m`: full MATLAB simulation. Sections: vision stages, Monte Carlo, ablation (2b), IMU, altitude, closed loop, summary. Needs the Image Processing and Signal Processing Toolboxes. **Never run in MATLAB yet, so expect small fixes.**
- `matlab/phone_sensor_demo.m`: MATLAB Mobile version. It reads the phone accel/gyro (`mobiledev`), runs FFT → notch → LPF → fusion, then takes a camera snapshot and runs the vision path. Also unverified.
- MATLAB style Rayon prefers: self-contained runnable `.m` scripts with local functions at the end, minimal student-style comments, and explicit per-step blocks rather than clever abstractions.

## App tabs (index.html)
- **Camera**: synthetic track, live phone camera, or photo. Pipeline: Median 3×3 (on the image) → optional Gaussian 5×5 → RGB→HSV threshold → close + open (disk r=2) → boundary edges → Hough (top 6 peaks, same-angle group, ρ = midpoint of the two track edges) → steering low-pass → PD steer.
  - Dashed pipeline chips toggle the filters. Sliders inject Gaussian noise, per-channel salt & pepper (like MATLAB `imnoise`), uneven lighting and motion blur.
  - The yellow line is the raw pipeline, green is DSP, white dashed is ground truth (synthetic only).
- **IMU**: live DeviceMotion (accelerationIncludingGravity + rotationRate) with a simulated fallback.
  - Chain: FFT on |a| finds up to 2 vibration peaks → auto notch biquads → IIR Butterworth or FIR (Hamming windowed-sinc) low-pass → roll = atan2(ax, hypot(ay, az)).
  - Gyro sign is auto-calibrated by correlation.
  - Fusion: gyro-only, complementary (α slider) and Kalman (angle + bias).
  - Readouts: fs, vibration peak, noise, drift, LPF delay at 1 Hz.
- **Altitude**: simulated take-off/hover/landing; median 7 → moving average 15 → 1D Kalman; spike burst button.
- **Flight**: a competition-style straight-segment track. Two drones get the same noise: raw (yellow) vs DSP (blue). Shows tracking error and steering jerk.
- **Drone**: camera + IMU together. Phone held flat, back camera down, top edge = nose. Artificial horizon shows roll and pitch, plus tracking and steering.
- **Navigate** (main localisation demo): optical flow by FFT phase correlation on 128×128 grayscale patches (Hann window, quality gate q<0.04, median of 3, EMA low-pass), gyro heading (rotationRate.alpha) with 1 s bias calibration at Set start and compass fusion (deviceorientationabsolute, 0.02 gain). The Calibrate 50 cm button learns cm/px and the camera axis, saved in localStorage. Arena 150×100 cm, obstacle, A* on a 5 cm grid with line-of-sight smoothing, re-planned every 300 ms; arrow + speechSynthesis directions; 'I'm at the goal' measures final error. Sim: value-noise floor at 5.12 px/cm with ground truth. E6 drift test: mean 11 cm goal error (7% drift over ~154 cm) filtered vs ~86 cm raw. Unit test: phase correlation recovers known shifts to within 0.03 cm.
- **Beacon (archived)**: ESP32 beacon navigation (see archive/esp32/README.md), reachable via a button in Navigate; not in the nav bar. Sim with path-loss + shadowing + multipath fades; raw vs filtered drones fly start → goal; calibration fit (P0, n); live input via Web Bluetooth (Nordic UART, device DRONE-RX) or Web Serial 115200; CSV replay (esp32/sample_log.csv). Sim result: position error about 2.2 m raw → about 0.8–1.0 m filtered; the median gives the biggest gain; the position Kalman mainly smooths the path rather than cutting RMS error.
- **Results**: E1–E5 with a fixed seed (mulberry32 `setSeed`). Live sensor sims use `randnM` (Math.random) so they never disturb the seeded runs. Copy tables (Markdown) and Download CSV.

## Current results (fixed seed, deterministic)
- **E1 vision, filter vs filter:**
  - Salt & pepper: Median + morph (ours) wins, 100% success / 0.31° vs raw 90% / 1.84°. Median beats Gaussian (0.31° vs 0.47°).
  - Gaussian noise: Median 0.51°, Gaussian 0.63°, raw 2.54°.
  - Mixed noise: the Gaussian alone is best and cheapest (0.62°, 2.7 ms) vs ours 0.71° (11.9 ms), which is within the spread.
  - Morphology alone hurts.
- **E2 noise sweep:** raw angle error goes from 0.5° to 3.2° as salt & pepper rises to 25%. All filtered versions stay ≤ 0.54°.
- **E3 inertial:**
  - IIR LPF 82 ms delay vs FIR (41 taps) 200 ms; IIR 1.8° vs FIR 4.5° roll RMSE.
  - The notch adds nothing at 18/36 Hz (the LPF already removes it), but at 5 Hz vibration it cuts error from 5.9° to 1.9°.
  - Gyro-only 21° (drift), complementary 1.4°, Kalman 0.7°.
- **E4 altitude:** raw 20 dB → median + Kalman 34.6 dB SNR. Linear filters smear spikes; the median removes them.
- **E5 flight:** both drones stay on track. The DSP drone steers 7–9× more smoothly at about 1 cm more tracking error (low-pass lag).
- **Honesty rule:** never tune or fake results to make "ours" win. Hough voting is already robust, so the DSP gain is precision and smoothness. Present trade-offs openly.

## Commit rules
- Commits authored as RayonBiswas only. No Co-Authored-By or Claude-Session lines, no "Generated with" footers.
- No force-push or history rewrite without his go-ahead and a backup.

## Known issues
- **Brave browser blocks motion sensors**; use Chrome on the phone (Android). In-app browsers (WhatsApp/Gmail) also block sensors.
- When sensors fail, the IMU tab shows a "Diagnostics:" line (secure page, DeviceMotionEvent present, events received).
- Phone IMU samples at about 60–100 Hz, so vibration injection is capped at 0.45·fs.
- Inside the claude.ai artifact viewer, camera, motion and downloads are blocked (simulated mode only). Use the GitHub Pages link.

## Current task: record the demo on the laptop
- Goal: a roughly 3-minute video showing the **phone screen** and the **phone/hands moving over the red cable** at the same time, so it is clearly live, not hardcoded.
- Setup on Windows (Mac is similar):
  1. **scrcpy** (github.com/Genymobile/scrcpy releases). On the phone, enable Developer options → USB debugging, connect USB, then run `scrcpy` or `scrcpy --record=demo.mp4`.
  2. **OBS Studio** sources: Window Capture (scrcpy window) as the main view, Video Capture Device (laptop webcam on the hands/cable) in a corner, and Mic. Set the recording format to mp4.
  3. Quick fallback: the phone's built-in Screen recorder with show-taps and mic.
- Shot list:
  1. Camera tab, synthetic track: raise salt & pepper, toggle Median off/on.
  2. Camera tab, phone camera on the red cable: rotate and slide the phone so θ, d and the steering respond.
  3. IMU tab: tilt; shake sideways without tilting (raw jumps, fused stays steady); fan → FFT peak → notch, then Auto notch off; switch IIR → FIR to show the delay.
  4. Drone tab: walk along the cable holding the phone flat.
  5. Flight tab: raw vs DSP steering plot.
  6. Results tab on the laptop: tables and verdicts.

## Plan A (current direction, see PLAN_A.md)
- Stick to the proposal: track following + landing marker, not map/navigation (Navigate tab stays as an extra, not the core).
- P1: Simulink minidrone with `matlab/simulink/` blocks (dspVisionBlock, addCameraNoise, steeringLowpass). Untested in Simulink.
- P2: ESP32 + MPU6050 + unbalanced DC motor, firmware `hardware/esp32_mpu6050/` at 500 Hz, internal DLPF off; IMU tab 'ESP32 + MPU6050 (USB)' via Web Serial 921600 baud, 'Record 20 s CSV'; analysis `matlab/esp32_mpu6050_analysis.m`. Phone IMU (60–100 Hz) aliases motor vibration, hence the rig.
- P3: real red-tape track + blue landing pad; Camera tab pad detection (median + morph blue mask, 6-of-8-frame vote → LAND ✓, flicker counter).

## Demo plan (Navigate, extra)
- Arena: chart paper or newspaper about 1.5 × 1 m with START, GOAL and a book as the obstacle.
- Steps: Calibrate 50 cm once → Set start (hold still 1 s) → follow the arrow and voice → go the wrong way on purpose (it re-plans) → reach the goal, tap 'I'm at the goal' to get the error → switch off Hann window or bias cal and repeat to show the raw drift.

## Next tasks (in order)
1. Record the demo video (above).
2. Make the presentation slides: problem → pipeline → results tables/graphs → live demo → limitations → next steps.
3. Run `matlab/minidrone_dsp_sim.m` in MATLAB and fix any errors.
4. Later: plug into the Simulink minidrone model (swap the synthetic frames for the model's camera output), and optional ESP32 + MPU6050 hardware (about ₹1,700).

## Viva answers to keep handy
- Median vs Gaussian: the median discards impulses; the Gaussian averages them into neighbours.
- IIR vs FIR: IIR has lower delay, which matters inside a control loop; FIR is linear-phase but slow.
- Complementary vs Kalman: Kalman is more accurate; complementary is nearly as good for far less compute.
- Yaw: the accelerometer cannot sense it because gravity doesn't change when you turn flat, so heading must come from the gyro (drifts) or the camera line angle θ.
- Why the raw drone survives: Hough voting is robust; the DSP adds precision and smooth commands, which stop motor oscillation on a real drone.
