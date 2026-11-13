# Running our DSP chain in the MathWorks minidrone model (proposal §4: test platform)

The competition's `parrotMinidroneCompetition` Simulink project has a **Flight Control System** block with two parts. The **Image Processing System** receives camera data and outputs vision data. The **Control System** (path planning, controllers) uses that data. Our vision chain goes in the first; the low-passed steering error goes in the second.

## Setup (once)
1. MATLAB with Simulink, Image Processing Toolbox, Signal Processing Toolbox, DSP System Toolbox and Computer Vision Toolbox. The competition licence includes them.
2. Install the **Simulink Support Package for Parrot Minidrones** (Add-On Explorer).
3. Open the competition project using the command in the competition guidelines, then open the **Flight Control System** block.
4. Copy `dspVisionBlock.m`, `addCameraNoise.m` and `steeringLowpass.m` into the main `parrotMinidroneCompetition` folder. The rules say not to add files to subfolders.

## Wire it up
1. **Image Processing System:** after the block that converts the camera image to RGB, split it into R, G and B planes.
2. Add a **MATLAB Function** block and paste in the body of `addCameraNoise.m`. Inputs: R, G, B plus constants `sigma`, `spDensity`, `seed`.
3. Add a second **MATLAB Function** block with `dspVisionBlock.m`. Feed it the noisy R, G, B and a constant `useFilters` (1 or 0).
4. Output `theta`, `d`, `lineFound`, `padFound`, `padX`, `padY` from the Image Processing System. They cross the existing rate-transition block into the Control System. Don't send image data across.
5. **Control System → Path Planning:** add a MATLAB Function block with `steeringLowpass.m` and use `eF` as the yaw/lateral error your path-planning logic feeds the controller. Use `padFound` and `padX/padY` to trigger and centre the landing.

## Experiments to run (same as the app's Results tab)
| Run | sigma | spDensity | useFilters | Record |
|---|---|---|---|---|
| Baseline | 0 | 0 | 1 | track completed? landing error |
| Noise, raw | 0.1 | 0.08 | 0 | |
| Noise, DSP | 0.1 | 0.08 | 1 | |
| Heavy noise, raw | 0.2 | 0.15 | 0 | |
| Heavy noise, DSP | 0.2 | 0.15 | 1 | |

For each run, log the drone's x, y position (simulation outputs) and compute tracking error against the track centreline and landing distance from the pad centre. Screenshot the 3D view and the video viewer for the slides.

## IMU vibration in the model (optional)
In the sensor model, add a **Sine Wave** block (e.g. 40 Hz, 2 m/s²) to the accelerometer signal to mimic propeller vibration. The model's estimator then sees the same problem our notch filter solves. Exact block names differ between support-package versions, so look for the accelerometer signal inside the sensors subsystem.

## Status
- The three blocks use plain loops and fixed-size arrays only (no Image Processing Toolbox calls), so they should compile in a MATLAB Function block on any licence.
- `dspVisionBlock` has the same real-camera fixes as the app: a bottom-half ROI, the biggest blob only, and a least-squares refinement of the Hough line. Change `hueC`, `hueW` and `satMin` at the top to match a non-red track.
- **Tested in plain MATLAB** with `test_simulink_blocks.m` (R2026b): line within 0.4° / 0.2 px, pad centroid exact, and steering low-pass step and hold are correct. Angle RMSE raw → DSP: 0.47° → 0.70° clean (the ROI halves the votes), 0.71° → 0.73° with noise, 1.90° → 0.82° with heavy noise.
- **Not yet run inside Simulink.** Expect small fixes in the model wiring, such as signal sizes, uint8 vs double ports and sample times.
