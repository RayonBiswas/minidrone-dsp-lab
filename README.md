# Noise-Robust Path Tracking for an Autonomous Mini-Drone

Classical DSP (no AI/ML) for camera, IMU and altitude signals: DSP course project ("Digital Signal Processing in Today's Era", VJTI) and MathWorks Minidrone Competition India 2026 entry.

**Live demo:** open the GitHub Pages link on a phone and tap *Phone camera* or *Start phone sensors* to run the pipeline on the phone's real camera, accelerometer and gyroscope.

| Path | Chain |
|---|---|
| Vision | Median 3×3 → RGB→HSV threshold → close/open → edges → Hough → low-pass → PID |
| Inertial | FFT peak search → auto notch → low-pass → atan2 roll → complementary / Kalman fusion with gyro |
| Altitude | Median → moving average / 1D Kalman |

- `index.html`: the phone web app (vision, IMU, altitude, closed-loop flight). It falls back to simulated sensors when the real ones are unavailable.
- `matlab/minidrone_dsp_sim.m`: the full simulation in MATLAB (Image Processing and Signal Processing Toolboxes).
- `matlab/phone_sensor_demo.m`: the same pipeline on real phone data via MATLAB Mobile (`mobiledev`).

Team: Rayon Biswas, Zaid Faruqui, Tanisha Nevrekar, Piyush Patil, Ananya Rane, Arpit Rajput.
