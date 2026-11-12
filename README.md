# Noise-Robust Path Tracking for an Autonomous Mini-Drone

Classical DSP (no AI/ML) for camera, IMU and altitude signals: DSP course project ("Digital Signal Processing in Today's Era", VJTI) and MathWorks Minidrone Competition India 2026 entry.

**Live demo:** open the GitHub Pages link on a phone and tap *Phone camera* or *Start phone sensors* to run the pipeline on the phone's real camera, accelerometer and gyroscope.

| Path | Chain |
|---|---|
| Vision | Median 3×3 → RGB→HSV threshold → close/open → edges → Hough → low-pass → PID |
| Inertial | FFT peak search → auto notch → low-pass → atan2 roll → complementary / Kalman fusion with gyro |
| Altitude | Median → moving average / 1D Kalman |

- `index.html`: the phone web app. Tabs:
  - **Camera / IMU / Altitude**: each path on its own, with filters you can switch off and sliders that inject noise
  - **Flight**: closed loop, comparing a raw drone and a DSP drone on the same noisy camera
  - **Drone**: camera and IMU together (track tracking plus an artificial horizon showing roll and pitch); hold the phone flat with the back camera facing down
  - **Beacon**: ESP32 navigation. 3 anchors → RSSI → median/Kalman → path-loss distance → trilateration → position Kalman → live map. Simulate, replay a CSV log, or connect a real ESP32 (Bluetooth on phone, USB on laptop). Firmware is in `esp32/`.
  - **Results**: runs every comparison from the proposal with a fixed seed (E1 filter vs filter, E2 accuracy vs noise, E3 FIR vs IIR, notch and fusion, E4 altitude, E5 flight), then Copy tables or Download CSV
  - Falls back to simulated sensors when the real ones are unavailable.
- `matlab/minidrone_dsp_sim.m`: the full simulation in MATLAB (Image Processing and Signal Processing Toolboxes).
- `matlab/phone_sensor_demo.m`: the same pipeline on real phone data via MATLAB Mobile (`mobiledev`).

Team: Rayon Biswas, Zaid Faruqui, Tanisha Nevrekar, Piyush Patil, Ananya Rane, Arpit Rajput.
