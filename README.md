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
  - **Navigate**: the phone locates itself with no extra hardware. Downward camera → Hann window → 2D FFT phase correlation (optical flow) → quality gate → median → low-pass, plus gyro heading with bias calibration and compass fusion. Shows a live map of a 150 × 100 cm arena with an obstacle, an A* route, arrow + voice directions, and raw vs filtered paths. Also has a simulation with ground truth and a 5-run drift test.
  - **Beacon (archived)**: the earlier ESP32 RSSI trilateration idea, kept as a simulation; firmware is in `archive/esp32/`. Open it from the Navigate tab.
  - **Results**: runs every comparison from the proposal with a fixed seed (E1 filter vs filter, E2 accuracy vs noise, E3 FIR vs IIR, notch and fusion, E4 altitude, E5 flight), then Copy tables or Download CSV
  - Falls back to simulated sensors when the real ones are unavailable.
- `PLAN_A.md`: **the demo plan**. Three proofs matching proposal §4: Simulink minidrone, ESP32 + MPU6050 vibration rig, real track + landing pad.
- `hardware/esp32_mpu6050/`: 500 Hz MPU6050 streaming firmware (IMU tab → ESP32 + MPU6050 (USB)).
- `matlab/esp32_mpu6050_analysis.m`: FFT / notch / FIR vs IIR / fusion analysis of rig logs.
- `matlab/simulink/`: MATLAB Function blocks and a guide for the parrotMinidroneCompetition model.
- `matlab/minidrone_dsp_sim.m`: the full simulation in MATLAB (Image Processing and Signal Processing Toolboxes).
- `matlab/phone_sensor_demo.m`: the same pipeline on real phone data via MATLAB Mobile (`mobiledev`).

Team: Rayon Biswas, Zaid Faruqui, Tanisha Nevrekar, Piyush Patil, Ananya Rane, Arpit Rajput.
