# Plan A: prove exactly what the proposal promised

Proposal §1: *a mini-drone follows a coloured track with its downward camera and onboard sensors, then lands on a marker; every signal is noisy.*
Proposal §4: Simulink minidrone test platform · noise injected on purpose · metrics · comparisons · optional ESP32 + MPU6050 on a vibrating motor.

## The three proofs

### P1. Simulink minidrone (the official test platform)
- Files: `matlab/simulink/` (vision block, noise block, steering low-pass, step-by-step guide).
- Output: a table of track completion and landing error for raw vs DSP at 3 noise levels, plus screenshots.

### P2. Real sensor data: ESP32 + MPU6050 on a vibrating motor
- **Build (required):** ESP32 (have), MPU6050 module (~₹150), a small DC motor (130-type, ~₹30) with a blob of hot glue or clay on the shaft as an unbalanced weight, a 2×AA holder with switch (~₹40), and a piece of foam board or cardboard (~₹30).
- **Wiring:** MPU6050 VCC→3V3, GND→GND, SDA→GPIO21, SCL→GPIO22. The motor runs from its own battery, not connected to the ESP32. Glue the motor and the MPU6050 on the same board.
- **Firmware:** `hardware/esp32_mpu6050/esp32_mpu6050.ino` streams at 500 Hz with the sensor's internal filter off, so our filters do the work.
- **Live:** laptop Chrome → app → IMU tab → **ESP32 + MPU6050 (USB)**. The FFT shows the real motor peak and the notch locks onto it.
- **Record:** **Record 20 s CSV**, then run `matlab/esp32_mpu6050_analysis.m`.
- **Protocol (motor ON throughout):** board flat for 5 s → tilt to 30° on a protractor for 5 s → flat for 5 s → repeat with the motor off for a clean reference.
- **Why not the phone IMU here:** the phone samples at 60–100 Hz, so a 50–150 Hz motor vibration aliases. The MPU6050 at 500 Hz captures it properly (Nyquist).

### P3. Real track + landing marker (camera path)
- **Arena:** red electrical tape (~₹30) as a 2–3 m track with one curve, ending at a blue paper circle (~15 cm) as the landing pad.
- **Demo:** phone flat, back camera down, carried along the track. The Camera tab follows the line (θ, d, steering bar), then shows **LAND ✓** when the pad is confirmed by the 6-of-8-frame vote.
- **Disturbances (real):** shadow with your hand, a torch for glare, fast motion for blur. Switch Median, Morphology and Vote off and on to show the difference.
- **Stretch:** mount the phone on the Autoport chassis for a closed loop. Optional, only if P1–P3 are done.

## Metrics to put on slides
| Proposal metric | Where it comes from |
|---|---|
| Detection accuracy % | Results E1 / E2, Simulink runs |
| SNR improvement dB | Results E4 (altitude), P2 analysis (roll noise dB) |
| RMSE of angle and height | Results E1 / E3 / E4, P2 at the 30° hold |
| Tracking error | Results E5, Simulink logged x, y |
| Processing time per frame | Camera tab ms/frame, Results E1 |
| FIR vs IIR phase delay | IMU tab readout, Results E3, P2 analysis |
| Gyro / accel / complementary / Kalman | Results E3, P2 analysis |

## Team split (6)
1. **Simulink A:** set up the project, wire the vision block and noise block.
2. **Simulink B:** steering low-pass + landing logic in path planning, run the 5-run table, screenshots.
3. **Hardware rig:** build the MPU6050 + motor board, flash the firmware, live demo, record logs.
4. **Rig analysis:** run `esp32_mpu6050_analysis.m`, make the FFT / roll / fusion plots and the numbers.
5. **Track demo + video:** tape arena, landing pad, live phone runs, screen recording (scrcpy).
6. **Report + slides + viva:** problem → chain → P1/P2/P3 results → limitations; viva sheet from CLAUDE.md.

## Timeline (~1 week)
- **Day 1:** buy parts, tape the arena, Simulink project opens.
- **Day 2–3:** rig streaming live; vision block running in Simulink.
- **Day 4:** recorded logs + MATLAB plots; Simulink 5-run table.
- **Day 5:** track demo recorded; slides drafted.
- **Day 6:** rehearsal, fix gaps.
