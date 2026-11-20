# Independent validation of the inertial and altitude chains

A Python re-implementation of sections 3 (inertial) and 4 (altitude) of `matlab/minidrone_dsp_sim.m`,
written from the MATLAB source, plus scipy as an independent reference. Nothing in the app or the MATLAB
code is changed. The goal is to check the project's claims with a second toolchain and to show whether
the single MATLAB run (`rng(1)`) is representative.

## Run
```bash
cd validation/imu_altitude
pip install -r requirements.txt
pytest -q                      # 15 tests
python run_validation.py       # ~2 min, writes results/  (use --seeds 20 for a quick run)
```

## 1. Filter designs vs scipy
The MATLAB file hand-codes `notchDesign`, `butter2`, `firLowpass` and `groupDelay`. Ported formula for
formula, they match scipy's reference designs:

| Design | max abs coefficient difference |
|---|---|
| Butterworth 2nd order, 5 Hz (`butter2` vs `signal.butter`) | 2e-16 |
| FIR 61 taps Hamming, 5 Hz (`firLowpass` vs `signal.firwin`) | 1e-17 |
| Notch (`notchDesign` vs `signal.iirnotch`) with Gb = 1/√2 | 0 (identical) |
| Notch as written in the MATLAB (Gb = −3.000 dB) | 3e-4 at 47 Hz, 5e-4 at 94 Hz |

**Small finding:** the MATLAB comment says `notchDesign` is the "same design as iirnotch". The formula is,
but it uses a band-edge gain of exactly −3.000 dB, whereas `iirnotch` uses 1/√2 (−3.0103 dB). The effect is
about 3e-4 in the coefficients and is negligible in practice; the comment could say "nearly".
Group delay at 1 Hz: IIR Butterworth 46.7 ms, FIR (61 taps) 150.0 ms (exactly half the order).

## 2. Roll estimation, 200 seeds (mean ± std, degrees RMSE)
| Method | RMSE |
|---|---|
| Accelerometer raw | 14.20 ± 0.05 |
| Accelerometer, notch + IIR low-pass | 1.24 ± 0.04 |
| Accelerometer, notch + FIR low-pass | 3.40 ± 0.05 |
| Gyro only (drifts) | 14.78 ± 0.22 |
| Complementary (α = 0.98) | 1.01 ± 0.04 |
| Kalman (MATLAB's Q, R) | 1.11 ± 0.05 |

* The FFT peak search found both vibration peaks (47 and 94 Hz) within 0.2 Hz in 200/200 runs.
* FIR is worse than IIR in 200/200 runs, because its 150 ms delay is large against the 0.2 and 0.5 Hz motion.
* Run-to-run variation is small (std ≈ 0.05°), so a single seeded MATLAB run is representative. To check
  yours, compare the line MATLAB prints (`Roll RMSE: ...`) with the 5th–95th percentile ranges in
  `results/imu_montecarlo.csv`. I could not run MATLAB while writing this, so that comparison is left to you.

## 3. Kalman Q/R sweep (`results/kalman_sweep.png`)
With MATLAB's Q = 1e-5, R = 3e-3 the Kalman is slightly *worse* than the complementary filter in every
run (1.11° vs 1.01°). A 6 × 6 sweep shows it is a tuning matter: the best region is small process noise
(Q ≈ 1e-8 to 1e-6) with a large R (≈ 3), giving about **0.43°**, a 2.5× reduction. Error rises again at
R = 30, so the optimum is inside the grid.

**Caveat:** the simulated gyro has a constant bias and white noise, which favours trusting the gyro (large R).
A real MEMS gyro's bias drifts, which pushes the best R lower, so treat 0.43° as a bound for this synthetic
world, not a hardware promise. The sweep is the method to re-run on a real recording.

## 4. Altitude, 200 seeds (mean)
| | Raw | Mov avg 15 | Median 7 | Median + Kalman |
|---|---|---|---|---|
| RMSE (cm) | 7.17 | 1.85 | 1.41 | 0.76 |
| SNR (dB) | 22.3 | 34.0 | 36.4 | 41.8 |

The ordering matches the MATLAB design intent: median removes the spikes, Kalman smooths what is left.

## Limits
* Signals are the MATLAB script's synthetic ones, re-generated with numpy (a different random stream from
  MATLAB's), so agreement is statistical, not sample-for-sample.
* No real sensor data is used. Recording an MPU6050 log and running these chains on it is the natural next step.
