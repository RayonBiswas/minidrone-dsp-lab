"""
Independent validation of the inertial + altitude chains in matlab/minidrone_dsp_sim.m.

  1. Filter designs vs scipy reference (notch, Butterworth, FIR, group delay)
  2. Monte Carlo over many seeds: is the single MATLAB rng(1) run representative?
  3. Altitude chain: RMSE / SNR distribution

Outputs go to results/. Run:  python run_validation.py [--seeds 200]
"""
import argparse, csv, os
import numpy as np
from scipy import signal
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import dsp_chain as dc

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "results")
FS = dc.FS


def design_check():
    rows = []
    for f0 in (47.0, 94.0):
        w0, bw = f0 / (FS / 2), f0 / (FS / 2) / 10
        for lab, Gb in (("MATLAB (-3.000 dB)", None), ("Gb=1/sqrt2", 1 / np.sqrt(2))):
            b, a = dc.notch_design(w0, bw, Gb); bs, as_ = signal.iirnotch(f0, 10, fs=FS)
            rows.append((f"notch {f0:.0f} Hz, {lab}", max(np.abs(b - bs).max(), np.abs(a - as_).max())))
    b, a = dc.butter2(5 / (FS / 2)); bs, as_ = signal.butter(2, 5 / (FS / 2))
    rows.append(("Butterworth 2nd order, 5 Hz", max(np.abs(b - bs).max(), np.abs(a - as_).max())))
    h = dc.fir_lowpass(60, 5 / (FS / 2)); rows.append(("FIR 61 taps Hamming, 5 Hz", np.abs(h - signal.firwin(61, 5 / (FS / 2), window="hamming")).max()))
    gdi = dc.group_delay(*dc.butter2(5 / (FS / 2)), [1.0], FS)[0] / FS * 1000
    gdf = dc.group_delay(h, [1.0], [1.0], FS)[0] / FS * 1000
    with open(os.path.join(OUT, "design_check.csv"), "w", newline="") as f:
        w = csv.writer(f); w.writerow(["design", "max_abs_coeff_diff_vs_scipy"]); w.writerows([(n, f"{d:.2e}") for n, d in rows])
    return rows, gdi, gdf


def pct(x): return np.percentile(x, [5, 50, 95])


def kalman_sweep(seeds=10):
    """Is the Kalman's small deficit vs the complementary filter just un-tuned Q/R?"""
    Qa = [1e-8, 1e-7, 1e-6, 1e-5, 1e-4, 1e-3]; Rs = [3e-4, 3e-3, 3e-2, 3e-1, 3.0, 30.0]
    pre = [dc.imu_prefilter(np.random.default_rng(9000 + s)) for s in range(seeds)]
    ok = pre[0]["t"] > 1; M = np.zeros((len(Qa), len(Rs)))
    comp = np.mean([np.rad2deg(dc.rms((dc.complementary(p["gyro"], p["acc_f"], dc.FS) - p["roll"])[ok])) for p in pre])
    for i, q in enumerate(Qa):
        for j, r_ in enumerate(Rs):
            M[i, j] = np.mean([np.rad2deg(dc.rms((dc.kalman_roll(p["gyro"], p["acc_f"], dc.FS, (q, 1e-7), r_) - p["roll"])[ok])) for p in pre])
    with open(os.path.join(OUT, "kalman_sweep.csv"), "w", newline="") as f:
        w = csv.writer(f); w.writerow(["Q_angle \\ R"] + Rs)
        for q, row in zip(Qa, M): w.writerow([q] + [f"{v:.3f}" for v in row])
    fig, ax = plt.subplots(figsize=(6, 4.4)); im = ax.imshow(M, cmap="viridis_r", origin="lower")
    ax.set_xticks(range(len(Rs))); ax.set_xticklabels([f"{r_:g}" for r_ in Rs]); ax.set_yticks(range(len(Qa))); ax.set_yticklabels([f"{q:g}" for q in Qa])
    ax.set_xlabel("R (measurement, rad²)"); ax.set_ylabel("Q angle (process)")
    for i in range(len(Qa)):
        for j in range(len(Rs)): ax.text(j, i, f"{M[i,j]:.2f}", ha="center", va="center", color="w", fontsize=8)
    ax.set_title(f"Kalman roll RMSE (deg), {seeds} seeds; complementary = {comp:.2f}"); fig.colorbar(im); fig.tight_layout()
    fig.savefig(os.path.join(OUT, "kalman_sweep.png"), dpi=140); plt.close(fig)
    return Qa, Rs, M, comp


def main(seeds):
    os.makedirs(OUT, exist_ok=True)
    rows, gdi, gdf = design_check()
    print("1. Filter designs vs scipy (max |coefficient difference|)")
    for n, d in rows: print(f"   {n:38s} {d:.2e}")
    print(f"   Group delay at 1 Hz: IIR Butterworth {gdi:.1f} ms, FIR(61 taps) {gdf:.1f} ms")

    keys = ["acc_raw", "acc_filt", "acc_fir", "gyro_only", "comp", "kalman"]
    labels = ["Acc raw", "Acc IIR", "Acc FIR", "Gyro only", "Complementary", "Kalman"]
    imu = {k: [] for k in keys}; peaks = []
    for s in range(seeds):
        r = dc.run_imu(np.random.default_rng(1000 + s)); peaks.append((r["f1"], r["f2"]))
        for k in keys: imu[k].append(r[k])
    print(f"\n2. Roll RMSE (deg) over {seeds} seeds:  mean +- std   [5th, median, 95th percentile]")
    with open(os.path.join(OUT, "imu_montecarlo.csv"), "w", newline="") as f:
        w = csv.writer(f); w.writerow(["method", "mean_deg", "std_deg", "p5", "median", "p95"])
        for k, l in zip(keys, labels):
            v = np.array(imu[k]); p = pct(v); w.writerow([l, f"{v.mean():.3f}", f"{v.std():.3f}", *[f"{x:.3f}" for x in p]])
            print(f"   {l:14s} {v.mean():6.2f} +- {v.std():5.2f}   [{p[0]:.2f}, {p[1]:.2f}, {p[2]:.2f}]")
    pk = np.array(peaks); print(f"   FFT peaks: {np.mean(pk[:,0]):.2f} / {np.mean(pk[:,1]):.2f} Hz (truth 47 / 94); "
          f"found within 0.2 Hz in {np.mean((abs(pk[:,0]-47)<.2)&(abs(pk[:,1]-94)<.2))*100:.0f}% of runs")
    print(f"   Kalman better than complementary in {np.mean(np.array(imu['kalman'])<np.array(imu['comp']))*100:.0f}% of runs; "
          f"FIR worse than IIR in {np.mean(np.array(imu['acc_fir'])>np.array(imu['acc_filt']))*100:.0f}%")

    alt = {"rmse": [], "snr": []}
    for s in range(seeds):
        r = dc.run_altitude(np.random.default_rng(5000 + s)); alt["rmse"].append(r["rmse_cm"]); alt["snr"].append(r["snr_db"])
    R, S = np.array(alt["rmse"]), np.array(alt["snr"]); names = ["Raw", "Mov avg 15", "Median 7", "Median+Kalman"]
    print(f"\n3. Altitude over {seeds} seeds (mean):")
    print("   " + " " * 10 + "".join(f"{n:>15s}" for n in names))
    print("   RMSE (cm) " + "".join(f"{v:15.2f}" for v in R.mean(0))); print("   SNR (dB)  " + "".join(f"{v:15.1f}" for v in S.mean(0)))
    with open(os.path.join(OUT, "altitude_montecarlo.csv"), "w", newline="") as f:
        w = csv.writer(f); w.writerow(["stage", "rmse_cm_mean", "rmse_cm_std", "snr_db_mean", "snr_db_std"])
        for i, n in enumerate(names): w.writerow([n, f"{R[:,i].mean():.3f}", f"{R[:,i].std():.3f}", f"{S[:,i].mean():.2f}", f"{S[:,i].std():.2f}"])

    # --- figures ---
    fig, axs = plt.subplots(1, 2, figsize=(11, 4))
    m = [np.mean(imu[k]) for k in keys]; sd = [np.std(imu[k]) for k in keys]
    axs[0].bar(labels, m, yerr=sd, capsize=4, color="#4f8cff"); axs[0].set_ylabel("roll RMSE (deg)"); axs[0].set_title(f"Roll error, {seeds} seeds (mean ± std)")
    axs[0].tick_params(axis="x", rotation=25); axs[0].grid(alpha=.3, axis="y")
    axs[1].bar(names, S.mean(0), yerr=S.std(0), capsize=4, color="#2fbf71"); axs[1].set_ylabel("SNR (dB)"); axs[1].set_title("Altitude SNR (mean ± std)")
    axs[1].tick_params(axis="x", rotation=25); axs[1].grid(alpha=.3, axis="y")
    fig.tight_layout(); fig.savefig(os.path.join(OUT, "montecarlo.png"), dpi=140); plt.close(fig)

    t, roll, gyro, ay, az = dc.make_imu_signals(np.random.default_rng(1)); f1, f2 = dc.find_two_peaks(ay, FS)
    bn1, an1 = dc.notch_design(f1 / 100, f1 / 100 / 10); bn2, an2 = dc.notch_design(f2 / 100, f2 / 100 / 10); bl, al = dc.butter2(5 / 100)
    ayf = signal.lfilter(bl, al, signal.lfilter(bn2, an2, signal.lfilter(bn1, an1, ay)))
    n = len(ay); f = np.arange(n // 2) * FS / n
    Ya = np.abs(np.fft.fft(ay - ay.mean()))[:n // 2] / n; Yf = np.abs(np.fft.fft(ayf - ayf.mean()))[:n // 2] / n
    fig, ax = plt.subplots(figsize=(7, 4)); ax.plot(f, 20 * np.log10(Ya + 1e-6), "r", label="raw"); ax.plot(f, 20 * np.log10(Yf + 1e-6), "b", label="notch + LPF")
    for x in (f1, f2): ax.axvline(x, ls="--", c="k", lw=.8)
    ax.set_xlabel("Frequency (Hz)"); ax.set_ylabel("dB"); ax.set_title(f"Accelerometer spectrum, peaks {f1:.1f} / {f2:.1f} Hz"); ax.grid(alpha=.3); ax.legend()
    fig.tight_layout(); fig.savefig(os.path.join(OUT, "spectrum.png"), dpi=140); plt.close(fig)
    Qa, Rs, M, comp = kalman_sweep(10 if seeds >= 50 else 3)
    i, j = np.unravel_index(np.argmin(M), M.shape)
    print(f"\n4. Kalman Q/R sweep (MATLAB uses Q=1e-5, R=3e-3 -> {M[3,1]:.2f} deg; complementary {comp:.2f} deg)")
    print(f"   best grid cell: Q={Qa[i]:g}, R={Rs[j]:g} -> {M[i,j]:.2f} deg")
    print("\nSaved to", OUT)


if __name__ == "__main__":
    ap = argparse.ArgumentParser(); ap.add_argument("--seeds", type=int, default=200); main(ap.parse_args().seeds)
