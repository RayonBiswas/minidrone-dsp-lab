"""
Python re-implementation of the inertial and altitude chains in
matlab/minidrone_dsp_sim.m (sections 3 and 4), written from the MATLAB source so the two
toolchains can be compared. The hand-rolled filter designs (notchDesign, butter2,
firLowpass, groupDelay) are ported formula-for-formula; scipy is used only as the
independent reference in run_validation.py and the tests.
"""
import numpy as np
from scipy.signal import lfilter

G = 9.81
FS, T = 200, 20          # MATLAB: fs = 200; T = 20


# ---------------- filter designs: same formulas as the MATLAB local functions ----------------
def notch_design(w0, bw, Gb=None):
    """MATLAB notchDesign(w0, bw): w0, bw normalised to Nyquist, -3 dB bandwidth bw.

    Gb is the band-edge gain. MATLAB's code uses exactly -3.000 dB (the default here);
    scipy/MATLAB iirnotch use 1/sqrt(2) = -3.0103 dB. Pass Gb=1/np.sqrt(2) to reproduce
    iirnotch bit-for-bit (the difference is about 3e-4 in the coefficients)."""
    Gb = 10 ** (-3 / 20) if Gb is None else Gb
    beta = np.sqrt(1 - Gb ** 2) / Gb * np.tan(bw * np.pi / 2)
    g = 1 / (1 + beta)
    b = g * np.array([1, -2 * np.cos(w0 * np.pi), 1])
    a = np.array([1, -2 * g * np.cos(w0 * np.pi), 2 * g - 1])
    return b, a


def butter2(wn):
    """MATLAB butter2(wn): 2nd-order Butterworth low-pass via bilinear transform."""
    K = np.tan(np.pi * wn / 2); nrm = 1 / (1 + np.sqrt(2) * K + K ** 2)
    b = K ** 2 * nrm * np.array([1, 2, 1])
    a = np.array([1, 2 * (K ** 2 - 1) * nrm, (1 - np.sqrt(2) * K + K ** 2) * nrm])
    return b, a


def fir_lowpass(N, wn):
    """MATLAB firLowpass(N, wn): windowed-sinc, Hamming, like fir1(N, wn). N+1 taps."""
    n = np.arange(N + 1) - N / 2
    with np.errstate(invalid="ignore", divide="ignore"):
        h = np.sin(np.pi * wn * n) / (np.pi * n)
    h[n == 0] = wn
    h = h * (0.54 - 0.46 * np.cos(2 * np.pi * np.arange(N + 1) / N))
    return h / h.sum()


def group_delay(b, a, f, fs):
    """Group delay in samples at frequencies f (Hz): MATLAB groupDelay / gdPoly."""
    w = 2 * np.pi * np.atleast_1d(f) / fs
    def gd(c):
        k = np.arange(len(c)); z = np.exp(-1j * np.outer(w, k))
        return np.real((z @ (k * c)) / (z @ c))
    return gd(np.asarray(b, float)) - gd(np.asarray(a, float))


# ---------------- inertial path ----------------
def make_imu_signals(rng, fs=FS, T=T):
    t = np.arange(0, T, 1 / fs); n = len(t)
    roll = np.deg2rad(15) * np.sin(2 * np.pi * 0.2 * t) + np.deg2rad(8) * np.sin(2 * np.pi * 0.5 * t + 1)
    rate = np.gradient(roll, 1 / fs)
    gyro = rate + 0.03 + 0.02 * rng.standard_normal(n)             # bias -> drift
    vib = 3 * np.sin(2 * np.pi * 47 * t) + 1.5 * np.sin(2 * np.pi * 94 * t)
    ay = G * np.sin(roll) + vib + 0.5 * rng.standard_normal(n)
    az = G * np.cos(roll) + 0.6 * vib + 0.5 * rng.standard_normal(n)
    return t, roll, gyro, ay, az


def find_two_peaks(x, fs):
    """FFT peak search exactly as in the MATLAB: ignore < 10 Hz, then exclude +-3 Hz around peak 1."""
    n = len(x); f = np.arange(n) * fs / n; half = np.arange(n // 2)
    Y = np.abs(np.fft.fft(x - x.mean()))[half]; f = f[half]
    Ys = Y.copy(); Ys[f < 10] = 0
    f1 = f[np.argmax(Ys)]
    Ys[np.abs(f - f1) < 3] = 0
    f2 = f[np.argmax(Ys)]
    return f1, f2


def kalman_roll(gyro, acc, fs, Q=(1e-5, 1e-7), R=0.003):
    dt = 1 / fs; x = np.zeros(2); P = np.eye(2); out = np.zeros(len(acc))
    F = np.array([[1, -dt], [0, 1]]); B = np.array([dt, 0.0]); H = np.array([1.0, 0.0]); Qm = np.diag(Q)
    for k in range(len(acc)):
        x = F @ x + B * gyro[k]; P = F @ P @ F.T + Qm
        K = P @ H / (H @ P @ H + R)
        x = x + K * (acc[k] - H @ x); P = (np.eye(2) - np.outer(K, H)) @ P
        out[k] = x[0]
    return out


def complementary(gyro, acc, fs, alpha=0.98):
    out = np.zeros(len(acc))
    for k in range(1, len(acc)):
        out[k] = alpha * (out[k - 1] + gyro[k] / fs) + (1 - alpha) * acc[k]
    return out


def rms(x): return float(np.sqrt(np.mean(np.square(x))))


def imu_prefilter(rng, fs=FS, T=T):
    """Signals + notch/low-pass accelerometer roll, shared by run_imu and the Q/R sweep."""
    t, roll, gyro, ay, az = make_imu_signals(rng, fs, T)
    f1, f2 = find_two_peaks(ay, fs)
    bn1, an1 = notch_design(f1 / (fs / 2), (f1 / (fs / 2)) / 10)
    bn2, an2 = notch_design(f2 / (fs / 2), (f2 / (fs / 2)) / 10)
    bl, al = butter2(5 / (fs / 2)); bf = fir_lowpass(60, 5 / (fs / 2))
    def chain(x, b, a=(1.0,)):
        return lfilter(b, a, lfilter(bn2, an2, lfilter(bn1, an1, x)))
    return dict(t=t, roll=roll, gyro=gyro, f1=f1, f2=f2,
                acc_raw=np.arctan2(ay, az),
                acc_f=np.arctan2(chain(ay, bl, al), chain(az, bl, al)),
                acc_fir=np.arctan2(chain(ay, bf), chain(az, bf)))


def run_imu(rng, fs=FS, T=T):
    p = imu_prefilter(rng, fs, T); t, roll, gyro = p["t"], p["roll"], p["gyro"]
    gyro_only = np.cumsum(gyro) / fs
    comp, kal = complementary(gyro, p["acc_f"], fs), kalman_roll(gyro, p["acc_f"], fs)
    ok = t > 1                                   # 1 s warm-up skipped, as in MATLAB
    e = lambda s: np.rad2deg(rms((s - roll)[ok]))
    return dict(f1=p["f1"], f2=p["f2"], acc_raw=e(p["acc_raw"]), acc_filt=e(p["acc_f"]), acc_fir=e(p["acc_fir"]),
                gyro_only=e(gyro_only), comp=e(comp), kalman=e(kal))


# ---------------- altitude path ----------------
def movmedian(x, k):
    """MATLAB movmedian with default 'shrink' endpoints (odd k)."""
    h = k // 2; return np.array([np.median(x[max(0, i - h):i + h + 1]) for i in range(len(x))])


def movmean(x, k):
    h = k // 2; return np.array([x[max(0, i - h):i + h + 1].mean() for i in range(len(x))])


def kalman_alt(z, dt, Q=(1e-6, 1e-4), R=0.03 ** 2):
    x = np.zeros(2); P = np.eye(2); F = np.array([[1, dt], [0, 1]]); H = np.array([1.0, 0.0]); Qm = np.diag(Q)
    out = np.zeros(len(z))
    for k in range(len(z)):
        x = F @ x; P = F @ P @ F.T + Qm
        K = P @ H / (H @ P @ H + R)
        x = x + K * (z[k] - H @ x); P = (np.eye(2) - np.outer(K, H)) @ P
        out[k] = x[0]
    return out


def run_altitude(rng, fs=FS, T=T):
    t = np.arange(0, T, 1 / fs); n = len(t)
    h = np.interp(t, [0, 3, 15, 19, 20], [0, 1.1, 1.1, 0, 0])
    hm = h + 0.03 * rng.standard_normal(n)
    spk = rng.random(n) < 0.02
    hm[spk] += np.sign(rng.standard_normal(spk.sum())) * (0.3 + 0.3 * rng.random(spk.sum()))
    h_med, h_ma = movmedian(hm, 7), movmean(hm, 15)
    h_kal = kalman_alt(h_med, 1 / fs)
    snr = lambda e: 10 * np.log10(np.sum(h ** 2) / np.sum(e ** 2))
    outs = [hm, h_ma, h_med, h_kal]
    return dict(rmse_cm=[100 * rms(o - h) for o in outs], snr_db=[snr(o - h) for o in outs])
