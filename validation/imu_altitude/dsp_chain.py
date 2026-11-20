import numpy as np
from scipy.signal import lfilter

# Gravitational acceleration and simulation settings
G = 9.81
FS, T = 200, 20          # MATLAB: fs = 200; T = 20


# ---------------- Filter designs ----------------

# Design a notch filter to remove unwanted frequencies
def notch_design(w0, bw, Gb=None):
    """MATLAB notchDesign(w0, bw): w0, bw normalised to Nyquist, -3 dB bandwidth bw.

    Gb is the band-edge gain. MATLAB's code uses exactly -3.000 dB (the default here);
    scipy/MATLAB iirnotch use 1/sqrt(2) = -3.0103 dB. Pass Gb=1/np.sqrt(2) to reproduce
    iirnotch bit-for-bit (the difference is about 3e-4 in the coefficients)."""

    # Set the gain at the bandwidth edges
    Gb = 10 ** (-3 / 20) if Gb is None else Gb

    # Calculate filter coefficients
    beta = np.sqrt(1 - Gb ** 2) / Gb * np.tan(bw * np.pi / 2)
    g = 1 / (1 + beta)
    b = g * np.array([1, -2 * np.cos(w0 * np.pi), 1])
    a = np.array([1, -2 * g * np.cos(w0 * np.pi), 2 * g - 1])

    return b, a


# Design a second-order Butterworth low-pass filter
def butter2(wn):
    """MATLAB butter2(wn): 2nd-order Butterworth low-pass via bilinear transform."""

    # Calculate the filter coefficients using bilinear transformation
    K = np.tan(np.pi * wn / 2)
    nrm = 1 / (1 + np.sqrt(2) * K + K ** 2)

    b = K ** 2 * nrm * np.array([1, 2, 1])
    a = np.array([1, 2 * (K ** 2 - 1) * nrm, (1 - np.sqrt(2) * K + K ** 2) * nrm])

    return b, a


# Design an FIR low-pass filter using a Hamming window
def fir_lowpass(N, wn):
    """MATLAB firLowpass(N, wn): windowed-sinc, Hamming, like fir1(N, wn). N+1 taps."""

    # Generate the filter sample positions
    n = np.arange(N + 1) - N / 2

    # Calculate the sinc filter coefficients
    with np.errstate(invalid="ignore", divide="ignore"):
        h = np.sin(np.pi * wn * n) / (np.pi * n)

    # Set the centre coefficient separately
    h[n == 0] = wn

    # Apply the Hamming window
    h = h * (0.54 - 0.46 * np.cos(2 * np.pi * np.arange(N + 1) / N))

    # Normalize the filter coefficients
    return h / h.sum() #I like Op amp


# Calculate the group delay of a filter
def group_delay(b, a, f, fs):
    """Group delay in samples at frequencies f (Hz): MATLAB groupDelay / gdPoly."""

    # Convert frequency from Hz to radians per sample
    w = 2 * np.pi * np.atleast_1d(f) / fs

    # Calculate group delay for the given coefficients
    def gd(c):
        k = np.arange(len(c))
        z = np.exp(-1j * np.outer(w, k))
        return np.real((z @ (k * c)) / (z @ c))

    # Subtract denominator delay from numerator delay
    return gd(np.asarray(b, float)) - gd(np.asarray(a, float))


# ---------------- Inertial path ----------------

# Generate simulated accelerometer and gyroscope signals
def make_imu_signals(rng, fs=FS, T=T):

    # Generate the time values
    t = np.arange(0, T, 1 / fs)
    n = len(t)

    # Generate the actual roll angle of the drone
    roll = np.deg2rad(15) * np.sin(2 * np.pi * 0.2 * t) + np.deg2rad(8) * np.sin(2 * np.pi * 0.5 * t + 1)

    # Calculate the angular velocity from the roll angle
    rate = np.gradient(roll, 1 / fs)

    # Add bias and random noise to the gyroscope signal
    gyro = rate + 0.03 + 0.02 * rng.standard_normal(n)             # bias -> drift

    # Generate vibration at 47 Hz and 94 Hz
    vib = 3 * np.sin(2 * np.pi * 47 * t) + 1.5 * np.sin(2 * np.pi * 94 * t)

    # Generate accelerometer signals with vibration and noise
    ay = G * np.sin(roll) + vib + 0.5 * rng.standard_normal(n)
    az = G * np.cos(roll) + 0.6 * vib + 0.5 * rng.standard_normal(n)

    return t, roll, gyro, ay, az


# Find the two main vibration frequencies using FFT
def find_two_peaks(x, fs):
    """FFT peak search exactly as in the MATLAB: ignore < 10 Hz, then exclude +-3 Hz around peak 1."""

    # Calculate FFT frequency values and spectrum
    n = len(x)
    f = np.arange(n) * fs / n
    half = np.arange(n // 2)

    Y = np.abs(np.fft.fft(x - x.mean()))[half]
    f = f[half]

    # Ignore frequencies below 10 Hz
    Ys = Y.copy()
    Ys[f < 10] = 0

    # Find the first major frequency peak
    f1 = f[np.argmax(Ys)]

    # Ignore frequencies within 3 Hz of the first peak
    Ys[np.abs(f - f1) < 3] = 0

    # Find the second major frequency peak
    f2 = f[np.argmax(Ys)]

    return f1, f2


# Estimate the roll angle using a Kalman filter
def kalman_roll(gyro, acc, fs, Q=(1e-5, 1e-7), R=0.003):

    # Initialize time step, state, and error covariance
    dt = 1 / fs
    x = np.zeros(2)
    P = np.eye(2)
    out = np.zeros(len(acc))

    # Define state transition, input, and measurement matrices
    F = np.array([[1, -dt], [0, 1]])
    B = np.array([dt, 0.0])
    H = np.array([1.0, 0.0])
    Qm = np.diag(Q)

    # Process each sensor sample
    for k in range(len(acc)):

        # Predict the next state using gyroscope data
        x = F @ x + B * gyro[k]
        P = F @ P @ F.T + Qm

        # Calculate Kalman gain
        K = P @ H / (H @ P @ H + R)

        # Correct the prediction using accelerometer data
        x = x + K * (acc[k] - H @ x)
        P = (np.eye(2) - np.outer(K, H)) @ P

        # Store the estimated roll angle
        out[k] = x[0]

    return out


# Combine gyroscope and accelerometer using a complementary filter
def complementary(gyro, acc, fs, alpha=0.98):

    # Initialize output array
    out = np.zeros(len(acc))

    # Combine gyro prediction and accelerometer measurement
    for k in range(1, len(acc)):
        out[k] = alpha * (out[k - 1] + gyro[k] / fs) + (1 - alpha) * acc[k]

    return out


# Calculate root mean square error
def rms(x):
    return float(np.sqrt(np.mean(np.square(x))))


# Apply filters to the IMU accelerometer signals
def imu_prefilter(rng, fs=FS, T=T):
    """Signals + notch/low-pass accelerometer roll, shared by run_imu and the Q/R sweep."""

    # Generate IMU signals
    t, roll, gyro, ay, az = make_imu_signals(rng, fs, T)

    # Detect the main vibration frequencies
    f1, f2 = find_two_peaks(ay, fs)

    # Design notch filters for both vibration frequencies
    bn1, an1 = notch_design(f1 / (fs / 2), (f1 / (fs / 2)) / 10)
    bn2, an2 = notch_design(f2 / (fs / 2), (f2 / (fs / 2)) / 10)

    # Design Butterworth and FIR low-pass filters
    bl, al = butter2(5 / (fs / 2))
    bf = fir_lowpass(60, 5 / (fs / 2))

    # Apply two notch filters followed by the selected low-pass filter
    def chain(x, b, a=(1.0,)):
        return lfilter(b, a, lfilter(bn2, an2, lfilter(bn1, an1, x)))

    # Calculate roll angle using raw, IIR-filtered and FIR-filtered signals
    return dict(t=t, roll=roll, gyro=gyro, f1=f1, f2=f2,
                acc_raw=np.arctan2(ay, az),
                acc_f=np.arctan2(chain(ay, bl, al), chain(az, bl, al)),
                acc_fir=np.arctan2(chain(ay, bf), chain(az, bf)))


# Run the complete IMU simulation and compare different methods
def run_imu(rng, fs=FS, T=T):

    # Get the filtered sensor signals
    p = imu_prefilter(rng, fs, T)
    t, roll, gyro = p["t"], p["roll"], p["gyro"]

    # Estimate roll using only gyroscope data
    gyro_only = np.cumsum(gyro) / fs

    # Estimate roll using complementary and Kalman filters
    comp, kal = complementary(gyro, p["acc_f"], fs), kalman_roll(gyro, p["acc_f"], fs)

    # Skip the first second to avoid initial filter effects
    ok = t > 1                                   # 1 s warm-up skipped, as in MATLAB

    # Calculate roll estimation error in degrees
    e = lambda s: np.rad2deg(rms((s - roll)[ok]))

    # Return the error of each estimation method
    return dict(f1=p["f1"], f2=p["f2"], acc_raw=e(p["acc_raw"]), acc_filt=e(p["acc_f"]), acc_fir=e(p["acc_fir"]),
                gyro_only=e(gyro_only), comp=e(comp), kalman=e(kal))


# ---------------- Altitude path ----------------

# Apply a moving median filter to remove sudden spikes
def movmedian(x, k):
    """MATLAB movmedian with default 'shrink' endpoints (odd k)."""

    # Calculate the median using a moving window
    h = k // 2
    return np.array([np.median(x[max(0, i - h):i + h + 1]) for i in range(len(x))])


# Apply a moving average filter to smooth the signal
def movmean(x, k):

    # Calculate the average within a moving window
    h = k // 2
    return np.array([x[max(0, i - h):i + h + 1].mean() for i in range(len(x))])


# Estimate altitude using a Kalman filter
def kalman_alt(z, dt, Q=(1e-6, 1e-4), R=0.03 ** 2):

    # Initialize altitude, velocity and error covariance
    x = np.zeros(2)
    P = np.eye(2)

    # Define state transition and measurement matrices
    F = np.array([[1, dt], [0, 1]])
    H = np.array([1.0, 0.0])
    Qm = np.diag(Q)

    # Initialize output array
    out = np.zeros(len(z))

    # Process each altitude measurement
    for k in range(len(z)):

        # Predict altitude and velocity
        x = F @ x
        P = F @ P @ F.T + Qm

        # Calculate Kalman gain
        K = P @ H / (H @ P @ H + R)

        # Correct the predicted altitude using sensor measurement
        x = x + K * (z[k] - H @ x)
        P = (np.eye(2) - np.outer(K, H)) @ P

        # Store estimated altitude
        out[k] = x[0]

    return out


# Simulate altitude measurements and compare filtering methods
def run_altitude(rng, fs=FS, T=T):

    # Generate time values
    t = np.arange(0, T, 1 / fs)
    n = len(t)

    # Generate the actual drone altitude over time
    h = np.interp(t, [0, 3, 15, 19, 20], [0, 1.1, 1.1, 0, 0])

    # Add random measurement noise
    hm = h + 0.03 * rng.standard_normal(n)

    # Generate random spikes in the altitude measurements
    spk = rng.random(n) < 0.02
    hm[spk] += np.sign(rng.standard_normal(spk.sum())) * (0.3 + 0.3 * rng.random(spk.sum()))

    # Apply median, moving average and Kalman filters
    h_med, h_ma = movmedian(hm, 7), movmean(hm, 15)
    h_kal = kalman_alt(h_med, 1 / fs)

    # Calculate signal-to-noise ratio
    snr = lambda e: 10 * np.log10(np.sum(h ** 2) / np.sum(e ** 2))

    # Store the output of all four methods
    outs = [hm, h_ma, h_med, h_kal]

    # Calculate RMSE in centimetres and SNR in decibels
    return dict(rmse_cm=[100 * rms(o - h) for o in outs], snr_db=[snr(o - h) for o in outs])
