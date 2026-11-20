import numpy as np, pytest
from scipy import signal
import dsp_chain as dc

# Define the sampling frequency
FS = 200


# Test notch filter at different frequencies
@pytest.mark.parametrize("f0", [30.0, 47.0, 94.0])
def test_notch_formula_matches_scipy_iirnotch_exactly(f0):
    """Check whether our notch filter matches SciPy exactly."""
    w0, bw = f0 / (FS / 2), (f0 / (FS / 2)) / 10

    # Design both notch filters and compare their coefficients
    b, a = dc.notch_design(w0, bw, Gb=1 / np.sqrt(2))
    bs, as_ = signal.iirnotch(f0, 10.0, fs=FS)

    # Check whether both filters have almost identical coefficients
    assert np.allclose(b, bs, atol=1e-12) and np.allclose(a, as_, atol=1e-12)


# Check the small difference between MATLAB and SciPy notch filters
@pytest.mark.parametrize("f0", [30.0, 47.0, 94.0])
def test_matlab_notch_differs_from_iirnotch_only_negligibly(f0):
    """Check that the difference between both notch filters is very small."""
    w0, bw = f0 / (FS / 2), (f0 / (FS / 2)) / 10

    # Calculate coefficients using both methods
    b, a = dc.notch_design(w0, bw)
    bs, as_ = signal.iirnotch(f0, 10.0, fs=FS)

    # Find the maximum difference between coefficients
    d = max(np.abs(b - bs).max(), np.abs(a - as_).max())

    # Check that the difference is positive but very small
    assert 0 < d < 1e-3


# Check the main properties of the notch filter
def test_notch_has_zero_at_centre_unit_dc_gain_and_requested_bandwidth():

    # Design notch filter for 47 Hz
    f0 = 47.0
    b, a = dc.notch_design(f0 / 100, f0 / 10 / 100)

    # Calculate the frequency response of the filter
    f = np.linspace(20, 80, 60001)
    _, h = signal.freqz(b, a, worN=f, fs=FS)
    m = np.abs(h)

    # Check attenuation at centre frequency and gain at DC
    assert m[np.argmin(np.abs(f - f0))] < 1e-6 and abs(np.abs(signal.freqz(b, a, [0.0], fs=FS)[1][0]) - 1) < 0.02

    # Check whether the filter has the requested bandwidth
    stop = f[m < 10 ** (-3 / 20)]
    assert abs((stop.max() - stop.min()) - f0 / 10) < 0.1 * f0 / 10


# Check whether our Butterworth filter matches SciPy
def test_butter2_matches_scipy_butter():

    # Design Butterworth filters using both methods
    b, a = dc.butter2(5 / (FS / 2))
    bs, as_ = signal.butter(2, 5 / (FS / 2))

    # Compare the filter coefficients
    assert np.allclose(b, bs, atol=1e-12) and np.allclose(a, as_, atol=1e-12)


# Check whether FIR filter matches SciPy's Hamming window design
def test_fir_matches_scipy_firwin_hamming():

    # Design FIR filters using both methods
    h = dc.fir_lowpass(60, 5 / (FS / 2))
    hs = signal.firwin(61, 5 / (FS / 2), window="hamming")

    # Compare coefficients and check that the filter gain is one
    assert np.allclose(h, hs, atol=1e-12) and abs(h.sum() - 1) < 1e-12


# Check the group delay of the FIR filter
def test_fir_group_delay_is_half_the_order():

    # Design FIR filter with order 60
    h = dc.fir_lowpass(60, 5 / (FS / 2))

    # FIR group delay should be half the filter order
    assert np.allclose(dc.group_delay(h, [1.0], [1, 2], FS), 30.0, atol=1e-6)   # 150 ms at 200 Hz


# Compare our group delay calculation with SciPy
def test_group_delay_matches_scipy():

    # Design Butterworth filter
    b, a = dc.butter2(5 / (FS / 2))

    # Calculate group delay using SciPy
    _, gd = signal.group_delay((b, a), w=[1, 2], fs=FS)

    # Check whether both group delay values match
    assert np.allclose(dc.group_delay(b, a, [1, 2], FS), gd, atol=1e-6)


# Check whether FFT detects the correct vibration frequencies
def test_fft_peaks_found():

    # Generate IMU signals
    t, roll, gyro, ay, az = dc.make_imu_signals(np.random.default_rng(0))

    # Find the two main frequency peaks
    f1, f2 = dc.find_two_peaks(ay, FS)

    # Check whether detected frequencies are close to actual frequencies
    assert abs(f1 - 47) < 0.2 and abs(f2 - 94) < 0.2


# Check moving median filter behaviour at signal edges
def test_movmedian_shrinks_at_edges():

    # Create sample input signal
    x = np.array([10.0, 1, 2, 3, 4, 5, 6, 7, 8])

    # Apply moving median filter with window size 7
    y = dc.movmedian(x, 7)

    # Check median calculation at the beginning and middle
    assert y[0] == np.median(x[:4]) and y[4] == np.median(x[1:8])


# Check whether sensor fusion performs better than individual sensors
def test_fusion_beats_raw_and_gyro_only():

    # Run IMU simulation
    r = dc.run_imu(np.random.default_rng(1))

    # Check whether Kalman and complementary filters reduce error
    assert r["kalman"] < r["acc_raw"] / 2 and r["comp"] < r["gyro_only"]


# Check whether Kalman filtering improves altitude estimation
def test_altitude_kalman_improves_snr():

    # Run altitude simulation
    r = dc.run_altitude(np.random.default_rng(1))

    # Check whether Kalman filtering improves SNR and reduces RMSE
    assert r["snr_db"][3] > r["snr_db"][0] + 5 and r["rmse_cm"][3] < r["rmse_cm"][0]
