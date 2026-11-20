import numpy as np, pytest
from scipy import signal
import dsp_chain as dc

FS = 200


@pytest.mark.parametrize("f0", [30.0, 47.0, 94.0])
def test_notch_formula_matches_scipy_iirnotch_exactly(f0):
    """Same formula as iirnotch: identical to machine precision when Gb = 1/sqrt(2)."""
    w0, bw = f0 / (FS / 2), (f0 / (FS / 2)) / 10
    b, a = dc.notch_design(w0, bw, Gb=1 / np.sqrt(2)); bs, as_ = signal.iirnotch(f0, 10.0, fs=FS)
    assert np.allclose(b, bs, atol=1e-12) and np.allclose(a, as_, atol=1e-12)


@pytest.mark.parametrize("f0", [30.0, 47.0, 94.0])
def test_matlab_notch_differs_from_iirnotch_only_negligibly(f0):
    """MATLAB uses -3.000 dB, iirnotch -3.0103 dB: documented, tiny (< 1e-3) difference."""
    w0, bw = f0 / (FS / 2), (f0 / (FS / 2)) / 10
    b, a = dc.notch_design(w0, bw); bs, as_ = signal.iirnotch(f0, 10.0, fs=FS)
    d = max(np.abs(b - bs).max(), np.abs(a - as_).max())
    assert 0 < d < 1e-3


def test_notch_has_zero_at_centre_unit_dc_gain_and_requested_bandwidth():
    f0 = 47.0; b, a = dc.notch_design(f0 / 100, f0 / 10 / 100)
    f = np.linspace(20, 80, 60001); _, h = signal.freqz(b, a, worN=f, fs=FS); m = np.abs(h)
    assert m[np.argmin(np.abs(f - f0))] < 1e-6 and abs(np.abs(signal.freqz(b, a, [0.0], fs=FS)[1][0]) - 1) < 0.02
    stop = f[m < 10 ** (-3 / 20)]; assert abs((stop.max() - stop.min()) - f0 / 10) < 0.1 * f0 / 10


def test_butter2_matches_scipy_butter():
    b, a = dc.butter2(5 / (FS / 2)); bs, as_ = signal.butter(2, 5 / (FS / 2))
    assert np.allclose(b, bs, atol=1e-12) and np.allclose(a, as_, atol=1e-12)


def test_fir_matches_scipy_firwin_hamming():
    h = dc.fir_lowpass(60, 5 / (FS / 2)); hs = signal.firwin(61, 5 / (FS / 2), window="hamming")
    assert np.allclose(h, hs, atol=1e-12) and abs(h.sum() - 1) < 1e-12


def test_fir_group_delay_is_half_the_order():
    h = dc.fir_lowpass(60, 5 / (FS / 2))
    assert np.allclose(dc.group_delay(h, [1.0], [1, 2], FS), 30.0, atol=1e-6)   # 150 ms at 200 Hz


def test_group_delay_matches_scipy():
    b, a = dc.butter2(5 / (FS / 2))
    _, gd = signal.group_delay((b, a), w=[1, 2], fs=FS)
    assert np.allclose(dc.group_delay(b, a, [1, 2], FS), gd, atol=1e-6)


def test_fft_peaks_found():
    t, roll, gyro, ay, az = dc.make_imu_signals(np.random.default_rng(0))
    f1, f2 = dc.find_two_peaks(ay, FS)
    assert abs(f1 - 47) < 0.2 and abs(f2 - 94) < 0.2


def test_movmedian_shrinks_at_edges():
    x = np.array([10.0, 1, 2, 3, 4, 5, 6, 7, 8]); y = dc.movmedian(x, 7)
    assert y[0] == np.median(x[:4]) and y[4] == np.median(x[1:8])


def test_fusion_beats_raw_and_gyro_only():
    r = dc.run_imu(np.random.default_rng(1))
    assert r["kalman"] < r["acc_raw"] / 2 and r["comp"] < r["gyro_only"]


def test_altitude_kalman_improves_snr():
    r = dc.run_altitude(np.random.default_rng(1))
    assert r["snr_db"][3] > r["snr_db"][0] + 5 and r["rmse_cm"][3] < r["rmse_cm"][0]
