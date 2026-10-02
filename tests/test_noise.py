"""noise_characterisation.py on synthetic numbers with known answers."""
import numpy as np

from noise_characterisation import autocorrelation, fit_sigma_vs_distance, longest_equal_run, quantisation_step, \
    summarise_session


def test_white_noise_has_small_autocorrelation():
    rng = np.random.default_rng(0)
    r = autocorrelation(rng.normal(size=5000), 5)
    assert r[0] == 1.0
    assert np.all(np.abs(r[1:]) < 0.05)


def test_ar1_noise_autocorrelation():
    rng = np.random.default_rng(1)
    x = np.zeros(20000)
    for i in range(1, len(x)):
        x[i] = 0.8 * x[i - 1] + rng.normal()
    assert abs(autocorrelation(x, 1)[1] - 0.8) < 0.02


def test_quantisation_and_runs():
    x = np.array([250, 250, 251, 253, 253, 253, 252], dtype=float)
    assert quantisation_step(x) == 1.0
    assert longest_equal_run(x) == 3


def test_gaussian_summary():
    rng = np.random.default_rng(2)
    s = summarise_session(300 + 2.0 * rng.normal(size=4000))
    assert abs(s["sigma_mm"] - 2.0) < 0.1
    assert abs(s["excess_kurtosis"]) < 0.3


def test_sigma_fit():
    fit = fit_sigma_vs_distance(np.array([100, 200, 300.0]), np.array([1.0, 1.5, 2.0]))
    assert abs(fit["b_per_mm"] - 0.005) < 1e-12 and abs(fit["a_mm"] - 0.5) < 1e-9
