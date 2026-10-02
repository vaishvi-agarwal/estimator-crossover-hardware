"""ground_truth.py recovers a known damped sine from synthetic noisy readings."""
import numpy as np
import pandas as pd

from ground_truth import damped_sine, rts_smooth, sinusoid_truth, windows


def synthetic_swing(seconds=40.0, noise=2.0, seed=0):
    rng = np.random.default_rng(seed)
    t = np.arange(0, seconds, 1 / 30.0)
    true = damped_sine(t, 300.0, 60.0, 0.02, 2 * np.pi / 1.42, 0.3)
    meas = np.round(true + noise * rng.normal(size=len(t)))
    df = pd.DataFrame({"t_us": (t * 1e6).astype(int), "t_s": t, "clean_mm": meas.astype(int),
                       "button": 0})
    return df, true


def test_windows_cover_the_session():
    w = windows(60.0)
    assert w[0][0] == 0.0 and w[-1][1] == 60.0
    assert all(b - a == 10.0 for a, b in w)


def test_fit_recovers_truth():
    df, true = synthetic_swing()
    out, info = sinusoid_truth(df, omega_hint=np.sqrt(9.81 / 0.5))
    err = out["truth_m"].to_numpy() * 1000 - true
    assert info["n_failed_windows"] == 0
    assert np.sqrt(np.mean(err ** 2)) < 0.6        # much better than the 2 mm noise
    assert abs(info["resid_rms_mm"] - 2.0) < 0.3   # residual = the noise


def test_gap_is_bridged():
    df, true = synthetic_swing()
    gap = (df["t_s"] > 20) & (df["t_s"] < 21)
    df.loc[gap, "clean_mm"] = -1
    out, _ = sinusoid_truth(df, None)
    err = out["truth_m"].to_numpy()[gap.to_numpy()] * 1000 - true[gap.to_numpy()]
    assert np.all(np.isfinite(err)) and np.max(np.abs(err)) < 2.0


def test_rts_smoother_reduces_noise():
    df, true = synthetic_swing()
    sm = rts_smooth(df["t_s"].to_numpy(), df["clean_mm"].to_numpy(float), 1e5, 4.0)
    assert np.sqrt(np.mean((sm - true) ** 2)) < 1.5
