"""The GRU trains, is causal, and beats doing nothing on a toy problem (needs PyTorch)."""
import numpy as np
import pytest

torch = pytest.importorskip("torch")

from estimators import train_gru  # noqa: E402


def stream(seed, n=600):
    rng = np.random.default_rng(seed)
    t = np.arange(n) / 30.0
    truth = 0.3 + 0.06 * np.sin(4.4 * t + rng.uniform(0, 6))
    meas = np.round((truth + 0.002 * rng.normal(size=n)) * 1000).astype(float)
    return {"t_us": (t * 1e6).astype(np.int64) + 1, "faulted_mm": meas, "truth_m": truth}


def score(est, streams):
    return float(np.mean([np.percentile(np.abs(est.run(s["t_us"], s["faulted_mm"])["pos"] - s["truth_m"]), 95)
                          for s in streams]))


def test_gru_trains_and_is_causal():
    train = [stream(i) for i in range(4)]
    val = [stream(10)]
    grid = [{"layers": 1, "hidden": 16, "lr": 1e-2}, {"layers": 2, "hidden": 8, "lr": 1e-2}]
    est = train_gru(train, val, score, grid=grid, max_epochs=10, patience=3, steps_per_epoch=40, batch=16,
                    crop=128)
    assert len(est.info["grid_searched"]) == 2  # every grid point was tried and reported
    s = val[0]
    out = est.run(s["t_us"], s["faulted_mm"])["pos"]
    assert np.all(np.isfinite(out))
    assert score(est, val) < 0.02  # far better than ignoring the readings (about 0.06 m)
    # Causal: changing a late reading must not change earlier outputs.
    f2 = s["faulted_mm"].copy()
    f2[400] += 100
    out2 = est.run(s["t_us"], f2)["pos"]
    assert np.array_equal(out[:400], out2[:400]) and not np.array_equal(out[400:], out2[400:])
