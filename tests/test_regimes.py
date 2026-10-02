"""Regime labelling (evaluate.py) against hand-worked examples, and the bootstrap."""
import numpy as np

from evaluate import (BURST, NOMINAL, RECOVERY, bad_from_events, find_crossover, label_regimes, paired_bootstrap,
                      pooled_bootstrap_ci)

N, B, R = NOMINAL, BURST, RECOVERY


def test_regimes_hand_worked():
    bad = [0, 0, 1, 1, 0, 0, 0, 1, 0]
    assert list(label_regimes(bad, 2)) == [N, N, B, B, R, R, N, B, R]


def test_regimes_back_to_back_bursts():
    # A new burst inside the recovery window is a burst again; recovery restarts after it.
    bad = [1, 0, 1, 0, 0, 0, 0]
    assert list(label_regimes(bad, 3)) == [B, R, B, R, R, R, N]


def test_regimes_no_bursts_and_end_of_session():
    assert list(label_regimes([0, 0, 0], 2)) == [N, N, N]
    assert list(label_regimes([0, 1, 1], 5)) == [N, B, B]


def test_bad_from_events():
    # 3 = fault starts, 4 = a reset was tried, 5 = recovered (that row is good again), 1 = latch-up trip
    codes = [0, 3, 0, 4, 0, 5, 0, 1, 0, 5, 0]
    assert list(bad_from_events(codes).astype(int)) == [0, 1, 1, 1, 1, 0, 0, 1, 1, 0, 0]


def test_paired_bootstrap_detects_a_real_difference():
    rng = np.random.default_rng(0)
    a = rng.normal(10, 1, 20)
    b = a - 2 + rng.normal(0, 0.3, 20)  # a is worse by 2 in every session
    r = paired_bootstrap(a, b, n_boot=4000, seed=1)
    assert r["ci_low"] > 1.5 and r["ci_high"] < 2.5
    assert r["p_two_sided"] < 0.01


def test_paired_bootstrap_no_difference():
    rng = np.random.default_rng(2)
    a = rng.normal(10, 1, 30)
    b = a + rng.normal(0, 0.5, 30)
    r = paired_bootstrap(a, b, n_boot=4000, seed=3)
    assert r["ci_low"] < 0 < r["ci_high"]
    assert r["p_two_sided"] > 0.05


def test_paired_bootstrap_interval_coverage():
    # Over many synthetic experiments, a 95 % interval should contain the true mean difference ~95 % of the time.
    rng = np.random.default_rng(4)
    hits = 0
    for k in range(200):
        d = rng.normal(1.0, 2.0, 25)
        r = paired_bootstrap(d, np.zeros(25), n_boot=1000, seed=k)
        hits += r["ci_low"] <= 1.0 <= r["ci_high"]
    assert 0.88 <= hits / 200 <= 0.99


def test_pooled_bootstrap_ci():
    rng = np.random.default_rng(5)
    sessions = [rng.normal(0, 1, 500) for _ in range(10)]
    v, lo, hi = pooled_bootstrap_ci(sessions, np.median, n_boot=500)
    assert lo <= v <= hi and hi - lo < 0.3


def test_find_crossover():
    assert abs(find_crossover([0, 0.5, 1], [2.0, 1.0, -1.0]) - 0.75) < 1e-12
    assert np.isnan(find_crossover([0, 1], [1.0, 2.0]))
