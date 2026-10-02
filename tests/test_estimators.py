"""The Python estimators must give the same numbers as the C++ firmware estimators.

tests/reference_io/estimators_reference.csv is written by the C++ test program
(`make reference`): a faulted stream with gaps and uneven time steps, and the
outputs of the KF, gated KF and IMM from firmware/include/*.h.

On the machine that wrote the file the match is exact (0 difference). The test
allows 1e-12 m (a millionth of a micrometre) so it also passes on a computer
whose maths library rounds exp() or log() differently in the last bit.
"""
from pathlib import Path

import numpy as np
import pandas as pd

from estimators import GatedKfEstimator, GatedParams, ImmEstimator, ImmParams, KfEstimator, KfParams, \
    run_estimator

REF = Path(__file__).parent / "reference_io" / "estimators_reference.csv"
TOL = 1e-12


def read_reference():
    params = {}
    with open(REF) as f:
        for line in f:
            if not line.startswith("#"):
                break
            words = line[1:].split()
            params[words[0]] = {k: float(v) for k, v in (w.split("=") for w in words[1:])}
    return pd.read_csv(REF, comment="#", float_precision="round_trip"), params


def compare(name, ref, out, cols):
    for col, key in cols:
        a, b = ref[f"{name}_{col}"].to_numpy(), out[key]
        diff = np.max(np.abs(a - b))
        assert diff <= TOL, f"{name} {col}: max difference {diff:g}"


def test_kf_matches_cpp():
    ref, p = read_reference()
    out = run_estimator(KfEstimator(KfParams(**p["kf"])), ref.t_us.to_numpy(), ref.faulted_mm.to_numpy())
    compare("kf", ref, out, [("pos", "pos"), ("vel", "vel"), ("var", "var")])


def test_gated_matches_cpp():
    ref, p = read_reference()
    est = GatedKfEstimator(GatedParams(**p["gated"]))
    rejected = []
    t, z = ref.t_us.to_numpy(), ref.faulted_mm.to_numpy()
    out = run_estimator(est, t, z)
    compare("gated", ref, out, [("pos", "pos"), ("vel", "vel"), ("var", "var")])
    # the same readings were rejected
    est.reset()
    from estimators import time_steps
    for dt, zz in zip(time_steps(t), z):
        est.step(float(dt), zz >= 0, zz / 1000.0 if zz >= 0 else 0.0)
        rejected.append(int(est.last_rejected))
    assert rejected == list(ref.gated_rejected)
    assert sum(rejected) > 0, "the reference should exercise the gate"
    assert est.forced == ref.gated_forced.iloc[-1] > 0, "the reference should exercise the coasting timeout"


def test_imm_matches_cpp():
    ref, p = read_reference()
    out = run_estimator(ImmEstimator(ImmParams(**p["imm"])), ref.t_us.to_numpy(), ref.faulted_mm.to_numpy())
    compare("imm", ref, out, [("pos", "pos"), ("vel", "vel"), ("var", "var"), ("p_bad", "p_bad")])


def test_reference_exercises_bursts_and_gaps():
    ref, _ = read_reference()
    assert ref.bad.mean() > 0.05
    assert (ref.faulted_mm < 0).sum() > 10


def test_imm_oracle_matches_cpp():
    """IMM-oracle: the same IMM fed the injector's true, time-varying switch probabilities."""
    ref, p = read_reference()
    trans = (ref.p_nb_true.to_numpy(), ref.p_bn_true.to_numpy())
    out = run_estimator(ImmEstimator(ImmParams(**p["imm"])), ref.t_us.to_numpy(), ref.faulted_mm.to_numpy(),
                        transitions=trans)
    compare("imm_oracle", ref, out, [("pos", "pos"), ("vel", "vel"), ("var", "var"), ("p_bad", "p_bad")])
    assert ref.p_nb_true.nunique() > 10, "the true probabilities should vary with distance"


def test_oracle_transitions_from_injector():
    """injector.oracle_transitions gives the reference file's true probabilities."""
    from injector import GeConfig, oracle_transitions

    ref, _ = read_reference()
    # The reference uses the default chain with d_src = 250 and the reference's clean swing.
    import numpy as np
    i = np.arange(len(ref))
    t = i / 30.0
    clean = np.floor(300.0 + 60.0 * np.exp(-0.02 * t) * np.sin(4.43 * t + 0.3) + 0.5)
    has = ref.faulted_mm.to_numpy() >= 0
    clean[~has] = -1
    p_nb, p_bn = oracle_transitions(clean, GeConfig(d_src_mm=250))
    assert np.max(np.abs(p_nb - ref.p_nb_true.to_numpy())) <= 1e-15
    assert np.max(np.abs(p_bn - ref.p_bn_true.to_numpy())) <= 1e-15
