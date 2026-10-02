"""The Python injector (analysis/injector.py) must match the firmware's C++ injector exactly.

tests/reference_io/injector_reference.csv is written by the C++ test program:
    make reference
"""
from pathlib import Path

import numpy as np
import pandas as pd

from injector import (GeConfig, GeInjector, count_transitions, entry_per_step, inject_stream, persist_per_step,
                      stationary_bad_fraction, steps_per)

REF = Path(__file__).parent / "reference_io" / "injector_reference.csv"


def test_python_injector_matches_cpp():
    ref = pd.read_csv(REF, dtype={"seed": "uint64"}, float_precision="round_trip")
    for _, case in ref.groupby("case"):
        c0 = case.iloc[0]
        cfg = GeConfig(entry_far=c0.entry_far, entry_near=c0.entry_near, persist_far=c0.persist_far,
                       persist_near=c0.persist_near, r_ref_mm=c0.r_ref_mm, d_src_mm=c0.d_src_mm,
                       sample_period_s=c0.sample_period_s, sigma_nom_mm=c0.sigma_nom_mm, kappa=c0.kappa,
                       bias_c_sigmas=c0.bias_c_sigmas, randbias_sigmas=c0.randbias_sigmas, lam=c0["lambda"],
                       mode=c0["mode"])
        inj = GeInjector(cfg, int(c0.seed))
        out = [inj.apply(c) for c in case.clean_mm]
        assert [o[1] for o in out] == list(case.bad.astype(bool)), f"GE states differ in case {c0['case']}"
        # Identical up to the last bit is expected on one machine; a 1e-9 mm
        # tolerance allows for a different maths library (e.g. Linux CI vs macOS).
        assert np.max(np.abs(np.array([o[0] for o in out]) - case.faulted_mm.to_numpy())) < 1e-9


def test_gaps_do_not_advance_the_injector():
    cfg = GeConfig(entry_far=0.3, entry_near=0.3)
    clean = np.array([300, 301, -1, 302, 303.0])
    f1, b1 = inject_stream(clean, cfg, 3)
    f2, b2 = inject_stream(clean[clean >= 0], cfg, 3)
    assert f1[2] == -1
    assert np.array_equal(f1[clean >= 0], f2)


def test_stationary_fraction():
    assert abs(stationary_bad_fraction(0.02, 0.1) - 1 / 6) < 1e-12


def test_reference_covers_every_mode():
    ref = pd.read_csv(REF)
    assert set(ref["mode"]) == {"variance", "frozen", "bias", "blend", "randbias", "sysbias"}
    assert ref.bad.mean() > 0.02


def test_paper_conversions():
    # docs/methods.md, "Converting 10 Hz values" (dt_new = 1/30 s)
    T = 1 / 30
    assert abs(entry_per_step(0.002, T) - 0.000667111605597) < 1e-12
    assert abs(entry_per_step(0.12, T) - 0.041716028587443) < 1e-12
    assert abs(persist_per_step(0.5, T) - 0.793700525984100) < 1e-12
    assert abs(persist_per_step(0.9, T) - 0.965489384605630) < 1e-12
    assert abs(steps_per(21, T) - 63) < 1e-9 and abs(steps_per(10, T) - 30) < 1e-9
    # converting back and forth changes nothing
    assert abs(persist_per_step(persist_per_step(0.67, T), 0.1, T) - 0.67) < 1e-12


def test_count_transitions():
    p_nb, p_bn = count_transitions([[0, 0, 0, 1, 1, 0, 0, 0], [0, 1, 0]])
    # GOOD->BAD in 2 of the 6 pairs that start GOOD; BAD->GOOD in 2 of the 3 that start BAD
    assert abs(p_nb - 2 / 6) < 1e-12 and abs(p_bn - 2 / 3) < 1e-12
