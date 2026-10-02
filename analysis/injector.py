"""Proximity-coupled Gilbert–Elliott burst-fault injector: a line-by-line Python
copy of firmware/include/ge_injector.h. Read that file for the explanation.

Used to replay recorded clean sessions with new fault settings (the crossover
sweep in evaluate.py, and training data for the GRU). For the same clean
readings, settings and seed it produces exactly the same output as the
firmware; tests/test_injector.py checks this against a file written by the C++
test program.

The fault-chain probabilities are the paper's values per 0.1 s (10 Hz); they
are converted to the hardware sample period with the functions below.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

MASK64 = (1 << 64) - 1
PAPER_STEP_S = 0.1


class SplitMix64:
    def __init__(self, seed: int = 0):
        self.state = seed & MASK64

    def next(self) -> int:
        self.state = (self.state + 0x9E3779B97F4A7C15) & MASK64
        z = self.state
        z = ((z ^ (z >> 30)) * 0xBF58476D1CE4E5B9) & MASK64
        z = ((z ^ (z >> 27)) * 0x94D049BB133111EB) & MASK64
        return z ^ (z >> 31)

    def uniform(self) -> float:
        return float(self.next() >> 11) * (1.0 / 9007199254740992.0)

    def gaussian(self) -> float:
        u1 = 1.0 - self.uniform()
        u2 = self.uniform()
        return math.sqrt(-2.0 * math.log(u1)) * math.cos(2.0 * math.pi * u2)


# ---------------------------------------------------------------------------
# Converting per-step quantities between step lengths (docs/methods.md)
# ---------------------------------------------------------------------------
def persist_per_step(p_stay: float, dt_new: float, dt_old: float = PAPER_STEP_S) -> float:
    """Probability of STAYING in a state: p_new = p_stay ^ (dt_new / dt_old)."""
    return p_stay ** (dt_new / dt_old)


def entry_per_step(p_entry: float, dt_new: float, dt_old: float = PAPER_STEP_S) -> float:
    """Probability of LEAVING a state (entering the other): p_new = 1 - (1 - p) ^ (dt_new / dt_old)."""
    return 1.0 - (1.0 - p_entry) ** (dt_new / dt_old)


def steps_per(n_steps: float, dt_new: float, dt_old: float = PAPER_STEP_S) -> float:
    """A duration given in steps of dt_old, in steps of dt_new: n * dt_old / dt_new."""
    return n_steps * dt_old / dt_new


MODES = ("variance", "frozen", "bias", "blend", "randbias", "sysbias")


@dataclass
class GeConfig:
    entry_far: float = 0.002
    entry_near: float = 0.12
    persist_far: float = 0.5
    persist_near: float = 0.9
    r_ref_mm: float = 40.0
    d_src_mm: float = 220.0
    sample_period_s: float = 1.0 / 30.0
    sigma_nom_mm: float = 2.0
    kappa: float = 100.0
    bias_c_sigmas: float = 4.0
    randbias_sigmas: float = 10.0
    lam: float = 0.5
    mode: str = "variance"

    def extra_var_mm2(self) -> float:
        """Variance added to a BAD reading in variance mode: (kappa - 1) * R (mm^2)."""
        return (self.kappa - 1.0) * self.sigma_nom_mm * self.sigma_nom_mm


def quantise(mm: float) -> float:
    return math.floor(mm * 1000.0 + 0.5) / 1000.0


class GeInjector:
    def __init__(self, cfg: GeConfig | None = None, seed: int = 0):
        self.cfg = cfg or GeConfig()
        self.reset(seed)

    def reset(self, seed: int):
        self.rng = SplitMix64(seed)
        self.bad = False
        self.have_last_good = False
        self.last_good_mm = 0.0
        self.burst_bias_mm = 0.0

    def rho(self, d_mm: float) -> float:
        c = self.cfg
        r = abs(d_mm - c.d_src_mm)
        rr = c.r_ref_mm * c.r_ref_mm
        return rr / (r * r + rr)

    def p_entry(self, d_mm: float) -> float:
        c = self.cfg
        p10 = c.entry_far + (c.entry_near - c.entry_far) * self.rho(d_mm)
        return 1.0 - math.pow(1.0 - p10, c.sample_period_s / PAPER_STEP_S)

    def p_persist(self, d_mm: float) -> float:
        c = self.cfg
        s10 = c.persist_far + (c.persist_near - c.persist_far) * self.rho(d_mm)
        return math.pow(s10, c.sample_period_s / PAPER_STEP_S)

    def apply(self, clean_mm: float) -> tuple[float, bool]:
        c = self.cfg
        u = self.rng.uniform()
        was_bad = self.bad
        if not self.bad:
            if u < self.p_entry(clean_mm):
                self.bad = True
        else:
            if u < 1.0 - self.p_persist(clean_mm):
                self.bad = False
        if not self.bad:
            self.last_good_mm = clean_mm
            self.have_last_good = True
            return quantise(clean_mm), False
        n = self.rng.gaussian()
        nb = self.rng.gaussian()
        if not was_bad:
            self.burst_bias_mm = c.randbias_sigmas * c.sigma_nom_mm * nb
        add_sd = math.sqrt(c.kappa - 1.0) * c.sigma_nom_mm
        prox_bias = c.bias_c_sigmas * c.sigma_nom_mm * self.rho(clean_mm)
        if c.mode == "variance":
            out = clean_mm + add_sd * n
        elif c.mode == "frozen":
            out = self.last_good_mm if self.have_last_good else clean_mm
        elif c.mode == "bias":
            out = clean_mm + prox_bias
        elif c.mode == "randbias":
            out = clean_mm + self.burst_bias_mm + add_sd * n
        elif c.mode == "sysbias":
            out = clean_mm + prox_bias + add_sd * n
        else:  # blend: assumed linear between the paper's endpoints
            out = clean_mm + math.sqrt(1.0 - c.lam) * add_sd * n + c.lam * prox_bias
        return quantise(out), True


def inject_stream(clean_mm, cfg: GeConfig, seed: int) -> tuple[np.ndarray, np.ndarray]:
    """Run the injector over a whole session.

    clean_mm: clean readings; values < 0 mean "no reading" and pass through as
    -1 without advancing the injector (the firmware does the same).
    Returns (faulted_mm, bad) arrays.
    """
    inj = GeInjector(cfg, seed)
    faulted = np.full(len(clean_mm), -1.0)
    bad = np.zeros(len(clean_mm), dtype=bool)
    for i, c in enumerate(clean_mm):
        if c < 0:
            bad[i] = inj.bad  # a gap inside a burst stays part of the burst
            continue
        faulted[i], bad[i] = inj.apply(float(c))
    return faulted, bad


def stationary_bad_fraction(p_entry: float, p_exit: float) -> float:
    """Long-run fraction of readings in BAD (constant probabilities): entry / (entry + exit)."""
    return p_entry / (p_entry + p_exit)


def count_transitions(bad_sequences) -> tuple[float, float]:
    """Estimate the Markov transition probabilities per step from label sequences
    (the IMM-est variant): p_nb = P(GOOD -> BAD), p_bn = P(BAD -> GOOD).

    Counts every consecutive pair of samples in every sequence.
    """
    n = np.zeros((2, 2))
    for b in bad_sequences:
        b = np.asarray(b, dtype=int)
        if len(b) < 2:
            continue
        np.add.at(n, (b[:-1], b[1:]), 1)
    p_nb = n[0, 1] / n[0].sum() if n[0].sum() else 0.0
    p_bn = n[1, 0] / n[1].sum() if n[1].sum() else 1.0
    return float(p_nb), float(p_bn)


def oracle_transitions(clean_mm, cfg: GeConfig) -> tuple[np.ndarray, np.ndarray]:
    """The TRUE per-sample switch probabilities the injector used (IMM-oracle):
    p_nb[k] = entry probability at reading k, p_bn[k] = 1 - persistence at reading k
    (both depend on the distance through rho). Rows without a reading do not
    advance the injector, so they get 0 and 0 (no switch possible)."""
    inj = GeInjector(cfg)
    p_nb = np.zeros(len(clean_mm))
    p_bn = np.zeros(len(clean_mm))
    for i, c in enumerate(clean_mm):
        if c >= 0:
            p_nb[i] = inj.p_entry(float(c))
            p_bn[i] = 1.0 - inj.p_persist(float(c))
    return p_nb, p_bn
