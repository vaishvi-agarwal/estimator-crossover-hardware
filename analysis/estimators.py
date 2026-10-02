"""Python versions of the firmware estimators (KF, gated KF, IMM).

These are line-by-line copies of firmware/include/kalman1d.h, gated_kf.h and
imm1d.h. Every formula is written in the same order with plain floats, so
the results are bit-for-bit the same as the C++ code (tests/test_estimators.py
checks this against a file written by the C++ test program). Read the C++
headers for the explanations; the comments here only point out differences.

If you change a formula, change it in BOTH places and run `make reference`
and `pytest`.

Units: positions in metres, velocities in m/s, time steps in seconds.
"""
from __future__ import annotations

import math
from dataclasses import asdict, dataclass

import numpy as np


# ---------------------------------------------------------------------------
# Parameters (same names and defaults as the C++ structs)
# ---------------------------------------------------------------------------
@dataclass
class KfParams:
    q: float = 1.0
    r: float = 4e-6
    p0_vel: float = 1.0


@dataclass
class GatedParams:
    q: float = 1.0
    r: float = 4e-6
    p0_vel: float = 1.0
    gate: float = 6.634896601021214  # chi-square 1 DOF at 99 %; tuned from {95, 99, 99.9 %}
    coast_s: float = 1.0


@dataclass
class ImmParams:
    q: float = 1.0
    r: float = 4e-6
    p0_vel: float = 1.0
    r_bad_factor: float = 100.0
    p_nb: float = 0.003344506587403595
    p_bn: float = 0.12496598771667256
    mu_bad0: float = 0.1


# ---------------------------------------------------------------------------
# Kalman1D (kalman1d.h)
# ---------------------------------------------------------------------------
class Kalman1D:
    __slots__ = ("x0", "x1", "P00", "P01", "P11", "initialised")

    def __init__(self):
        self.reset()

    def reset(self):
        self.x0 = 0.0
        self.x1 = 0.0
        self.P00 = 0.0
        self.P01 = 0.0
        self.P11 = 0.0
        self.initialised = False

    def copy(self) -> "Kalman1D":
        k = Kalman1D()
        k.x0, k.x1, k.P00, k.P01, k.P11, k.initialised = self.x0, self.x1, self.P00, self.P01, self.P11, \
            self.initialised
        return k

    def init(self, z, r, p0_vel):
        self.x0 = z
        self.x1 = 0.0
        self.P00 = r
        self.P01 = 0.0
        self.P11 = p0_vel
        self.initialised = True

    def predict(self, dt, q):
        dt2 = dt * dt
        dt3 = dt2 * dt
        self.x0 = self.x0 + dt * self.x1
        p00 = self.P00 + 2.0 * dt * self.P01 + dt2 * self.P11 + q * dt3 / 3.0
        p01 = self.P01 + dt * self.P11 + q * dt2 / 2.0
        p11 = self.P11 + q * dt
        self.P00 = p00
        self.P01 = p01
        self.P11 = p11

    def innovation(self, z):
        return z - self.x0

    def innovation_var(self, r):
        return self.P00 + r

    def update(self, z, r):
        y = z - self.x0
        S = self.P00 + r
        k0 = self.P00 / S
        k1 = self.P01 / S
        self.x0 = self.x0 + k0 * y
        self.x1 = self.x1 + k1 * y
        p00 = (1.0 - k0) * self.P00
        p01 = (1.0 - k0) * self.P01
        p11 = self.P11 - k1 * self.P01
        self.P00 = p00
        self.P01 = p01
        self.P11 = p11


# ---------------------------------------------------------------------------
# The three estimators
# ---------------------------------------------------------------------------
class KfEstimator:
    name = "kf"

    def __init__(self, params: KfParams | None = None):
        self.params = params or KfParams()
        self.kf = Kalman1D()

    def reset(self):
        self.kf.reset()

    def step(self, dt, has_z, z):
        p = self.params
        if not self.kf.initialised:
            if has_z:
                self.kf.init(z, p.r, p.p0_vel)
            return
        if dt > 0.0:
            self.kf.predict(dt, p.q)
        if has_z:
            self.kf.update(z, p.r)

    def ready(self):
        return self.kf.initialised

    def pos(self):
        return self.kf.x0

    def vel(self):
        return self.kf.x1

    def pos_var(self):
        return self.kf.P00


class GatedKfEstimator(KfEstimator):
    name = "gated"

    def __init__(self, params: GatedParams | None = None):
        super().__init__()
        self.params = params or GatedParams()
        self.reset()

    def reset(self):
        self.kf.reset()
        self.last_rejected = False
        self.rejected = 0
        self.forced = 0
        self.coasting = 0.0

    def step(self, dt, has_z, z):
        p = self.params
        self.last_rejected = False
        if not self.kf.initialised:
            if has_z:
                self.kf.init(z, p.r, p.p0_vel)
                self.coasting = 0.0
            return
        if dt > 0.0:
            self.kf.predict(dt, p.q)
            self.coasting = self.coasting + dt
        if not has_z:
            return
        y = self.kf.innovation(z)
        S = self.kf.innovation_var(p.r)
        d2 = y * y / S
        if d2 > p.gate:
            if self.coasting < p.coast_s - 1e-9:
                self.last_rejected = True
                self.rejected += 1
                return
            self.forced += 1
        self.kf.update(z, p.r)
        self.coasting = 0.0


class ImmEstimator:
    name = "imm"

    def __init__(self, params: ImmParams | None = None):
        self.params = params or ImmParams()
        self.reset()

    def reset(self):
        self.f = [Kalman1D(), Kalman1D()]
        self.mu = [1.0, 0.0]
        self.x0 = self.x1 = self.P00 = self.P01 = self.P11 = 0.0
        self.initialised = False

    def r_of(self, j):
        return self.params.r if j == 0 else self.params.r * self.params.r_bad_factor

    def step(self, dt, has_z, z):
        self.step_with(dt, has_z, z, self.params.p_nb, self.params.p_bn)

    def step_with(self, dt, has_z, z, p_nb, p_bn):
        p = self.params
        f, mu = self.f, self.mu
        if not self.initialised:
            if not has_z:
                return
            f[0].init(z, p.r, p.p0_vel)
            f[1].init(z, p.r, p.p0_vel)
            mu[0] = 1.0 - p.mu_bad0
            mu[1] = p.mu_bad0
            self._combine()
            self.initialised = True
            return

        T = ((1.0 - p_nb, p_nb), (p_bn, 1.0 - p_bn))

        # (1) Mixing
        c = [T[0][0] * mu[0] + T[1][0] * mu[1], T[0][1] * mu[0] + T[1][1] * mu[1]]
        mixed = [Kalman1D(), Kalman1D()]
        for j in range(2):
            if c[j] > 0.0:
                w0 = T[0][j] * mu[0] / c[j]
                w1 = T[1][j] * mu[1] / c[j]
            else:
                w0 = 1.0 if j == 0 else 0.0
                w1 = 1.0 if j == 1 else 0.0
            m0 = w0 * f[0].x0 + w1 * f[1].x0
            m1 = w0 * f[0].x1 + w1 * f[1].x1
            a0, a1 = f[0].x0 - m0, f[0].x1 - m1
            b0, b1 = f[1].x0 - m0, f[1].x1 - m1
            k = mixed[j]
            k.x0 = m0
            k.x1 = m1
            k.P00 = w0 * (f[0].P00 + a0 * a0) + w1 * (f[1].P00 + b0 * b0)
            k.P01 = w0 * (f[0].P01 + a0 * a1) + w1 * (f[1].P01 + b0 * b1)
            k.P11 = w0 * (f[0].P11 + a1 * a1) + w1 * (f[1].P11 + b1 * b1)
            k.initialised = True

        # (2) + (3) Predict and update each mode
        ll = [0.0, 0.0]
        for j in range(2):
            f[j] = mixed[j]
            if dt > 0.0:
                f[j].predict(dt, p.q)
            if has_z:
                r = self.r_of(j)
                y = f[j].innovation(z)
                S = f[j].innovation_var(r)
                ll[j] = -0.5 * (y * y / S + math.log(2.0 * math.pi * S))
                f[j].update(z, r)

        # (4) Mode probabilities
        if has_z:
            mx = ll[0] if ll[0] > ll[1] else ll[1]
            u0 = c[0] * math.exp(ll[0] - mx)
            u1 = c[1] * math.exp(ll[1] - mx)
            s = u0 + u1
            if s > 0.0:
                mu[0] = u0 / s
                mu[1] = u1 / s
            else:
                mu[0] = c[0]
                mu[1] = c[1]
        else:
            mu[0] = c[0]
            mu[1] = c[1]

        # (5) Combine
        self._combine()

    def _combine(self):
        f, mu = self.f, self.mu
        self.x0 = mu[0] * f[0].x0 + mu[1] * f[1].x0
        self.x1 = mu[0] * f[0].x1 + mu[1] * f[1].x1
        a0, a1 = f[0].x0 - self.x0, f[0].x1 - self.x1
        b0, b1 = f[1].x0 - self.x0, f[1].x1 - self.x1
        self.P00 = mu[0] * (f[0].P00 + a0 * a0) + mu[1] * (f[1].P00 + b0 * b0)
        self.P01 = mu[0] * (f[0].P01 + a0 * a1) + mu[1] * (f[1].P01 + b0 * b1)
        self.P11 = mu[0] * (f[0].P11 + a1 * a1) + mu[1] * (f[1].P11 + b1 * b1)

    def ready(self):
        return self.initialised

    def pos(self):
        return self.x0

    def vel(self):
        return self.x1

    def pos_var(self):
        return self.P00

    def p_bad(self):
        return self.mu[1]


class GenieKf(KfEstimator):
    """Reference only, NOT a real estimator: a KF that is told the true GE state.

    It gives the "genie-aided bound" the paper compared against. During BAD
    readings it uses the true corrupted-noise variance (variance mode: the
    reading's own R plus the injected (kappa - 1) R) or ignores the reading
    (frozen / bias / blend / real outages). It cannot exist on the ESP32
    because nobody knows the true state there.
    """

    name = "genie"

    def __init__(self, params: KfParams | None = None, extra_var_m2: float = 0.0, mode: str = "variance"):
        super().__init__(params)
        self.extra_var_m2 = extra_var_m2
        self.mode = mode

    def step_with_state(self, dt, has_z, z, bad):
        p = self.params
        if not self.kf.initialised:
            if has_z and not bad:
                self.kf.init(z, p.r, p.p0_vel)
            return
        if dt > 0.0:
            self.kf.predict(dt, p.q)
        if not has_z:
            return
        if not bad:
            self.kf.update(z, p.r)
        elif self.mode == "variance":
            self.kf.update(z, p.r + self.extra_var_m2)
        # frozen / bias / blend / physical: a BAD reading carries no trustworthy information


# The estimator classes. The two IMM variants are the same IMM code; only
# their mode-switch probabilities differ:
#   imm_oracle  the injector's TRUE probabilities at every sample (they change
#               with distance through rho); known on hardware because the
#               injector generates them, so only for injected-fault conditions
#   imm_est     probabilities counted from the training sessions' fault sequences
# "imm" is the firmware's live IMM (fixed probabilities); it is only used to
# check the Python replay against the firmware. The firmware's IMM compute
# time stands for both variants.
CLASSICAL = {"kf": (KfEstimator, KfParams), "gated": (GatedKfEstimator, GatedParams),
             "imm": (ImmEstimator, ImmParams), "imm_oracle": (ImmEstimator, ImmParams),
             "imm_est": (ImmEstimator, ImmParams)}
EVALUATED = ("kf", "gated", "imm_oracle", "imm_est")  # the classical filters that are compared
FIRMWARE_ESTIMATORS = ("kf", "gated", "imm")  # the ones with columns in the CSV


def make_estimator(name: str, params: dict | None = None):
    """Build 'kf', 'gated', 'imm' or 'imm_est' with parameters from a dict (e.g. tuned_params.json)."""
    cls, pcls = CLASSICAL[name]
    return cls(pcls(**(params or {})))


def default_params(name: str) -> dict:
    return asdict(CLASSICAL[name][1]())


# ---------------------------------------------------------------------------
# Running an estimator over a whole session
# ---------------------------------------------------------------------------
def time_steps(t_us: np.ndarray) -> np.ndarray:
    """dt (s) for each row, exactly as the firmware computes it: (t - t_prev) / 1e6. First row: 0."""
    t = np.asarray(t_us, dtype=np.int64)
    dt = np.zeros(len(t))
    dt[1:] = (t[1:] - t[:-1]) / 1e6
    return dt


def run_estimator(est, t_us, faulted_mm, bad=None, transitions=None) -> dict:
    """Feed a whole session through one estimator.

    faulted_mm < 0 means "no reading". Positions are in metres:
    z = faulted_mm / 1000.0, as in the firmware.
    transitions: (p_nb, p_bn) arrays, one value per row, for IMM-oracle.
    Returns arrays pos, vel, var (position variance) and, for the IMM, p_bad.
    Rows before the estimator has started are NaN.
    """
    n = len(faulted_mm)
    dts = time_steps(t_us)
    pos, vel, var = np.full(n, np.nan), np.full(n, np.nan), np.full(n, np.nan)
    p_bad = np.full(n, np.nan) if isinstance(est, ImmEstimator) else None
    genie = isinstance(est, GenieKf)
    fm = np.asarray(faulted_mm, dtype=float)
    est.reset()
    for i in range(n):
        has_z = bool(fm[i] >= 0)
        z = fm[i] / 1000.0 if has_z else 0.0
        if genie:
            est.step_with_state(float(dts[i]), has_z, z, bool(bad[i]))
        elif transitions is not None:
            est.step_with(float(dts[i]), has_z, z, float(transitions[0][i]), float(transitions[1][i]))
        else:
            est.step(float(dts[i]), has_z, z)
        if est.ready():
            pos[i], vel[i], var[i] = est.pos(), est.vel(), est.pos_var()
            if p_bad is not None:
                p_bad[i] = est.p_bad()
    out = {"pos": pos, "vel": vel, "var": var}
    if p_bad is not None:
        out["p_bad"] = p_bad
    return out


# ---------------------------------------------------------------------------
# Checking a real recording against the firmware
# ---------------------------------------------------------------------------
def firmware_params(meta: dict) -> dict:
    """The estimator settings the firmware used, from the session's sidecar (# params lines)."""
    fw = meta.get("firmware", {})
    out = {}
    for name in FIRMWARE_ESTIMATORS:
        pcls = CLASSICAL[name][1]
        fields = {}
        for key in asdict(pcls()):
            v = fw.get(f"{name}_{key}")
            if v is not None:
                fields[key] = float(v)
        out[name] = fields
    return out


def check_against_firmware(df, meta: dict) -> dict:
    """Replay a recorded session through the Python estimators with the firmware's
    settings and return the largest difference (m) from the logged estimates.

    The firmware prints positions with 7 decimals, so anything below about
    1e-7 m means "identical". Larger numbers mean the Python and C++ code
    have drifted apart (or the recording did not start with `reset`).
    """
    params = firmware_params(meta)
    out = {}
    for name in FIRMWARE_ESTIMATORS:
        col = f"{name}_pos"
        if col not in df:
            continue
        res = run_estimator(make_estimator(name, params[name]), df["t_us"].to_numpy(), df["faulted_mm"].to_numpy())
        logged = df[col].to_numpy(dtype=float)
        both = np.isfinite(logged) & np.isfinite(res["pos"])
        out[name] = float(np.max(np.abs(logged[both] - res["pos"][both]))) if both.any() else float("nan")
    return out


# ---------------------------------------------------------------------------
# GRU estimator (learned; Python only, never runs on the ESP32)
# ---------------------------------------------------------------------------
# A small recurrent neural network that reads the faulted stream one sample
# at a time (like the filters: it only sees the past) and outputs a position.
# It is trained on TRAINING sessions only, against the fitted truth from
# ground_truth.py. It learns from examples what a fault looks like, instead
# of being told a noise model, which is why the paper found it best for
# frozen readings (they look precise, so the filters believe them).
#
# Inputs per sample: [normalised reading (0 if none), 1 if there is a reading,
#                     time step in units of 1/30 s]
# Output: normalised position. Normalisation constants come from the training
# truth and are stored with the model.
try:
    import torch
    from torch import nn
except ImportError:  # the classical estimators work without PyTorch
    torch = None
    nn = None


def _need_torch():
    if torch is None:
        raise SystemExit("PyTorch is needed for the GRU: pip install -r requirements.txt")


def gru_features(t_us, faulted_mm, offset_m: float, scale_m: float) -> np.ndarray:
    fm = np.asarray(faulted_mm, dtype=float)
    has = fm >= 0
    z = np.where(has, (fm / 1000.0 - offset_m) / scale_m, 0.0)
    dt = time_steps(t_us) * 30.0
    return np.stack([z, has.astype(float), dt], axis=1).astype(np.float32)


if nn is not None:
    class GruNet(nn.Module):
        def __init__(self, hidden: int = 32, layers: int = 1):
            super().__init__()
            self.gru = nn.GRU(input_size=3, hidden_size=hidden, num_layers=layers, batch_first=True)
            self.head = nn.Linear(hidden, 1)

        def forward(self, x):
            h, _ = self.gru(x)
            return self.head(h).squeeze(-1)


class GruEstimator:
    """Wraps a trained GruNet with its normalisation constants."""

    name = "gru"

    def __init__(self, net, offset_m: float, scale_m: float, info: dict | None = None):
        self.net, self.offset_m, self.scale_m = net, offset_m, scale_m
        self.info = info or {}

    def run(self, t_us, faulted_mm) -> dict:
        """Estimate a whole session (causal: each output uses only earlier and current samples)."""
        x = torch.from_numpy(gru_features(t_us, faulted_mm, self.offset_m, self.scale_m))[None]
        self.net.eval()
        with torch.no_grad():
            y = self.net(x)[0].numpy().astype(float)
        pos = y * self.scale_m + self.offset_m
        n = len(pos)
        return {"pos": pos, "vel": np.full(n, np.nan), "var": np.full(n, np.nan)}

    def state(self) -> dict:
        return {"state_dict": self.net.state_dict(), "offset_m": self.offset_m, "scale_m": self.scale_m,
                "hidden": self.net.gru.hidden_size, "layers": self.net.gru.num_layers, "info": self.info}

    @staticmethod
    def from_state(s: dict) -> "GruEstimator":
        _need_torch()
        net = GruNet(s["hidden"], s.get("layers", 1))
        net.load_state_dict(s["state_dict"])
        return GruEstimator(net, s["offset_m"], s["scale_m"], s.get("info"))


# The grid searched on validation. The paper's final model was 2 layers,
# hidden size 256, learning rate 1e-3, up to 250 epochs, early-stopping
# patience 30. Hidden size 256 is slow on a laptop CPU, so the default grid
# stops at 128; add 256 with --gru-grid if there is time.
DEFAULT_GRU_GRID = [{"layers": l, "hidden": h, "lr": 1e-3} for l in (1, 2) for h in (64, 128)]


def train_gru(train_streams: list[dict], val_streams: list[dict], score_fn, grid=None,
              max_epochs: int = 250, patience: int = 30, steps_per_epoch: int = 25, crop: int = 256,
              batch: int = 32, seed: int = 0, verbose: bool = False) -> GruEstimator:
    """Grid search over GRU settings; keep the model with the best validation score.

    Each stream is a dict with t_us, faulted_mm, truth_m (NaN where unknown).
    score_fn(estimator, val_streams) -> number to MINIMISE (tune.py passes the
    mean per-session 95th-percentile error, the same objective as for the filters).
    Each grid point trains for up to max_epochs epochs of steps_per_epoch
    batches, and stops early when the validation score has not improved for
    `patience` epochs (as in the paper). The best epoch of the best grid point wins.
    """
    _need_torch()
    grid = grid or DEFAULT_GRU_GRID
    rng = np.random.default_rng(seed)
    truth_all = np.concatenate([s["truth_m"][np.isfinite(s["truth_m"])] for s in train_streams])
    offset, scale = float(np.mean(truth_all)), float(max(np.std(truth_all), 1e-3))

    seqs = []
    for s in train_streams:
        x = gru_features(s["t_us"], s["faulted_mm"], offset, scale)
        y = ((s["truth_m"] - offset) / scale).astype(np.float32)
        seqs.append((x, y))
    seqs = [(x, y) for x, y in seqs if len(x) > 8]

    best_est, best_score = None, np.inf
    searched = []
    for g in grid:
        torch.manual_seed(seed)
        net = GruNet(g["hidden"], g["layers"])
        opt = torch.optim.Adam(net.parameters(), lr=g["lr"])
        point_best, since_best, epochs_run = np.inf, 0, 0
        for epoch in range(max_epochs):
            net.train()
            for _ in range(steps_per_epoch):
                xb, yb = [], []
                for _ in range(batch):
                    x, y = seqs[rng.integers(len(seqs))]
                    L = min(crop, len(x))
                    # A quarter of the crops start at the beginning, like a real run.
                    a = 0 if rng.random() < 0.25 else int(rng.integers(0, len(x) - L + 1))
                    xb.append(x[a:a + L])
                    yb.append(y[a:a + L])
                L = min(len(v) for v in xb)
                xt = torch.from_numpy(np.stack([v[:L] for v in xb]))
                yt = torch.from_numpy(np.stack([v[:L] for v in yb]))
                mask = torch.isfinite(yt)
                pred = net(xt)
                loss = ((pred - torch.nan_to_num(yt)) ** 2)[mask].mean()
                opt.zero_grad()
                loss.backward()
                nn.utils.clip_grad_norm_(net.parameters(), 1.0)
                opt.step()
            epochs_run = epoch + 1
            score = score_fn(GruEstimator(net, offset, scale), val_streams)
            if verbose:
                print(f"    GRU {g} epoch {epoch + 1}: validation score {score:.4g}")
            if score < point_best:
                point_best, since_best = score, 0
                if score < best_score:
                    best_score = score
                    info = {**g, "epoch": epoch + 1, "val_score": float(score)}
                    best_est = GruEstimator(GruNet(g["hidden"], g["layers"]), offset, scale, info)
                    best_est.net.load_state_dict({k: v.clone() for k, v in net.state_dict().items()})
            else:
                since_best += 1
                if since_best >= patience:
                    break
        searched.append({**g, "best_val_score": float(point_best), "epochs_run": epochs_run})
    best_est.info["grid_searched"] = searched
    best_est.info["max_epochs"], best_est.info["patience"] = max_epochs, patience
    best_est.info["steps_per_epoch"] = steps_per_epoch
    return best_est
