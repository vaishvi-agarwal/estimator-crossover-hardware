"""Reconstruct the true pendulum position from the CLEAN stream (SPEC.md 4.2.2).

    python analysis/ground_truth.py

There is no second instrument measuring the true position, so the truth is
estimated from physics. A swinging pendulum, seen from the side, moves almost
exactly like a damped sine wave:

    d(t) = d0 + A * exp(-gamma * t) * sin(omega * t + phi)

d0 = rest distance, A = amplitude, gamma = damping, omega = 2*pi/period.
Over a short stretch of time (a ~10 s window) these four numbers are nearly
constant, so fitting this curve through ~300 noisy readings averages the
noise away. The fitted curve is the "truth".

Primary method: the fit above in overlapping 10 s windows (scipy curve_fit).
Every sample takes the truth from the window whose centre is nearest to it.

Check: a Rauch-Tung-Striebel (RTS) smoother on the same clean readings. It
assumes nothing about sine waves, so if the two agree, the sine model is not
forcing a wrong shape on the data.

Quality check: the fit residual (reading minus fitted curve) should be about
as large as the sensor's static noise from Block A. If it is much larger
(--max-ratio, default 2), the sine model is not good enough for that session
and the script prints a warning.

Writes:
  <data>/processed/truth/<session>_truth.csv   one row per CSV row
  <results>/tables/truth_quality.csv           one row per session
"""
from __future__ import annotations

import argparse
import json
import sys
import warnings
from pathlib import Path

import numpy as np
import pandas as pd
from scipy.optimize import curve_fit

sys.path.insert(0, str(Path(__file__).resolve().parent))
from common import (Paths, add_path_args, check_no_synthetic_in_results, list_sessions, load_session,  # noqa: E402
                    write_json, provenance)

G = 9.81  # m/s^2

WINDOW_S = 10.0      # length of each fitting window
STEP_S = 5.0         # windows overlap by half
MIN_COVERAGE = 0.6   # a window needs readings for 60 % of its samples to be fitted
# Samples within this time of a button press are not used for fitting (Block E:
# a hand or lamp may be corrupting the "clean" readings then, and a person
# presses and releases the button a little late).
# TODO(hardware): check on Block E recordings that 0.5 s covers the delay
# between the fault starting/ending and the button press/release.
BUTTON_MARGIN_S = 0.5


def damped_sine(t, d0, A, gamma, omega, phi):
    return d0 + A * np.exp(-gamma * t) * np.sin(omega * t + phi)


def damped_sine_velocity(t, d0, A, gamma, omega, phi):
    """Time derivative of damped_sine (mm/s when d is in mm and t in s)."""
    e = A * np.exp(-gamma * t)
    return e * (omega * np.cos(omega * t + phi) - gamma * np.sin(omega * t + phi))


def dominant_omega(t: np.ndarray, d: np.ndarray, omega_hint: float | None) -> float:
    """Angular frequency of the swing: FFT peak near the hint (from the string length)."""
    tt = np.arange(t[0], t[-1], 1 / 30.0)
    if len(tt) < 16:
        return omega_hint or 2 * np.pi / 1.4
    dd = np.interp(tt, t, d) - np.mean(d)
    spec = np.abs(np.fft.rfft(dd * np.hanning(len(dd))))
    freqs = np.fft.rfftfreq(len(dd), 1 / 30.0) * 2 * np.pi
    ok = freqs > 1.0  # ignore slow drift (period > 6 s)
    if omega_hint:
        ok &= (freqs > 0.5 * omega_hint) & (freqs < 1.5 * omega_hint)
    if not ok.any():
        return omega_hint or 2 * np.pi / 1.4
    return float(freqs[ok][np.argmax(spec[ok])])


def fit_window(t: np.ndarray, d: np.ndarray, omega_hint: float | None):
    """Fit damped_sine to one window. t starts near 0. Returns (params, rms) or (None, nan)."""
    d0 = float(np.median(d))
    A0 = float((np.percentile(d, 95) - np.percentile(d, 5)) / 2)
    if A0 <= 0:
        return None, float("nan")
    w0 = dominant_omega(t, d, omega_hint)
    best, best_rms = None, np.inf
    for phi0 in np.linspace(0, 2 * np.pi, 8, endpoint=False):  # several starts: avoid a bad local fit
        try:
            with warnings.catch_warnings():
                warnings.simplefilter("ignore")
                p, _ = curve_fit(damped_sine, t, d, p0=[d0, A0, 0.01, w0, phi0],
                                 bounds=([d0 - 5 * A0, 0, -0.5, 0.8 * w0, -10], [d0 + 5 * A0, 10 * A0, 2.0, 1.2 * w0, 20]),
                                 maxfev=4000)
        except (RuntimeError, ValueError):
            continue
        rms = float(np.sqrt(np.mean((d - damped_sine(t, *p)) ** 2)))
        if rms < best_rms:
            best, best_rms = p, rms
    return best, best_rms


def windows(t_end: float) -> list[tuple[float, float]]:
    """Start/end times of the overlapping windows covering [0, t_end]."""
    if t_end <= WINDOW_S:
        return [(0.0, t_end)]
    starts = list(np.arange(0.0, t_end - WINDOW_S, STEP_S)) + [t_end - WINDOW_S]
    return [(float(s), float(s + WINDOW_S)) for s in starts]


def fit_mask(df: pd.DataFrame) -> np.ndarray:
    """Which rows may be used to fit the truth: a clean reading, and no button press nearby."""
    ok = df["clean_mm"].to_numpy() >= 0
    if "button" in df and df["button"].any():
        t = df["t_s"].to_numpy()
        pressed_t = t[df["button"].to_numpy() == 1]
        # distance from each sample to the nearest pressed sample
        idx = np.clip(np.searchsorted(pressed_t, t), 1, len(pressed_t) - 1) if len(pressed_t) > 1 else None
        if idx is None:
            near = np.abs(t - pressed_t[0]) <= BUTTON_MARGIN_S
        else:
            near = np.minimum(np.abs(t - pressed_t[idx - 1]), np.abs(t - pressed_t[idx])) <= BUTTON_MARGIN_S
        ok &= ~near
    return ok


def sinusoid_truth(df: pd.DataFrame, omega_hint: float | None) -> tuple[pd.DataFrame, dict]:
    """Fit every window; give each sample the truth of its nearest successful window."""
    t = df["t_s"].to_numpy()
    d = df["clean_mm"].to_numpy(dtype=float)
    use = fit_mask(df)
    period = np.median(np.diff(t)) if len(t) > 1 else 1 / 30
    wins = windows(float(t[-1]))
    fits = []
    for (a, b) in wins:
        inside = (t >= a) & (t <= b)
        sel = inside & use
        coverage = sel.sum() / max(1, inside.sum())
        if sel.sum() < 30 or coverage < MIN_COVERAGE:
            fits.append(None)
            continue
        p, rms = fit_window(t[sel] - a, d[sel], omega_hint)
        fits.append(None if p is None else (a, b, p, rms))

    truth = np.full(len(t), np.nan)
    vel = np.full(len(t), np.nan)
    win_id = np.full(len(t), -1)
    best_dist = np.full(len(t), np.inf)
    for k, f in enumerate(fits):
        if f is None:
            continue
        a, b, p, _ = f
        inside = (t >= a - period / 2) & (t <= b + period / 2)
        dist = np.abs(t - (a + b) / 2)
        take = inside & (dist < best_dist)
        truth[take] = damped_sine(t[take] - a, *p)
        vel[take] = damped_sine_velocity(t[take] - a, *p)
        win_id[take] = k
        best_dist[take] = dist[take]

    resid = np.where(use & np.isfinite(truth), d - truth, np.nan)
    ok = [f for f in fits if f is not None]
    info = {
        "n_windows": len(wins),
        "n_failed_windows": len(wins) - len(ok),
        "resid_rms_mm": float(np.sqrt(np.nanmean(resid ** 2))) if np.isfinite(resid).any() else float("nan"),
        "omega_mean": float(np.mean([f[2][3] for f in ok])) if ok else float("nan"),
        "gamma_mean": float(np.mean([f[2][2] for f in ok])) if ok else float("nan"),
        "amplitude_mean_mm": float(np.mean([f[2][1] for f in ok])) if ok else float("nan"),
        "mean_distance_mm": float(np.mean([f[2][0] for f in ok])) if ok else float("nan"),
        "truth_coverage": float(np.isfinite(truth).mean()),
    }
    out = pd.DataFrame({"t_us": df["t_us"], "t_s": t, "truth_m": truth / 1000.0, "truth_vel_m_s": vel / 1000.0,
                        "window": win_id, "used_in_fit": use, "resid_mm": resid})
    return out, info


# ---------------------------------------------------------------------------
# RTS smoother (the check). Constant-velocity model, as in the estimators.
# ---------------------------------------------------------------------------
def rts_smooth(t: np.ndarray, z: np.ndarray, q: float, r: float) -> np.ndarray:
    """Forward Kalman filter + backward RTS pass. z = NaN means no reading."""
    n = len(t)
    xs, Ps, xp, Pp = np.zeros((n, 2)), np.zeros((n, 2, 2)), np.zeros((n, 2)), np.zeros((n, 2, 2))
    first = np.flatnonzero(np.isfinite(z))[0]
    x = np.array([z[first], 0.0])
    P = np.diag([r, 1e4])
    H = np.array([1.0, 0.0])
    for k in range(n):
        dt = t[k] - t[k - 1] if k > 0 else 0.0
        F = np.array([[1, dt], [0, 1]])
        Q = q * np.array([[dt ** 3 / 3, dt ** 2 / 2], [dt ** 2 / 2, dt]])
        x, P = F @ x, F @ P @ F.T + Q
        xp[k], Pp[k] = x, P
        if np.isfinite(z[k]):
            S = H @ P @ H + r
            K = P @ H / S
            x = x + K * (z[k] - H @ x)
            P = P - np.outer(K, H @ P)
        xs[k], Ps[k] = x, P
    for k in range(n - 2, -1, -1):
        dt = t[k + 1] - t[k]
        F = np.array([[1, dt], [0, 1]])
        C = Ps[k] @ F.T @ np.linalg.inv(Pp[k + 1])
        xs[k] = xs[k] + C @ (xs[k + 1] - xp[k + 1])
        Ps[k] = Ps[k] + C @ (Ps[k + 1] - Pp[k + 1]) @ C.T
    return xs[:, 0]


def rts_check(df: pd.DataFrame, use: np.ndarray, sigma_mm: float) -> np.ndarray:
    """RTS-smoothed position (mm). q is chosen from a small grid by the innovation likelihood."""
    t = df["t_s"].to_numpy()
    z = np.where(use, df["clean_mm"].to_numpy(dtype=float), np.nan)
    r = sigma_mm ** 2
    best_q, best_ll = None, -np.inf
    for q in (1e3, 1e4, 1e5, 1e6):  # mm^2/s^3
        ll = _innovation_loglik(t, z, q, r)
        if ll > best_ll:
            best_q, best_ll = q, ll
    return rts_smooth(t, z, best_q, r)


def _innovation_loglik(t, z, q, r):
    first = np.flatnonzero(np.isfinite(z))[0]
    x, P, ll = np.array([z[first], 0.0]), np.diag([r, 1e4]), 0.0
    for k in range(first + 1, len(t)):
        dt = t[k] - t[k - 1]
        F = np.array([[1, dt], [0, 1]])
        x, P = F @ x, F @ P @ F.T + q * np.array([[dt ** 3 / 3, dt ** 2 / 2], [dt ** 2 / 2, dt]])
        if np.isfinite(z[k]):
            S = P[0, 0] + r
            y = z[k] - x[0]
            ll += -0.5 * (y * y / S + np.log(2 * np.pi * S))
            K = P[:, 0] / S
            x = x + K * y
            P = P - np.outer(K, P[0, :])
    return ll


# ---------------------------------------------------------------------------
def static_sigma(paths: Paths, distance_mm: float) -> float | None:
    """Static noise sigma at this distance from noise_model.json (Block A), if it exists."""
    p = paths.tables / "noise_model.json"
    if not p.exists():
        return None
    from noise_characterisation import sigma_at  # noqa: PLC0415

    return sigma_at(json.loads(p.read_text())["sigma_vs_distance"], distance_mm)


def process_session(paths: Paths, sid: str, max_ratio: float) -> dict:
    df, meta = load_session(paths, sid)
    L = (meta.get("user") or {}).get("string_length_m")
    omega_hint = float(np.sqrt(G / float(L))) if isinstance(L, (int, float)) and L > 0 else None
    truth, info = sinusoid_truth(df, omega_hint)

    sigma = static_sigma(paths, info["mean_distance_mm"]) if np.isfinite(info["mean_distance_mm"]) else None
    rts_mm = rts_check(df, truth["used_in_fit"].to_numpy(), sigma or max(1.0, info["resid_rms_mm"]))
    truth["rts_m"] = rts_mm / 1000.0
    both = np.isfinite(truth["truth_m"]) & truth["used_in_fit"]
    rts_vs_fit = float(np.sqrt(np.mean(((truth["rts_m"] - truth["truth_m"])[both] * 1000) ** 2))) if both.any() \
        else float("nan")

    ratio = info["resid_rms_mm"] / sigma if sigma else float("nan")
    adequate = bool(ratio <= max_ratio) if sigma else None
    if sigma is None:
        print(f"  {sid}: no noise_model.json yet (run noise_characterisation.py) - cannot check the fit residual")
    elif not adequate:
        print(f"  WARNING {sid}: fit residual {info['resid_rms_mm']:.2f} mm is {ratio:.1f}x the static noise "
              f"({sigma:.2f} mm). The damped-sine truth model is inadequate for this session.")
    if info["n_failed_windows"]:
        print(f"  {sid}: {info['n_failed_windows']} of {info['n_windows']} windows could not be fitted")

    out = paths.processed / "truth" / f"{sid}_truth.csv"
    out.parent.mkdir(parents=True, exist_ok=True)
    truth.to_csv(out, index=False)
    return {"session_id": sid, "block": meta.get("block"), **info, "static_sigma_mm": sigma,
            "resid_over_static": ratio, "truth_adequate": adequate, "rts_vs_fit_rms_mm": rts_vs_fit,
            "omega_from_string": omega_hint}


def run(paths: Paths, max_ratio: float = 2.0, blocks: str = "BCDEFG") -> pd.DataFrame:
    sessions = [s for b in blocks for s in list_sessions(paths, b)]
    if not sessions:
        raise SystemExit(f"No motion sessions (blocks {blocks}) in {paths.raw}.")
    metas = [json.loads((paths.raw / f"{s}.json").read_text()) for s in sessions]
    check_no_synthetic_in_results(paths, metas)
    paths.make_output_dirs()
    rows = [process_session(paths, s, max_ratio) for s in sessions]
    table = pd.DataFrame(rows)
    table.to_csv(paths.tables / "truth_quality.csv", index=False)
    write_json(paths.tables / "truth_quality_provenance.json",
               {"max_ratio": max_ratio, "window_s": WINDOW_S, "step_s": STEP_S, **provenance(paths, sessions)})
    print(table[["session_id", "resid_rms_mm", "static_sigma_mm", "resid_over_static", "rts_vs_fit_rms_mm"]]
          .to_string(index=False))
    return table


def main(argv=None):
    ap = add_path_args(argparse.ArgumentParser(description=__doc__,
                                               formatter_class=argparse.RawDescriptionHelpFormatter))
    ap.add_argument("--max-ratio", type=float, default=2.0,
                    help="warn when fit residual RMS > this times the static noise (default 2)")
    ap.add_argument("--blocks", default="BCDEFG", help="which blocks to process (default BCDEFG)")
    args = ap.parse_args(argv)
    run(Paths(args.data, args.results), args.max_ratio, args.blocks)


if __name__ == "__main__":
    main()
