"""Characterise the real sensor noise from the static recordings (Block A, RQ3).

    python analysis/noise_characterisation.py

For every Block A session (card fixed, nothing moving) it measures:
  * sigma: standard deviation of the readings (mm) = the noise level
  * skewness and excess kurtosis: both are 0 for Gaussian noise
  * KS test against a Gaussian with the same mean and sigma
  * quantisation step: the smallest gap between two different readings
  * autocorrelation: is the noise of one reading related to the next one?
    (the simulation assumed independent "white" noise: autocorrelation 0)
  * longest run of identical readings (to check the guard's stuck_count)

Writes:
  results/tables/noise_summary.csv   one row per session
  results/tables/noise_model.json    sigma(distance) straight-line fit, used by ground_truth.py
  results/figures/fig1_noise_vs_gaussian.png/.pdf   (Figure 1)
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
import pandas as pd
from scipy import stats

sys.path.insert(0, str(Path(__file__).resolve().parent))
import plotstyle  # noqa: E402
from common import (Paths, add_path_args, check_no_synthetic_in_results, list_sessions, load_session,  # noqa: E402
                    provenance, write_json)

MAX_LAG = 60  # autocorrelation up to 60 samples (about 2 s at 30 Hz)


def static_readings(df: pd.DataFrame) -> np.ndarray:
    """The usable readings of a static session (rows with no reading are dropped)."""
    col = "clean_mm" if "clean_mm" in df else "raw_mm"
    v = df[col].to_numpy(dtype=float)
    return v[v >= 0]


def autocorrelation(x: np.ndarray, max_lag: int = MAX_LAG) -> np.ndarray:
    """Sample autocorrelation r(k) for k = 0..max_lag (r(0) = 1)."""
    x = np.asarray(x, dtype=float) - np.mean(x)
    denom = np.dot(x, x)
    if denom == 0:
        return np.r_[1.0, np.zeros(max_lag)]
    return np.array([np.dot(x[: len(x) - k], x[k:]) / denom for k in range(max_lag + 1)])


def quantisation_step(x: np.ndarray) -> float:
    """Smallest difference between two distinct readings (1 mm for an integer sensor)."""
    u = np.unique(x)
    if len(u) < 2:
        return float("nan")
    return float(np.min(np.diff(u)))


def longest_equal_run(x: np.ndarray) -> int:
    """Longest run of identical consecutive readings."""
    if len(x) == 0:
        return 0
    best = run = 1
    for a, b in zip(x[:-1], x[1:]):
        run = run + 1 if a == b else 1
        best = max(best, run)
    return best


def summarise_session(x: np.ndarray) -> dict:
    mean, sd = float(np.mean(x)), float(np.std(x, ddof=1))
    ks = stats.kstest((x - mean) / sd, "norm") if sd > 0 else None
    acf = autocorrelation(x)
    return {
        "n": int(len(x)),
        "mean_mm": mean,
        "sigma_mm": sd,
        "skew": float(stats.skew(x)),
        "excess_kurtosis": float(stats.kurtosis(x)),  # Fisher definition: Gaussian = 0
        # Mean and sigma come from the same data, so this p-value is only
        # approximate (too large); a small p is still strong evidence against a Gaussian.
        "ks_stat": float(ks.statistic) if ks else float("nan"),
        "ks_p_approx": float(ks.pvalue) if ks else float("nan"),
        "quant_step_mm": quantisation_step(x),
        "acf_lag1": float(acf[1]),
        "acf_lag5": float(acf[5]) if len(acf) > 5 else float("nan"),
        "longest_equal_run": longest_equal_run(x),
    }


def fit_sigma_vs_distance(d: np.ndarray, s: np.ndarray) -> dict:
    """Straight line sigma = a + b * distance (mm). Needs at least 2 distances."""
    if len(d) >= 2 and np.ptp(d) > 0:
        b, a = np.polyfit(d, s, 1)
    else:
        a, b = (float(np.mean(s)) if len(s) else float("nan")), 0.0
    return {"a_mm": float(a), "b_per_mm": float(b), "formula": "sigma_mm = a_mm + b_per_mm * distance_mm"}


def sigma_at(model: dict, distance_mm: float) -> float:
    """Noise level predicted by noise_model.json at a distance."""
    return max(0.1, model["a_mm"] + model["b_per_mm"] * distance_mm)


def make_figure(paths: Paths, per_session: list[tuple[str, np.ndarray, dict]], fit: dict, synthetic: bool):
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(2, 2, figsize=(8, 6.2))
    (ax_h, ax_q), (ax_a, ax_s) = axes
    cmap = plt.get_cmap("viridis")
    order = sorted(per_session, key=lambda t: t[2]["mean_mm"])

    # (a) Histogram of the middle distance, with the Gaussian of the same mean and sigma.
    sid, x, s = order[len(order) // 2]
    lo, hi = np.floor(x.min()) - 0.5, np.ceil(x.max()) + 0.5
    ax_h.hist(x, bins=np.arange(lo, hi + 1, max(1.0, s["quant_step_mm"] or 1.0)), density=True,
              color=plotstyle.COLORS["meas"], label="readings")
    g = np.linspace(lo, hi, 300)
    ax_h.plot(g, stats.norm.pdf(g, s["mean_mm"], s["sigma_mm"]), color="k", label="Gaussian, same mean and sigma")
    ax_h.set(title=f"(a) Histogram at {s['mean_mm']:.0f} mm", xlabel="reading (mm)", ylabel="density")
    ax_h.legend(fontsize=7)

    # (b) Q-Q plot of every distance (standardised). On the diagonal = Gaussian.
    for i, (sid, x, s) in enumerate(order):
        z = np.sort((x - s["mean_mm"]) / s["sigma_mm"])
        q = stats.norm.ppf((np.arange(1, len(z) + 1) - 0.5) / len(z))
        ax_q.plot(q, z, ".", ms=2, color=cmap(i / max(1, len(order) - 1)), label=f"{s['mean_mm']:.0f} mm")
    lim = [-4, 4]
    ax_q.plot(lim, lim, "k--", lw=0.8)
    ax_q.set(title="(b) Q-Q plot against a Gaussian", xlabel="Gaussian quantile", ylabel="standardised reading",
             xlim=lim, ylim=[-6, 6])
    ax_q.legend(fontsize=6, ncol=2)

    # (c) Autocorrelation. The 95 % band is where white noise would fall.
    for i, (sid, x, s) in enumerate(order):
        ax_a.plot(np.arange(MAX_LAG + 1), autocorrelation(x), color=cmap(i / max(1, len(order) - 1)), lw=1)
    n = min(s["n"] for _, _, s in order)
    ax_a.axhspan(-1.96 / np.sqrt(n), 1.96 / np.sqrt(n), color="k", alpha=0.1, label="white-noise 95 % band")
    ax_a.set(title="(c) Autocorrelation", xlabel="lag (samples, about 33 ms each)", ylabel="r(lag)")
    ax_a.legend(fontsize=7)

    # (d) Noise level against distance, with the straight-line fit.
    d = np.array([s["mean_mm"] for _, _, s in order])
    sg = np.array([s["sigma_mm"] for _, _, s in order])
    ax_s.plot(d, sg, "o", color=plotstyle.COLORS["kf"])
    dd = np.linspace(d.min(), d.max(), 50)
    ax_s.plot(dd, fit["a_mm"] + fit["b_per_mm"] * dd, "k-", lw=0.8, label="straight-line fit")
    ax_s.set(title="(d) Noise level vs distance", xlabel="distance (mm)", ylabel="sigma (mm)")
    ax_s.legend(fontsize=7)

    fig.suptitle("Figure 1. Real VL53L0X noise compared with the Gaussian assumption (Block A)")
    fig.tight_layout()
    return plotstyle.save(fig, paths.figures, "fig1_noise_vs_gaussian", synthetic)


def run(paths: Paths) -> dict:
    sessions = list_sessions(paths, block="A")
    if not sessions:
        raise SystemExit(f"No Block A sessions in {paths.raw}. Record them first (docs/BRING_UP.md, Milestone 3).")
    metas, rows, per_session = [], [], []
    for sid in sessions:
        df, meta = load_session(paths, sid)
        metas.append(meta)
        x = static_readings(df)
        if len(x) < 30:
            print(f"warning: {sid} has only {len(x)} readings; skipped")
            continue
        s = summarise_session(x)
        user = meta.get("user", {})
        rows.append({"session_id": sid, "target_distance_mm": user.get("target_distance_mm"),
                     "temp_c": user.get("temp_c"), **s})
        per_session.append((sid, x, s))
    check_no_synthetic_in_results(paths, metas)
    paths.make_output_dirs()

    table = pd.DataFrame(rows)
    table.to_csv(paths.tables / "noise_summary.csv", index=False)
    fit = fit_sigma_vs_distance(table["mean_mm"].to_numpy(), table["sigma_mm"].to_numpy())
    model = {
        "sigma_vs_distance": fit,
        "pooled": {
            "median_sigma_mm": float(table["sigma_mm"].median()),
            "median_excess_kurtosis": float(table["excess_kurtosis"].median()),
            "median_skew": float(table["skew"].median()),
            "median_acf_lag1": float(table["acf_lag1"].median()),
            "quant_step_mm": float(table["quant_step_mm"].min()),
            "max_equal_run": int(table["longest_equal_run"].max()),
        },
        "provenance": provenance(paths, sessions),
    }
    write_json(paths.tables / "noise_model.json", model)
    make_figure(paths, per_session, fit, model["provenance"]["synthetic"])
    print(table[["session_id", "mean_mm", "sigma_mm", "excess_kurtosis", "acf_lag1", "longest_equal_run"]]
          .to_string(index=False))
    print(f"Wrote {paths.tables / 'noise_summary.csv'}, noise_model.json and Figure 1.")
    return model


def main(argv=None):
    ap = add_path_args(argparse.ArgumentParser(description=__doc__,
                                               formatter_class=argparse.RawDescriptionHelpFormatter))
    args = ap.parse_args(argv)
    run(Paths(args.data, args.results))


if __name__ == "__main__":
    main()
