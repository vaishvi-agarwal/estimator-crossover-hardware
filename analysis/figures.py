"""Make every figure in SPEC.md section 6 from the tables the other scripts wrote.

    python analysis/figures.py            # all figures
    python analysis/figures.py --only 3   # one figure

Order of work: noise_characterisation.py -> ground_truth.py -> make_splits.py
-> tune.py -> evaluate.py (--crossover) -> figures.py.

    1  Real noise vs Gaussian (histogram, Q-Q, autocorrelation)    RQ3
    2  Example session: truth, faulted readings, all estimators    -
    3  Median and p95 error per regime, variance and frozen        RQ1, RQ2
    4  NEES over time, KF vs IMM (overconfidence check)            -
    5  Crossover: p95 vs lambda, simulation crossover marked       RQ3
    6  Physical-fault results                                      RQ4
    7  Assumed vs measured GE parameters vs real faults: ranking   RQ5
    8  Compute time per update on the ESP32                        RQ6

A figure whose input does not exist yet is skipped with a message.
Figures made from synthetic demo data are stamped "SYNTHETIC DEMO DATA".
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np
import pandas as pd
from scipy import stats

sys.path.insert(0, str(Path(__file__).resolve().parent))
import plotstyle  # noqa: E402
from common import Paths, add_path_args, list_sessions, load_session  # noqa: E402
from plotstyle import COLORS, NAMES, REGIME_NAMES  # noqa: E402

import matplotlib.pyplot as plt  # noqa: E402

PAPER_REFERENCE = Path(__file__).resolve().parent / "paper_reference.json"
EST_ORDER = ["kf", "gated", "imm_oracle", "imm_est", "gru"]
CLASSICAL_NAMES = ["kf", "gated", "imm_oracle", "imm_est"]


def retuned(summary: pd.DataFrame) -> pd.DataFrame:
    """Figures show the re-tuned results; carried-over ones are in carried_over_vs_retuned.csv."""
    return summary[summary.tuning == "retuned"] if "tuning" in summary else summary


def _any_synthetic(paths: Paths) -> bool:
    p = paths.tables / "evaluate_provenance.json"
    return bool(json.loads(p.read_text()).get("synthetic", False)) if p.exists() else False


def _need(path: Path, what: str) -> bool:
    if not path.exists():
        print(f"  skipped: {path.name} not found ({what})")
        return False
    return True


def shade_bursts(ax, t, bad, label=True):
    bad = np.asarray(bad, bool)
    edges = np.diff(np.r_[0, bad.astype(int), 0])
    starts, ends = np.flatnonzero(edges == 1), np.flatnonzero(edges == -1)
    for k, (a, b) in enumerate(zip(starts, ends)):
        ax.axvspan(t[a], t[min(b, len(t) - 1)], color=COLORS["burst"], alpha=0.15, lw=0,
                   label="burst (BAD)" if (label and k == 0) else None)


# ---------------------------------------------------------------------------
def fig1(paths: Paths):
    """Re-run the noise script (it draws Figure 1)."""
    from noise_characterisation import run as noise_run  # noqa: PLC0415

    if not list_sessions(paths, "A"):
        print("  skipped: no Block A sessions")
        return
    noise_run(paths)


def example_session(paths: Paths, condition: str):
    d = paths.processed / "estimates" / condition
    files = sorted(d.glob("*.csv")) if d.exists() else []
    return (files[0].stem, pd.read_csv(files[0])) if files else (None, None)


def fig2(paths: Paths, condition: str = "variance", window_s: float = 12.0):
    sid, est = example_session(paths, condition)
    if est is None:
        print(f"  skipped: no estimates for {condition} (run evaluate.py)")
        return
    t = est["t_s"].to_numpy()
    # Show the window with the most BAD samples.
    bad = est["bad"].to_numpy().astype(bool)
    starts = np.arange(0, max(t[-1] - window_s, 0) + 0.01, 1.0)
    a = max(starts, key=lambda s: bad[(t >= s) & (t < s + window_s)].sum()) if len(starts) else 0
    w = (t >= a) & (t < a + window_s)
    fig, ax = plt.subplots(figsize=(9, 4))
    shade_bursts(ax, t[w], bad[w])
    fm = est["faulted_mm"].to_numpy()
    has = w & (fm >= 0)
    ax.plot(t[has], fm[has], ".", ms=3, color=COLORS["meas"], label="faulted readings")
    ax.plot(t[w], est["truth_m"][w] * 1000, color=COLORS["truth"], lw=1.6, label="truth (fitted)")
    for n in EST_ORDER:
        if f"{n}_pos" in est:
            ax.plot(t[w], est[f"{n}_pos"][w] * 1000, color=COLORS[n], lw=1, label=NAMES[n])
    lo, hi = np.nanpercentile(est["truth_m"][w] * 1000, [0, 100])
    ax.set_ylim(lo - 40, hi + 40)
    ax.set(xlabel="time (s)", ylabel="distance (mm)",
           title=f"Figure 2. Example test session {sid} ({condition} bursts)")
    ax.legend(ncol=4, fontsize=7, loc="upper right")
    plotstyle.save(fig, paths.figures, "fig2_example_session", _any_synthetic(paths))


def _bar_panel(ax, summary: pd.DataFrame, condition: str, metric: str, estimators, show_genie=True):
    regimes = ["nominal", "burst", "recovery"]
    names = [n for n in estimators if ((summary.condition == condition) & (summary.estimator == n)).any()]
    if show_genie and ((summary.condition == condition) & (summary.estimator == "genie")).any():
        names.append("genie")
    width = 0.8 / max(1, len(names))
    for k, n in enumerate(names):
        rows = summary[(summary.condition == condition) & (summary.estimator == n)].set_index("regime")
        vals = [rows.loc[r, f"{metric}_mm"] if r in rows.index else np.nan for r in regimes]
        lo = [rows.loc[r, f"{metric}_ci_low"] if r in rows.index else np.nan for r in regimes]
        hi = [rows.loc[r, f"{metric}_ci_high"] if r in rows.index else np.nan for r in regimes]
        x = np.arange(len(regimes)) + (k - (len(names) - 1) / 2) * width
        yerr = np.array([np.subtract(vals, lo), np.subtract(hi, vals)])
        ax.bar(x, vals, width, color=COLORS[n], label=NAMES[n], hatch="//" if n == "genie" else None,
               alpha=0.6 if n == "genie" else 1, yerr=np.clip(yerr, 0, None), capsize=2, error_kw={"lw": 0.8})
    ax.set_xticks(range(len(regimes)), [REGIME_NAMES[r] for r in regimes])


def fig3(paths: Paths):
    p = paths.tables / "metrics_summary.csv"
    if not _need(p, "run evaluate.py"):
        return
    s = retuned(pd.read_csv(p))
    conds = [c for c in ("variance", "frozen") if (s.condition == c).any()]
    if not conds:
        print("  skipped: no variance / frozen results")
        return
    fig, axes = plt.subplots(2, len(conds), figsize=(4.6 * len(conds), 6), squeeze=False)
    for j, c in enumerate(conds):
        for i, metric in enumerate(("median", "p95")):
            ax = axes[i, j]
            _bar_panel(ax, s, c, metric, EST_ORDER)
            ax.set(ylabel=f"{'median' if metric == 'median' else '95th-percentile'} |error| (mm)",
                   title=f"{c.capitalize()} bursts: {'median' if metric == 'median' else 'p95'}")
    axes[0, 0].legend(fontsize=7)
    fig.suptitle("Figure 3. Position error per regime, re-tuned settings (test sessions; bars = 95 % bootstrap "
                 "intervals over sessions)")
    fig.tight_layout()
    plotstyle.save(fig, paths.figures, "fig3_error_per_regime", _any_synthetic(paths))


def fig4(paths: Paths, condition: str = "variance", window_s: float = 1.0):
    sid, est = example_session(paths, condition)
    if est is None or "imm_oracle_var" not in est:
        print(f"  skipped: no estimates for {condition}")
        return
    t = est["t_s"].to_numpy()
    scored = est["scored"].to_numpy().astype(bool)
    rate = 1 / np.median(np.diff(t))
    n = max(1, int(round(window_s * rate)))
    fig, ax = plt.subplots(figsize=(9, 3.6))
    shade_bursts(ax, t, est["bad"].to_numpy())
    for name in ("kf", "imm_oracle"):
        e2 = (est[f"{name}_pos"] - est["truth_m"]) ** 2 / est[f"{name}_var"]
        e2 = e2.where(scored)
        ax.plot(t, e2.rolling(n, min_periods=max(1, n // 2)).mean(), color=COLORS[name], lw=1,
                label=f"{NAMES[name]} (mean over {window_s:g} s)")
    lo, hi = stats.chi2.ppf([0.025, 0.975], n) / n
    ax.axhspan(lo, hi, color="k", alpha=0.1, label="95 % band for an honest filter")
    ax.axhline(1, color="k", lw=0.6)
    ax.set_yscale("log")
    ax.set(xlabel="time (s)", ylabel="NEES (1 = honest)",
           title=f"Figure 4. Normalised estimation error squared, {sid} ({condition} bursts)")
    ax.legend(fontsize=7, ncol=4, loc="upper center", bbox_to_anchor=(0.5, -0.18))
    plotstyle.save(fig, paths.figures, "fig4_nees_kf_vs_imm", _any_synthetic(paths))


def fig5(paths: Paths):
    p = paths.tables / "crossover.csv"
    if not _need(p, "run evaluate.py --crossover"):
        return
    c = pd.read_csv(p)
    summary = json.loads((paths.tables / "crossover.json").read_text())
    fig, ax = plt.subplots(figsize=(6.8, 4.2))
    best = c[c.best_classical].sort_values("lambda")
    gru = c[c.estimator == "gru"].sort_values("lambda")
    for n in CLASSICAL_NAMES:
        r = c[c.estimator == n].sort_values("lambda")
        ax.plot(r["lambda"], r.test_mean_session_p95_mm, color=COLORS[n], lw=0.8, alpha=0.5, label=NAMES[n])
    ax.plot(best["lambda"], best.test_mean_session_p95_mm, "k-o", ms=4, label="best classical (chosen on validation)")
    ax.plot(gru["lambda"], gru.test_mean_session_p95_mm, "-s", color=COLORS["gru"], ms=4, label="GRU")
    a = summary.get("lambda_crossover")
    if a is not None and np.isfinite(a):
        ax.axvline(a, color="k", ls="--", lw=1, label=f"hardware crossover lambda = {a:.2f}")
        if summary.get("ci95"):
            ax.axvspan(*summary["ci95"], color="k", alpha=0.08)
    ref = json.loads(PAPER_REFERENCE.read_text()) if PAPER_REFERENCE.exists() else {}
    pa = ref.get("crossover_lambda")
    if pa is not None:
        ax.axvline(pa, color=COLORS["burst"], ls=":", lw=1.5, label=f"simulation paper crossover = {pa:.2f}")
        br = ref.get("crossover_bracket", {})
        if br:
            ax.axvspan(br["classical_ahead_at"], br["learned_ahead_at"], color=COLORS["burst"], alpha=0.07,
                       label=f"paper: classical ahead at {br['classical_ahead_at']}, learned at {br['learned_ahead_at']}")
    ax.set(xlabel="lambda  (0 = zero-mean inflated variance, 1 = nominal variance + bias)",
           ylabel=f"mean session p95 |error|, {summary.get('regime', 'burst')} regime (mm)",
           title="Figure 5. Crossover between the fault types")
    ax.legend(fontsize=7)
    plotstyle.save(fig, paths.figures, "fig5_crossover", _any_synthetic(paths))


def fig6(paths: Paths):
    p = paths.tables / "metrics_summary.csv"
    if not _need(p, "run evaluate.py"):
        return
    s = retuned(pd.read_csv(p))
    if not (s.condition == "physical").any():
        print("  skipped: no physical-fault (Block E) results")
        return
    fig, axes = plt.subplots(1, 2, figsize=(9, 3.6))
    for ax, metric in zip(axes, ("median", "p95")):
        _bar_panel(ax, s, "physical", metric, EST_ORDER)
        ax.set(ylabel=f"{metric} |error| (mm)", title=f"Physical faults: {metric}")
    axes[0].legend(fontsize=7)
    fig.suptitle("Figure 6. Physically caused faults: occlusion, strong light, I2C hangs (Block E, test sessions)")
    fig.tight_layout()
    plotstyle.save(fig, paths.figures, "fig6_physical_faults", _any_synthetic(paths))


def fig7(paths: Paths):
    p = paths.tables / "metrics_summary.csv"
    if not _need(p, "run evaluate.py"):
        return
    s = retuned(pd.read_csv(p))
    s = s[(s.regime == "burst") & (s.estimator.isin(EST_ORDER))]
    conds = [c for c in ("variance", "frozen", "measured", "live") if (s.condition == c).any()]
    if not any(c in conds for c in ("measured", "live")):
        print("  skipped: no Block F / G results yet")
        return
    labels = {"variance": "Assumed GE\n(variance, C)", "frozen": "Assumed GE\n(frozen, D)",
              "measured": "Measured GE\n(Spec 2, F)", "live": "Real faults\n(guard, G)"}
    fig, axes = plt.subplots(1, 2, figsize=(10, 4), gridspec_kw={"width_ratios": [3, 2]})
    ax = axes[0]
    width = 0.8 / len(EST_ORDER)
    for k, n in enumerate(EST_ORDER):
        r = s[s.estimator == n].set_index("condition")
        vals = [r.loc[c, "p95_mm"] if c in r.index else np.nan for c in conds]
        lo = [r.loc[c, "p95_ci_low"] if c in r.index else np.nan for c in conds]
        hi = [r.loc[c, "p95_ci_high"] if c in r.index else np.nan for c in conds]
        x = np.arange(len(conds)) + (k - (len(EST_ORDER) - 1) / 2) * width
        ax.bar(x, vals, width, color=COLORS[n], label=NAMES[n],
               yerr=np.clip([np.subtract(vals, lo), np.subtract(hi, vals)], 0, None), capsize=2)
    ax.set_xticks(range(len(conds)), [labels[c] for c in conds])
    ax.set(ylabel="burst-regime p95 |error| (mm)", title="(a) Error during bursts")
    ax.legend(fontsize=7)
    # (b) Rank table: 1 = best
    ax = axes[1]
    ax.axis("off")
    cell = []
    for n in EST_ORDER:
        row = []
        for c in conds:
            r = s[s.condition == c].dropna(subset=["p95_mm"]).sort_values("p95_mm").reset_index(drop=True)
            idx = r.index[r.estimator == n]
            row.append(str(int(idx[0]) + 1) if len(idx) else "-")
        cell.append(row)
    short = {"variance": "C", "frozen": "D", "measured": "F", "live": "G"}
    tab = ax.table(cellText=cell, rowLabels=[NAMES[n] for n in EST_ORDER],
                   colLabels=[f"Block {short[c]}" for c in conds], loc="center", cellLoc="center")
    tab.auto_set_font_size(False)
    tab.set_fontsize(7)
    tab.scale(1, 1.6)
    ax.set_title("(b) Rank by burst p95 (1 = best)")
    fig.suptitle("Figure 7. Assumed vs measured fault parameters, and injected vs real faults (RQ5)")
    fig.tight_layout()
    plotstyle.save(fig, paths.figures, "fig7_assumed_measured_real", _any_synthetic(paths))


def fig8(paths: Paths):
    rows, mem, synthetic = [], {}, False
    for block in "BCDEFG":
        for sid in list_sessions(paths, block):
            df, meta = load_session(paths, sid)
            synthetic |= bool(meta.get("synthetic"))
            fw = meta.get("firmware", {})
            for n in ("kf", "gated", "imm"):
                if f"{n}_us" in df:
                    rows.append(pd.DataFrame({"estimator": n, "us": df[f"{n}_us"].to_numpy()}))
                if f"{n}_bytes" in fw:
                    mem[n] = int(fw[f"{n}_bytes"])
    if not rows:
        print("  skipped: no sessions with timing columns")
        return
    d = pd.concat(rows)
    fig, ax = plt.subplots(figsize=(6, 3.6))
    names = ["kf", "gated", "imm"]
    data = [d[d.estimator == n].us.to_numpy() for n in names]
    bp = ax.boxplot(data, whis=(1, 99), showfliers=False, patch_artist=True)
    ax.set_xticks(range(1, 4), [NAMES[n] for n in names])
    for patch, n in zip(bp["boxes"], names):
        patch.set_facecolor(COLORS[n])
        patch.set_alpha(0.6)
    for i, n in enumerate(names):
        med = np.median(data[i])
        txt = f"median {med:.0f} us"
        if n in mem:
            txt += f"\n{mem[n]} bytes"
        ax.text(i + 1, ax.get_ylim()[1] * 0.98, txt, ha="center", va="top", fontsize=7)
    ax.set(ylabel="time per update on the ESP32 (us)",
           title="Figure 8. Compute cost per update (box: quartiles, whiskers: 1st-99th percentile)")
    summary = d.groupby("estimator").us.describe(percentiles=[0.5, 0.95, 0.99])
    summary["bytes"] = pd.Series(mem)
    paths.tables.mkdir(parents=True, exist_ok=True)
    summary.to_csv(paths.tables / "compute_cost.csv")
    plotstyle.save(fig, paths.figures, "fig8_compute_time", synthetic)


FIGURES = {1: fig1, 2: fig2, 3: fig3, 4: fig4, 5: fig5, 6: fig6, 7: fig7, 8: fig8}


def main(argv=None):
    ap = add_path_args(argparse.ArgumentParser(description=__doc__,
                                               formatter_class=argparse.RawDescriptionHelpFormatter))
    ap.add_argument("--only", type=int, choices=sorted(FIGURES), help="make just this figure")
    args = ap.parse_args(argv)
    paths = Paths(args.data, args.results)
    paths.figures.mkdir(parents=True, exist_ok=True)
    for k, fn in FIGURES.items():
        if args.only and k != args.only:
            continue
        print(f"Figure {k}:")
        fn(paths)
    print(f"Figures are in {paths.figures}")


if __name__ == "__main__":
    main()
