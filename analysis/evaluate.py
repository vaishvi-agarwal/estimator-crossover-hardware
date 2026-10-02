"""Evaluate every estimator on the TEST sessions (SPEC.md 4.2.5).

    python analysis/evaluate.py                 # all conditions that have data
    python analysis/evaluate.py --crossover     # also run the crossover sweep (slow)

Run tune.py first: it writes the tuned parameters (validation sessions only)
and trains the GRUs (training sessions only). This script never tunes on the
test sessions; it only measures.

Conditions (which recordings, and how bursts are labelled):
    variance  Block C   BAD = injector state (ge_state column)          primary
    frozen    Block D   BAD = injector state                           primary
    bias      Blocks B-D clean streams replayed through the Python injector
    randbias  in each of the paper's secondary modes (bias-dominated, random
    sysbias   per-burst bias, systematic proximity bias); BAD = injector state
    physical  Block E   BAD = button held, or a sensor outage (recovery events)
    measured  Block F   BAD = injector state (GE parameters measured in Spec 2)
    live      Block G   BAD = an outage reported by the Spec 2 guard (recovery events)

Every condition is scored twice for the classical filters, as in the paper:
with settings re-tuned for that condition, and with the settings carried
over unchanged from the primary (inflated-variance) condition (see tune.py).
The GRU is never carried over: it is always trained on the condition's own
training sessions and selected on its own validation sessions.

Regimes, exactly as in the paper:
    burst     the sample is BAD
    recovery  within --recovery-s seconds after a burst ends
    nominal   everything else

Metrics per regime: median and 95th-percentile absolute position error
(primary), RMSE (secondary), and NEES = error^2 / predicted variance, which
should average 1 for an honest filter (much more than 1 = overconfident).

Comparisons: paired bootstrap (10,000 resamples) of per-session differences
in 95th-percentile error between every pair of estimators.

Crossover sweep (--crossover): replay the clean streams of Blocks B-D through
the Python injector in blend mode, lambda = 0 (zero-mean, inflated variance,
kappa = 100) ... 1 (nominal variance with a systematic bias), as in the paper.
Re-tune the filters and re-train the GRU at each lambda, and find the lambda
where the GRU starts to beat the best classical filter. (Frozen output stays a
separate condition, as in the paper.)

Writes into <results>/tables/ (and per-sample estimates into <data>/processed/estimates/).
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np
import pandas as pd

sys.path.insert(0, str(Path(__file__).resolve().parent))
from common import (EVENT_SEL_LOCKOUT, EVENT_SEL_TRIP, EVENT_SENSOR_FAILED, EVENT_SENSOR_FAULT,  # noqa: E402
                    EVENT_SENSOR_RECOVERED, Paths, add_path_args, check_no_synthetic_in_results, git_commit,
                    load_meta, load_session, load_truth, sessions_in, write_json)
from estimators import (EVALUATED, GenieKf, KfParams, check_against_firmware, make_estimator,  # noqa: E402
                        run_estimator)
from injector import MODES, GeConfig, inject_stream, oracle_transitions  # noqa: E402

CONDITIONS = {
    "variance": {"blocks": "C", "label": "ge", "title": "Variance bursts (Block C)"},
    "frozen": {"blocks": "D", "label": "ge", "title": "Frozen bursts (Block D)"},
    "bias": {"blocks": "BCD", "label": "ge", "replay": "bias", "title": "Bias-dominated (replayed B-D)"},
    "randbias": {"blocks": "BCD", "label": "ge", "replay": "randbias", "title": "Random per-burst bias (replayed B-D)"},
    "sysbias": {"blocks": "BCD", "label": "ge", "replay": "sysbias",
                "title": "Systematic proximity bias (replayed B-D)"},
    "physical": {"blocks": "E", "label": "physical", "title": "Physical faults (Block E)"},
    "measured": {"blocks": "F", "label": "ge", "title": "Measured GE parameters (Block F)"},
    "live": {"blocks": "G", "label": "events", "title": "Integrated live run (Block G)"},
}
ESTIMATORS = ["kf", "gated", "imm_oracle", "imm_est", "gru"]
TUNINGS = ("retuned", "carried_over")
REGIMES = ["nominal", "burst", "recovery"]
NOMINAL, BURST, RECOVERY = 0, 1, 2
REGIME_CODE = {"nominal": NOMINAL, "burst": BURST, "recovery": RECOVERY}

# The first seconds of a session are not scored: every filter starts from the
# first reading with an unknown velocity and needs a moment to settle.
WARMUP_S = 2.0
# Recovery regime length: the paper's N = 21 steps at 10 Hz = 2.1 s, kept in
# seconds (about 63 samples at 30 Hz).
RECOVERY_S = 2.1


# ---------------------------------------------------------------------------
# Labels
# ---------------------------------------------------------------------------
def label_regimes(bad, n_recovery: int) -> np.ndarray:
    """Per-sample regime codes: 1 = burst (BAD), 2 = recovery (within n_recovery
    samples after a burst ends), 0 = nominal. A burst that starts inside a
    recovery window is labelled burst (BAD always wins), and the recovery
    window restarts when that burst ends.

    Example, n_recovery = 2:
        bad     0 0 1 1 0 0 0 1 0
        regime  0 0 1 1 2 2 0 1 2
    """
    bad = np.asarray(bad, dtype=bool)
    reg = np.where(bad, BURST, NOMINAL)
    since_end = n_recovery + 1  # samples since the last BAD sample
    for i in range(len(bad)):
        if bad[i]:
            since_end = 0
            continue
        since_end += 1
        if since_end <= n_recovery and i > 0:
            reg[i] = RECOVERY
    # Before any burst there is nothing to recover from.
    if bad.any():
        reg[: int(np.argmax(bad))] = NOMINAL
    else:
        reg[:] = NOMINAL
    return reg


def bad_from_events(codes) -> np.ndarray:
    """BAD from the recovery_event column: from a fault (or latch-up trip) until the
    first row with SENSOR_RECOVERED. That recovered row is the first good reading
    again, so it is GOOD. (Same codes in both firmware builds; see common.py.)
    """
    starts = {EVENT_SEL_TRIP, EVENT_SEL_LOCKOUT, EVENT_SENSOR_FAULT, EVENT_SENSOR_FAILED}
    bad = np.zeros(len(codes), dtype=bool)
    inside = False
    for i, c in enumerate(np.asarray(codes, dtype=int)):
        if c in starts:
            inside = True
        elif c == EVENT_SENSOR_RECOVERED:
            inside = False
        bad[i] = inside
    return bad


def bad_labels(df: pd.DataFrame, how: str) -> np.ndarray:
    if how == "ge":
        return df["ge_state"].to_numpy() == 1
    if how == "events":
        return bad_from_events(df["recovery_event"].to_numpy())
    if how == "physical":
        return (df["button"].to_numpy() == 1) | bad_from_events(df["recovery_event"].to_numpy())
    raise ValueError(how)


def recovery_samples(t_us, seconds: float = RECOVERY_S) -> int:
    dt = np.median(np.diff(np.asarray(t_us))) / 1e6 if len(t_us) > 1 else 1 / 30
    return max(1, int(round(seconds / dt)))


# ---------------------------------------------------------------------------
# Streams: everything about one session that the estimators and metrics need
# ---------------------------------------------------------------------------
def ge_config_from_meta(meta: dict) -> GeConfig:
    """The injector settings the firmware used in this session (from its # params line)."""
    fw = meta.get("firmware", {})
    c = GeConfig()
    for k in ("entry_far", "entry_near", "persist_far", "persist_near", "r_ref_mm", "d_src_mm", "sample_period_s",
              "sigma_nom_mm", "kappa", "bias_c_sigmas", "randbias_sigmas"):
        if k in fw:
            setattr(c, k, float(fw[k]))
    if "lambda" in fw:
        c.lam = float(fw["lambda"])
    if fw.get("mode") in MODES:
        c.mode = fw["mode"]
    return c


def make_stream(paths: Paths, sid: str, label: str, faulted=None, bad=None, mode=None, cfg: GeConfig | None = None
                ) -> dict:
    """Load one session. faulted / bad replace the logged ones (replays and the crossover
    sweep pass the injector settings `cfg` they used, for IMM-oracle)."""
    df, meta = load_session(paths, sid)
    truth = load_truth(paths, sid)
    if bad is None:
        bad = bad_labels(df, label)
    fm = df["faulted_mm"].to_numpy(dtype=float) if faulted is None else np.asarray(faulted, dtype=float)
    t_s = df["t_s"].to_numpy()
    truth_m = truth["truth_m"].to_numpy(dtype=float)
    ge = cfg or ge_config_from_meta(meta)
    injected = cfg is not None or (label == "ge" and str(meta.get("firmware", {}).get("inject", "0")) == "1")
    # IMM-oracle: the injector's true switch probabilities (only where faults were injected)
    oracle = oracle_transitions(df["clean_mm"].to_numpy(dtype=float), ge) if injected else None
    return {
        "sid": sid, "meta": meta, "df": df,
        "t_us": df["t_us"].to_numpy(), "t_s": t_s, "faulted_mm": fm, "bad": np.asarray(bad, dtype=bool),
        "truth_m": truth_m, "score_mask": np.isfinite(truth_m) & (t_s >= WARMUP_S),
        "regime": label_regimes(bad, recovery_samples(df["t_us"].to_numpy())),
        "extra_var_m2": ge.extra_var_mm2() / 1e6, "mode": mode or (ge.mode if label == "ge" else "physical"),
        "oracle": oracle,
    }


def condition_streams(paths: Paths, condition: str, split: str) -> list[dict]:
    c = CONDITIONS[condition]
    if "replay" in c:
        return replay_streams(paths, split, c["replay"], seeds_per_session=2 if split == "train" else 1)
    return [make_stream(paths, s, c["label"]) for s in sessions_in(paths, split, list(c["blocks"]))]


def replay_streams(paths: Paths, split: str, mode: str, lam: float = 0.0, base: GeConfig | None = None,
                   seeds_per_session: int = 1) -> list:
    """Clean streams of Blocks B-D re-injected with the Python injector in `mode`
    (and lambda for blend), with the fault-chain settings of the Block C/D recordings."""
    from logger import session_seed  # noqa: PLC0415

    base = base or sweep_base_config(paths)
    out = []
    for sid in sessions_in(paths, split, ["B", "C", "D"]):
        df, _ = load_session(paths, sid)
        clean = df["clean_mm"].to_numpy(dtype=float)
        for k in range(seeds_per_session):
            cfg = GeConfig(**{**base.__dict__, "mode": mode, "lam": float(lam)})
            faulted, bad = inject_stream(clean, cfg, session_seed(sid) + 1000 * k + 1)
            out.append(make_stream(paths, sid, "ge", faulted=faulted, bad=bad, mode=mode, cfg=cfg))
    return out


def crossover_streams(paths: Paths, split: str, lam: float, base: GeConfig, seeds_per_session: int = 1) -> list:
    """Clean streams of Blocks B-D re-injected in blend mode at lambda with the Python injector."""
    return replay_streams(paths, split, "blend", lam, base, seeds_per_session)


# ---------------------------------------------------------------------------
# Running estimators and scoring
# ---------------------------------------------------------------------------
def estimate(stream: dict, name: str, params: dict | None = None, gru=None) -> dict:
    if name == "gru":
        return gru.run(stream["t_us"], stream["faulted_mm"])
    if name == "imm_oracle":
        if stream["oracle"] is None:  # no injector, so no true probabilities
            n = len(stream["t_us"])
            return {"pos": np.full(n, np.nan), "vel": np.full(n, np.nan), "var": np.full(n, np.nan)}
        return run_estimator(make_estimator(name, params), stream["t_us"], stream["faulted_mm"],
                             transitions=stream["oracle"])
    if name == "genie":
        g = GenieKf(KfParams(**{k: v for k, v in (params or {}).items() if k in ("q", "r", "p0_vel")}),
                    stream["extra_var_m2"],
                    "variance" if stream["mode"] == "variance" else "skip")
        return run_estimator(g, stream["t_us"], stream["faulted_mm"], stream["bad"])
    return run_estimator(make_estimator(name, params), stream["t_us"], stream["faulted_mm"])


def abs_error_mm(stream: dict, est: dict) -> tuple[np.ndarray, np.ndarray]:
    """|estimate - truth| in mm on the scored samples, and the mask used."""
    m = stream["score_mask"] & np.isfinite(est["pos"])
    return np.abs(est["pos"][m] - stream["truth_m"][m]) * 1000.0, m


def p95(x) -> float:
    return float(np.percentile(x, 95)) if len(x) else float("nan")


def session_p95(stream: dict, est: dict, regime: str | None = None) -> float:
    err, m = abs_error_mm(stream, est)
    if regime is not None:
        err = err[stream["regime"][m] == REGIME_CODE[regime]]
    return p95(err)


def objective(streams: list[dict], name: str, params: dict | None = None, gru=None) -> float:
    """Tuning objective: mean over sessions of the per-session 95th-percentile error (mm)."""
    vals = [session_p95(s, estimate(s, name, params, gru)) for s in streams]
    vals = [v for v in vals if np.isfinite(v)]
    return float(np.mean(vals)) if vals else float("inf")


def session_metrics(stream: dict, est: dict) -> list[dict]:
    """Median / p95 / RMSE / NEES per regime (and 'all') for one session."""
    err, m = abs_error_mm(stream, est)
    reg = stream["regime"][m]
    var = est["var"][m]
    nees = (err / 1000.0) ** 2 / var if np.isfinite(var).any() else np.full(len(err), np.nan)
    rows = []
    for regime in REGIMES + ["all"]:
        sel = np.ones(len(err), bool) if regime == "all" else reg == REGIME_CODE[regime]
        e = err[sel]
        rows.append({
            "regime": regime, "n": int(sel.sum()),
            "median_mm": float(np.median(e)) if len(e) else np.nan,
            "p95_mm": p95(e),
            "rmse_mm": float(np.sqrt(np.mean(e ** 2))) if len(e) else np.nan,
            "nees_mean": float(np.nanmean(nees[sel])) if sel.any() and np.isfinite(nees[sel]).any() else np.nan,
        })
    return rows


# ---------------------------------------------------------------------------
# Bootstrap
# ---------------------------------------------------------------------------
def paired_bootstrap(a, b, n_boot: int = 10000, seed: int = 0, ci: float = 0.95) -> dict:
    """Paired bootstrap of the mean difference a - b over sessions.

    a[i] and b[i] are the same session scored with two estimators. Sessions
    (not samples) are resampled with replacement, so the interval reflects
    session-to-session variation. p_two_sided = how often the resampled mean
    difference falls on the other side of 0, doubled (small = a real difference).
    """
    a, b = np.asarray(a, float), np.asarray(b, float)
    ok = np.isfinite(a) & np.isfinite(b)
    d = a[ok] - b[ok]
    n = len(d)
    if n == 0:
        return {"n_sessions": 0, "mean_diff": np.nan, "ci_low": np.nan, "ci_high": np.nan, "p_two_sided": np.nan}
    rng = np.random.default_rng(seed)
    means = d[rng.integers(0, n, size=(n_boot, n))].mean(axis=1)
    lo, hi = np.percentile(means, [100 * (1 - ci) / 2, 100 * (1 + ci) / 2])
    p = 2 * min(np.mean(means <= 0), np.mean(means >= 0))
    return {"n_sessions": int(n), "mean_diff": float(d.mean()), "ci_low": float(lo), "ci_high": float(hi),
            "p_two_sided": float(min(1.0, p))}


def pooled_bootstrap_ci(per_session: list[np.ndarray], stat, n_boot: int = 10000, seed: int = 0,
                        ci: float = 0.95) -> tuple[float, float, float]:
    """Statistic of all samples pooled, with a cluster-bootstrap interval (sessions resampled)."""
    per_session = [np.asarray(x) for x in per_session if len(x)]
    if not per_session:
        return np.nan, np.nan, np.nan
    value = float(stat(np.concatenate(per_session)))
    rng = np.random.default_rng(seed)
    k = len(per_session)
    vals = [float(stat(np.concatenate([per_session[i] for i in rng.integers(0, k, k)]))) for _ in range(n_boot)]
    lo, hi = np.percentile(vals, [100 * (1 - ci) / 2, 100 * (1 + ci) / 2])
    return value, float(lo), float(hi)


# ---------------------------------------------------------------------------
# One condition
# ---------------------------------------------------------------------------
def load_tuned(paths: Paths) -> dict:
    p = paths.tables / "tuned_params.json"
    if not p.exists():
        raise SystemExit(f"{p} not found. Run: python analysis/tune.py --data {paths.data} --results {paths.results}")
    return json.loads(p.read_text())


def load_gru(paths: Paths, condition: str):
    p = paths.processed / "models" / f"gru_{condition}.pt"
    if not p.exists():
        return None
    import torch  # noqa: PLC0415

    from estimators import GruEstimator  # noqa: PLC0415

    return GruEstimator.from_state(torch.load(p, weights_only=False))


def settings_for(tuned: dict, condition: str, tuning: str) -> dict:
    """{estimator: params} for one condition, re-tuned or carried over."""
    if tuning == "retuned":
        return {n: v.get("params") for n, v in tuned.get("conditions", {}).get(condition, {}).items() if n != "gru"}
    return tuned.get("carried_over", {}).get("params", {})


def evaluate_condition(paths: Paths, condition: str, tuned: dict, n_boot: int) -> dict | None:
    streams = condition_streams(paths, condition, "test")
    if not streams:
        return None
    # The GRU is always the condition's own model; it appears only in the re-tuned results.
    grus = {"retuned": load_gru(paths, condition), "carried_over": None}
    rows, summary, paired = [], [], []
    out_dir = paths.processed / "estimates" / condition
    out_dir.mkdir(parents=True, exist_ok=True)
    for tuning in TUNINGS:
        params = settings_for(tuned, condition, tuning)
        gru = grus[tuning]
        names = [n for n in ESTIMATORS if (n == "gru" and gru is not None) or (n != "gru" and n in params)]
        names.append("genie")
        t_rows, errors = [], {n: {r: [] for r in REGIMES + ["all"]} for n in names}
        for s in streams:
            per_sample = {"t_us": s["t_us"], "t_s": s["t_s"], "truth_m": s["truth_m"], "faulted_mm": s["faulted_mm"],
                          "bad": s["bad"].astype(int), "regime": s["regime"], "scored": s["score_mask"].astype(int)}
            for n in names:
                p = params.get("kf" if n == "genie" else n)
                est = estimate(s, n, p, gru)
                per_sample[f"{n}_pos"] = est["pos"]
                per_sample[f"{n}_var"] = est["var"]
                if "p_bad" in est:
                    per_sample[f"{n}_p_bad"] = est["p_bad"]
                for r in session_metrics(s, est):
                    t_rows.append({"condition": condition, "tuning": tuning, "session_id": s["sid"], "estimator": n,
                                   **r})
                err, m = abs_error_mm(s, est)
                reg = s["regime"][m]
                for r in REGIMES:
                    errors[n][r].append(err[reg == REGIME_CODE[r]])
                errors[n]["all"].append(err)
            if tuning == "retuned":
                pd.DataFrame(per_sample).to_csv(out_dir / f"{s['sid']}.csv", index=False)
        per_session = pd.DataFrame(t_rows)
        rows.append(per_session)
        for n in names:
            for r in REGIMES + ["all"]:
                if not any(len(e) for e in errors[n][r]):
                    continue
                med, med_lo, med_hi = pooled_bootstrap_ci(errors[n][r], np.median, n_boot)
                q, q_lo, q_hi = pooled_bootstrap_ci(errors[n][r], lambda x: np.percentile(x, 95), n_boot)
                ps = per_session[(per_session.estimator == n) & (per_session.regime == r)]
                summary.append({"condition": condition, "tuning": tuning, "estimator": n, "regime": r,
                                "n_sessions": len(streams), "n_samples": int(sum(len(e) for e in errors[n][r])),
                                "median_mm": med, "median_ci_low": med_lo, "median_ci_high": med_hi,
                                "p95_mm": q, "p95_ci_low": q_lo, "p95_ci_high": q_hi,
                                "rmse_mm": float(np.sqrt(np.mean(np.concatenate(errors[n][r]) ** 2))),
                                "nees_mean": float(ps["nees_mean"].mean()) if ps["nees_mean"].notna().any() else np.nan,
                                "mean_session_p95_mm": float(ps["p95_mm"].mean())})
        real = [n for n in names if n != "genie"]
        for r in REGIMES + ["all"]:
            for i, a in enumerate(real):
                for b in real[i + 1:]:
                    pa = per_session[(per_session.estimator == a) & (per_session.regime == r)].set_index("session_id")
                    pb = per_session[(per_session.estimator == b) & (per_session.regime == r)].set_index("session_id")
                    common = pa.index.intersection(pb.index)
                    res = paired_bootstrap(pa.loc[common, "p95_mm"], pb.loc[common, "p95_mm"], n_boot)
                    paired.append({"condition": condition, "tuning": tuning, "regime": r, "estimator_a": a,
                                   "estimator_b": b, "metric": "p95_mm (a - b)", **res})
    return {"per_session": pd.concat(rows), "summary": pd.DataFrame(summary), "paired": pd.DataFrame(paired),
            "streams": streams}


# ---------------------------------------------------------------------------
# Crossover sweep
# ---------------------------------------------------------------------------
def find_crossover(lams, diff) -> float:
    """First lambda where diff = p95(GRU) - p95(best classical) goes from >= 0 to < 0
    (linear interpolation). NaN if it never crosses."""
    lams, diff = np.asarray(lams, float), np.asarray(diff, float)
    for i in range(len(lams) - 1):
        a, b = diff[i], diff[i + 1]
        if np.isfinite(a) and np.isfinite(b) and a >= 0 > b:
            return float(lams[i] + (lams[i + 1] - lams[i]) * a / (a - b))
    return float("nan")


def crossover_sweep(paths: Paths, lams, gru_kwargs: dict, n_boot: int, q_shared: float, r_fixed: float,
                    regime: str = "burst", seed: int = 0) -> tuple[pd.DataFrame, dict]:
    from injector import count_transitions  # noqa: PLC0415
    from tune import tune_classical, tune_gru  # noqa: PLC0415

    base = sweep_base_config(paths)
    rows, per_session = [], {}
    for lam in lams:
        print(f"  crossover: lambda = {lam:.2f}")
        train = crossover_streams(paths, "train", lam, base, seeds_per_session=2)
        val = crossover_streams(paths, "validation", lam, base)
        test = crossover_streams(paths, "test", lam, base)
        if not (train and val and test):
            raise SystemExit("The crossover sweep needs Block B-D sessions in train, validation and test.")
        transitions = count_transitions([s["bad"] for s in train])
        val_scores, test_p95 = {}, {}
        for name in EVALUATED:
            best = tune_classical(val, name, q_shared, r_fixed, transitions)
            val_scores[name] = best["objective"]
            test_p95[name] = [session_p95(s, estimate(s, name, best["params"]), regime) for s in test]
        gru = tune_gru(train, val, seed=seed, **gru_kwargs)
        val_scores["gru"] = gru.info["val_score"]
        test_p95["gru"] = [session_p95(s, estimate(s, "gru", gru=gru), regime) for s in test]
        best_classical = min(EVALUATED, key=lambda n: val_scores[n])  # chosen on validation
        per_session[lam] = {"gru": test_p95["gru"], "classical": test_p95[best_classical]}
        for name, vals in test_p95.items():
            rows.append({"lambda": lam, "estimator": name, "test_mean_session_p95_mm": float(np.nanmean(vals)),
                         "validation_objective": val_scores[name], "best_classical": name == best_classical,
                         "regime": regime, "n_test_sessions": len(test)})
    table = pd.DataFrame(rows)

    ll = list(lams)
    diff = [np.nanmean(per_session[a]["gru"]) - np.nanmean(per_session[a]["classical"]) for a in ll]
    crossing = find_crossover(ll, diff)
    # Bootstrap: resample test sessions (the same ones at every lambda: paired).
    rng = np.random.default_rng(seed)
    k = len(per_session[ll[0]]["gru"])
    boots = []
    for _ in range(n_boot):
        idx = rng.integers(0, k, k)
        d = [np.nanmean(np.asarray(per_session[a]["gru"])[idx]) - np.nanmean(np.asarray(per_session[a]["classical"])[idx])
             for a in ll]
        boots.append(find_crossover(ll, d))
    boots = np.array(boots)
    found = np.isfinite(boots)
    summary = {
        "lambda_crossover": crossing,
        # An interval only makes sense if the curves cross at all (point estimate exists).
        "ci95": [float(np.percentile(boots[found], 2.5)), float(np.percentile(boots[found], 97.5))]
        if np.isfinite(crossing) and found.sum() > 10 else None,
        "fraction_of_resamples_with_crossing": float(found.mean()),
        "diff_gru_minus_best_classical_mm": dict(zip(map(float, ll), map(float, diff))),
        "regime": regime, "q_shared": q_shared, "r_fixed": r_fixed, "base_ge_config": base.__dict__,
        "definition": "first lambda where mean test p95(GRU) - p95(best classical, chosen on validation) "
                      "changes from >= 0 to < 0; linear interpolation",
    }
    return table, summary


def sweep_base_config(paths: Paths) -> GeConfig:
    """GE settings for the sweep: those of the first Block C/D session (the paper's assumed
    parameters), else the firmware defaults."""
    for split in ("train", "validation", "test"):
        for sid in sessions_in(paths, split, ["C", "D"]):
            return ge_config_from_meta(load_meta(paths, sid))
    return GeConfig()


# ---------------------------------------------------------------------------
# Test card numbers (docs/test_card.md)
# ---------------------------------------------------------------------------
def test_card_values(paths: Paths, results: dict, fail_threshold_mm: float | None) -> dict:
    out = {}
    for cond, res in results.items():
        streams = res["streams"]
        n_bursts = int(sum(np.sum(np.diff(np.r_[0, s["bad"].astype(int)]) == 1) for s in streams))
        bad_s = float(sum(np.sum(np.diff(s["t_us"], prepend=s["t_us"][0])[s["bad"]]) / 1e6 for s in streams))
        metas = [s["meta"] for s in streams]
        fw = [m.get("firmware", {}) for m in metas]
        temps = [m.get("user", {}).get("temp_c") for m in metas if m.get("user", {}).get("temp_c") is not None]
        volts = [m.get("user", {}).get("supply_v") for m in metas if m.get("user", {}).get("supply_v") is not None]
        entry = {
            "fault_type_and_source": CONDITIONS[cond]["title"],
            "fault_rate": sorted({tuple(f.get(k) for k in ("entry_far", "entry_near", "persist_far", "persist_near",
                                                           "r_ref_mm", "d_src_mm", "sample_period_s")) for f in fw},
                                 key=str),
            "fault_rate_fields": "(entry_far, entry_near, persist_far, persist_near) per 0.1 s; r_ref_mm, "
                                 "d_src_mm; sample_period_s",
            "exposure": {"test_sessions": len(streams), "bursts": n_bursts, "total_bad_time_s": bad_s,
                         "total_time_s": float(sum((s["t_us"][-1] - s["t_us"][0]) / 1e6 for s in streams))},
            "operating_state": {"modes": sorted({f.get("mode", "?") for f in fw}),
                                "timing_budget_us": sorted({f.get("timing_budget_us", "?") for f in fw}),
                                "supply_v": [min(volts), max(volts)] if volts else None},
            "ambient_temperature_c": [min(temps), max(temps)] if temps else None,
            "device_identity": {"firmware_commits": sorted({f.get("fw_commit", "?") for f in fw}),
                                "analysis_commit": git_commit()},
        }
        if fail_threshold_mm is not None:
            ps = res["per_session"]
            burst = ps[(ps.regime == "burst") & (ps.tuning == "retuned")]
            entry["failure_criterion"] = {
                "definition": f"session burst-regime p95 error > {fail_threshold_mm} mm",
                "fraction_failing": burst.assign(fail=burst.p95_mm > fail_threshold_mm)
                .groupby("estimator")["fail"].mean().to_dict()}
        out[cond] = entry
    return out


# ---------------------------------------------------------------------------
def run(paths: Paths, conditions: list[str], n_boot: int = 10000, crossover: bool = False, lams=None,
        gru_kwargs: dict | None = None, fail_threshold_mm: float | None = None):
    tuned = load_tuned(paths)
    all_sessions = []
    results = {}
    for cond in conditions:
        res = evaluate_condition(paths, cond, tuned, n_boot)
        if res is None:
            print(f"{cond}: no test sessions, skipped")
            continue
        check_no_synthetic_in_results(paths, [s["meta"] for s in res["streams"]])
        results[cond] = res
        all_sessions += [s["sid"] for s in res["streams"]]
        for tuning in TUNINGS:
            best = res["summary"].query(f"regime == 'burst' and estimator != 'genie' and tuning == '{tuning}'")
            best = best.sort_values("p95_mm")
            print(f"{cond:9s} {tuning:12s} {len(res['streams'])} test sessions; burst p95 (mm): "
                  + ", ".join(f"{r.estimator}={r.p95_mm:.2f}" for r in best.itertuples()))
    if not results:
        raise SystemExit("Nothing to evaluate: no test sessions with ground truth.")
    paths.make_output_dirs()
    pd.concat([r["per_session"] for r in results.values()]).to_csv(paths.tables / "metrics_per_session.csv", index=False)
    pd.concat([r["summary"] for r in results.values()]).to_csv(paths.tables / "metrics_summary.csv", index=False)
    pd.concat([r["paired"] for r in results.values()]).to_csv(paths.tables / "paired_bootstrap.csv", index=False)
    # The paper's comparison: burst-regime p95 with carried-over vs re-tuned settings, side by side.
    summ = pd.concat([r["summary"] for r in results.values()])
    burst = summ[(summ.regime == "burst") & summ.estimator.isin(EVALUATED)]  # classical filters only
    side = burst.pivot_table(index=["condition", "estimator"], columns="tuning", values="p95_mm").reset_index()
    side.to_csv(paths.tables / "carried_over_vs_retuned.csv", index=False)

    # Are the Python estimators still identical to the firmware on the recordings?
    fw_rows = []
    for res in results.values():
        for s in res["streams"]:
            fw_rows.append({"session_id": s["sid"], **check_against_firmware(s["df"], s["meta"])})
    fw = pd.DataFrame(fw_rows)
    fw.to_csv(paths.tables / "firmware_check.csv", index=False)
    worst = np.nanmax(fw.drop(columns="session_id").to_numpy()) if len(fw) else np.nan
    if np.isfinite(worst) and worst > 1e-6:
        print(f"WARNING: Python and firmware estimates differ by up to {worst:.3g} m (see firmware_check.csv)")

    from common import provenance  # noqa: PLC0415

    write_json(paths.tables / "test_card_values.json", test_card_values(paths, results, fail_threshold_mm))
    write_json(paths.tables / "evaluate_provenance.json",
               {"n_boot": n_boot, "warmup_s": WARMUP_S, "recovery_s": RECOVERY_S, **provenance(paths, all_sessions)})

    if crossover:
        table, summary = crossover_sweep(paths, lams, gru_kwargs or {}, n_boot, tuned["q_shared"], tuned["r_fixed"])
        table.to_csv(paths.tables / "crossover.csv", index=False)
        write_json(paths.tables / "crossover.json", summary)
        print(f"crossover lambda = {summary['lambda_crossover']:.3f}  (95 % CI {summary['ci95']}; curves cross in "
              f"{100 * summary['fraction_of_resamples_with_crossing']:.0f} % of bootstrap resamples)")
    return results


# GRU search settings. FULL follows the paper (grid scaled for a laptop CPU,
# see docs/methods.md); QUICK is the --quick preset for a first look on a laptop.
GRU_PRESETS = {
    "full": {"grid": "1:64:1e-3,2:64:1e-3,1:128:1e-3,2:128:1e-3", "max_epochs": 250, "patience": 30, "steps": 25},
    "quick": {"grid": "1:64:1e-3", "max_epochs": 60, "patience": 10, "steps": 25},
}
LAMBDAS = {"full": "0,0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.75,0.8,0.85,0.9,1.0", "quick": "0,0.5,0.75,0.9,1.0"}


def add_gru_args(ap):
    """GRU grid-search options, shared by tune.py and evaluate.py. Explicit options override --quick."""
    ap.add_argument("--quick", action="store_true",
                    help="laptop preset: GRU grid 1 layer x 64 units, up to 60 epochs (patience 10); "
                         "crossover at lambda 0, 0.5, 0.75, 0.9, 1.0. Not the paper's protocol")
    ap.add_argument("--gru-grid", help=f"layers:hidden:learning-rate points (default {GRU_PRESETS['full']['grid']})")
    ap.add_argument("--gru-max-epochs", type=int, help="default 250 (--quick: 60)")
    ap.add_argument("--gru-patience", type=int, help="default 30 (--quick: 10)")
    ap.add_argument("--gru-steps", type=int, help="batches per epoch (default 25)")
    return ap


def gru_kwargs_from_args(args) -> dict:
    preset = GRU_PRESETS["quick" if getattr(args, "quick", False) else "full"]
    args.gru_grid = args.gru_grid or preset["grid"]
    args.gru_max_epochs = args.gru_max_epochs or preset["max_epochs"]
    args.gru_patience = args.gru_patience or preset["patience"]
    args.gru_steps = args.gru_steps or preset["steps"]
    grid = []
    for item in args.gru_grid.split(","):
        layers, hidden, lr = item.split(":")
        grid.append({"layers": int(layers), "hidden": int(hidden), "lr": float(lr)})
    return {"grid": grid, "max_epochs": args.gru_max_epochs, "patience": args.gru_patience,
            "steps_per_epoch": args.gru_steps}


def main(argv=None):
    ap = add_path_args(argparse.ArgumentParser(description=__doc__,
                                               formatter_class=argparse.RawDescriptionHelpFormatter))
    ap.add_argument("--conditions", default=",".join(CONDITIONS), help="comma-separated (default: all)")
    ap.add_argument("--n-boot", type=int, default=10000, help="bootstrap resamples (default 10000)")
    ap.add_argument("--crossover", action="store_true", help="also run the crossover sweep")
    ap.add_argument("--lambdas", help=f"crossover points (default {LAMBDAS['full']}; --quick: {LAMBDAS['quick']})")
    add_gru_args(ap)
    ap.add_argument("--fail-threshold-mm", type=float, default=None,
                    help="test card failure criterion: burst p95 above this (mm)")
    args = ap.parse_args(argv)
    run(Paths(args.data, args.results), args.conditions.split(","), args.n_boot, args.crossover,
        [float(a) for a in (args.lambdas or LAMBDAS["quick" if args.quick else "full"]).split(",")],
        gru_kwargs_from_args(args),
        args.fail_threshold_mm)


if __name__ == "__main__":
    main()
