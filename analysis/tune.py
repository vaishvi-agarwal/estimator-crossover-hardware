"""Tune the estimators on the VALIDATION sessions only, following the paper's protocol.

    python analysis/tune.py

Fixed for every filter and every condition (as in the paper):
  * R, the reading-noise variance: the known nominal noise, measured in
    Block A (results/tables/noise_model.json). Not tuned.
  * q, the process noise: ONE shared value, chosen once on the validation
    sessions of Block B (clean swings, no faults) with the plain KF, then
    fixed (the paper fixed q = 0.01 in its own units). `--q` sets it by hand.

Re-tuned per condition, by grid search on the validation sessions,
minimising the mean over sessions of the per-session 95th-percentile error:
  * gated KF: gate confidence {0.95, 0.99, 0.999} x coasting timeout
    {3, 5, 10, 20, 50} steps at 10 Hz = {0.3, 0.5, 1.0, 2.0, 5.0} s
  * IMM-oracle and IMM-est: inflation kappa_hat {5, 50, 100, 200}
  * KF: nothing (R and q are fixed)
Switch probabilities: IMM-oracle uses the injector's TRUE probabilities
(time-varying with rho); IMM-est counts them from the training sessions'
fault sequences.

The GRU is trained on the TRAINING sessions of each condition and selected
by grid search on validation (--gru-* options).

Carried-over settings (classical filters only): the paper reports results
both with settings carried over unchanged and with settings re-tuned for
each condition. The carried-over settings are those tuned on the primary
condition (inflated variance, Block C), applied unchanged to every other
condition; if there are no Block C validation sessions, the defaults are
used. The GRU is never carried over: as in the paper, a fresh GRU is trained
on each condition's own training sessions and selected on its own
validation sessions.

Options beyond the paper's protocol:
  --quick          laptop preset (smaller GRU search; see README)
  --estimate-time  time 2 GRU epochs on one condition and estimate the full run
  --extended-grid  sensitivity check of the gated KF with a wider grid
                   (deviation from paper; written to a separate file, never
                   used by evaluate.py)

The test sessions are never opened here.

Writes:
  <results>/tables/tuned_params.json
  <data>/processed/models/gru_<condition>.pt
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np
from scipy import stats

sys.path.insert(0, str(Path(__file__).resolve().parent))
from common import Paths, add_path_args, check_no_synthetic_in_results, provenance, sessions_in, write_json  # noqa: E402
from estimators import EVALUATED, default_params, train_gru  # noqa: E402
from evaluate import (CONDITIONS, add_gru_args, condition_streams, ge_config_from_meta, gru_kwargs_from_args,  # noqa: E402
                      make_stream, objective)
from injector import count_transitions, inject_stream  # noqa: E402

GATE_CONFIDENCES = (0.95, 0.99, 0.999)
COAST_STEPS_10HZ = (3, 5, 10, 20, 50)          # the paper's grid, in 0.1 s steps
KAPPA_HAT_GRID = (5.0, 50.0, 100.0, 200.0)
Q_GRID = np.logspace(-2, 3, 26)                 # m^2/s^3, for the shared q
PRIMARY = "variance"                            # carried-over settings come from this condition


def gate_threshold(confidence: float) -> float:
    """Chi-square threshold with 1 degree of freedom for a gate confidence."""
    return float(stats.chi2.ppf(confidence, 1))


def grid(name: str) -> list[dict]:
    """The settings tried for one filter (empty dict = nothing to tune)."""
    if name == "gated":
        return [{"gate": gate_threshold(c), "coast_s": n / 10.0, "gate_confidence": c, "coast_steps_10hz": n}
                for c in GATE_CONFIDENCES for n in COAST_STEPS_10HZ]
    if name in ("imm_oracle", "imm_est"):
        return [{"r_bad_factor": k} for k in KAPPA_HAT_GRID]
    return [{}]


def r_from_noise_model(paths: Paths | None) -> float | None:
    """Reading-noise variance R (m^2) from Block A at the typical distance, if available."""
    if paths is None:
        return None
    p = paths.tables / "noise_model.json"
    if not p.exists():
        return None
    from noise_characterisation import sigma_at  # noqa: PLC0415

    m = json.loads(p.read_text())
    sigma_mm = sigma_at(m["sigma_vs_distance"], 300.0)
    return (sigma_mm / 1000.0) ** 2


def choose_shared_q(paths: Paths, r: float) -> tuple[float, list]:
    """The one process-noise value for every filter: best plain-KF objective on the
    validation sessions of Block B (clean swings)."""
    val = [make_stream(paths, s, "ge") for s in sessions_in(paths, "validation", ["B"])]
    if not val:
        raise SystemExit("Choosing q needs Block B sessions in the validation split (or pass --q).")
    base = {**default_params("kf"), "r": r}
    scores = [(float(q), objective(val, "kf", {**base, "q": float(q)})) for q in Q_GRID]
    return min(scores, key=lambda t: t[1])[0], scores


def base_params(name: str, q: float, r: float, transitions=None) -> dict:
    p = default_params(name)
    p["q"], p["r"] = q, r
    if name == "imm_est" and transitions is not None:
        p["p_nb"], p["p_bn"] = transitions
    return p


def tune_classical(val_streams: list[dict], name: str, q: float, r: float, transitions=None) -> dict:
    """Exhaustive grid search over grid(name). Returns the best params, its objective,
    and the score of every grid point."""
    base = base_params(name, q, r, transitions)
    tried = []
    for g in grid(name):
        p = {**base, **{k: v for k, v in g.items() if k in base}}
        tried.append({"setting": g, "objective": objective(val_streams, name, p), "params": p})
    best = min(tried, key=lambda t: t["objective"])
    return {"params": best["params"], "setting": best["setting"], "objective": best["objective"],
            "grid": [{"setting": t["setting"], "objective": t["objective"]} for t in tried]}


def defaults_for(name: str, q: float, r: float) -> dict:
    """Default settings (used for carried-over results if Block C cannot be tuned)."""
    return base_params(name, q, r)


def tune_gru(train_streams, val_streams, seed: int = 0, grid=None, max_epochs: int = 250, patience: int = 30,
             steps_per_epoch: int = 25, verbose=False):
    return train_gru(train_streams, val_streams, lambda est, vs: objective(vs, "gru", gru=est), grid=grid,
                     max_epochs=max_epochs, patience=patience, steps_per_epoch=steps_per_epoch, seed=seed,
                     verbose=verbose)


def augmented_training_streams(paths: Paths, condition: str, streams: list[dict], extra_seeds: int) -> list[dict]:
    """More GRU training data: re-inject each TRAINING session's clean stream with new
    seeds and the same injector settings (only for recorded injected-fault conditions)."""
    c = CONDITIONS[condition]
    if c["label"] != "ge" or "replay" in c or extra_seeds <= 0:
        return list(streams)
    from logger import session_seed  # noqa: PLC0415

    out = list(streams)
    for s in streams:
        cfg = ge_config_from_meta(s["meta"])
        clean = s["df"]["clean_mm"].to_numpy(dtype=float)
        for k in range(extra_seeds):
            faulted, bad = inject_stream(clean, cfg, session_seed(s["sid"]) + 7919 * (k + 1))
            out.append(make_stream(paths, s["sid"], "ge", faulted=faulted, bad=bad, mode=cfg.mode, cfg=cfg))
    return out


def run(paths: Paths, conditions: list[str], seed: int, gru_kwargs: dict, extra_seeds: int,
        q_fixed: float | None = None, skip_gru: bool = False) -> dict:
    import torch  # noqa: PLC0415

    r = r_from_noise_model(paths)
    if r is None:
        r = default_params("kf")["r"]
        print(f"WARNING: no noise_model.json (run noise_characterisation.py); using the default R = {r:g} m^2")
    if q_fixed is None:
        q_shared, q_scores = choose_shared_q(paths, r)
        print(f"shared q = {q_shared:.4g} m^2/s^3 (chosen on Block B validation sessions); R = {r:.3g} m^2 (Block A)")
    else:
        q_shared, q_scores = q_fixed, []
    out = {"objective": "mean over validation sessions of per-session p95 |error| (mm)", "seed": seed,
           "r_fixed": r, "r_source": "Block A noise_model.json at 300 mm", "q_shared": q_shared,
           "q_search": {"sessions": "Block B validation", "estimator": "kf", "scores": q_scores},
           "grids": {"gate_confidence": GATE_CONFIDENCES, "coast_steps_10hz": COAST_STEPS_10HZ,
                     "coast_s": [n / 10.0 for n in COAST_STEPS_10HZ], "kappa_hat": KAPPA_HAT_GRID},
           "gru_search": dict(gru_kwargs), "conditions": {}}
    used = []
    for cond in conditions:
        val = condition_streams(paths, cond, "validation")
        train = condition_streams(paths, cond, "train")
        if not val:
            print(f"{cond}: no validation sessions, skipped")
            continue
        check_no_synthetic_in_results(paths, [s["meta"] for s in val + train])
        used += [s["sid"] for s in val + train]
        transitions = count_transitions([s["bad"] for s in train]) if train else None
        res = {}
        for name in EVALUATED:
            if name == "imm_oracle" and any(s["oracle"] is None for s in val):
                continue  # no injector, so no true probabilities (physical / live faults)
            best = tune_classical(val, name, q_shared, r, transitions)
            best["validation_sessions"] = [s["sid"] for s in val]
            if name == "imm_est":
                best["transitions_counted"] = {"p_nb": transitions[0], "p_bn": transitions[1],
                                               "from": [s["sid"] for s in train]} if transitions else None
            res[name] = best
            print(f"{cond:9s} {name:10s} objective {best['objective']:.3f} mm  {best['setting']}")
        if not skip_gru and train:
            gru = tune_gru(augmented_training_streams(paths, cond, train, extra_seeds), val, seed, **gru_kwargs)
            model_dir = paths.processed / "models"
            model_dir.mkdir(parents=True, exist_ok=True)
            torch.save(gru.state(), model_dir / f"gru_{cond}.pt")
            res["gru"] = {"objective": gru.info["val_score"], "params": gru.info,
                          "training_sessions": [s["sid"] for s in train],
                          "validation_sessions": [s["sid"] for s in val]}
            print(f"{cond:9s} gru        objective {gru.info['val_score']:.3f} mm  layers={gru.info['layers']} "
                  f"hidden={gru.info['hidden']} lr={gru.info['lr']} epoch={gru.info['epoch']}")
        out["conditions"][cond] = res

    primary = out["conditions"].get(PRIMARY)
    out["carried_over"] = {
        "from": PRIMARY if primary else "defaults",
        "params": {n: (primary[n]["params"] if primary and n in primary else defaults_for(n, q_shared, r))
                   for n in EVALUATED},
        "gru": "never carried over: each condition uses its own GRU",
    }
    out["provenance"] = provenance(paths, sorted(set(used)))
    paths.make_output_dirs()
    write_json(paths.tables / "tuned_params.json", out)
    print(f"Wrote {paths.tables / 'tuned_params.json'}")
    return out


# ---------------------------------------------------------------------------
# Optional: extended grid (sensitivity, deviation from the paper)
# ---------------------------------------------------------------------------
EXT_GATE_CONFIDENCES = (0.95, 0.99, 0.999, 0.9999)
EXT_COAST_STEPS_10HZ = (1, 2, 3, 5, 10, 20, 50)  # adds 0.1 s and 0.2 s


def extended_gated_grid() -> list[dict]:
    return [{"gate": gate_threshold(c), "coast_s": n / 10.0, "gate_confidence": c, "coast_steps_10hz": n}
            for c in EXT_GATE_CONFIDENCES for n in EXT_COAST_STEPS_10HZ]


def run_extended_grid(paths: Paths, conditions: list[str]) -> dict:
    """Sensitivity check, NOT the primary analysis: re-tune the gated KF on each
    condition's validation sessions with a wider grid (gate confidence up to
    0.9999, coasting timeouts down to 0.1 s), and score both the paper-grid
    choice and the extended-grid choice on the test sessions.
    Uses q and R from tuned_params.json (run the normal tune.py first).
    Writes <results>/tables/sensitivity_extended_grid.json; nothing else."""
    from evaluate import REGIME_CODE, abs_error_mm, estimate  # noqa: PLC0415

    tp = paths.tables / "tuned_params.json"
    if not tp.exists():
        raise SystemExit("Run tune.py without --extended-grid first (it fixes q and R).")
    tuned = json.loads(tp.read_text())
    q, r = tuned["q_shared"], tuned["r_fixed"]
    out = {"label": "SENSITIVITY CHECK - deviation from paper (extended grid); not used by evaluate.py",
           "grid": {"gate_confidence": EXT_GATE_CONFIDENCES, "coast_s": [n / 10.0 for n in EXT_COAST_STEPS_10HZ]},
           "q_shared": q, "r_fixed": r, "conditions": {}}
    base = base_params("gated", q, r)
    for cond in conditions:
        val = condition_streams(paths, cond, "validation")
        test = condition_streams(paths, cond, "test")
        if not val or not test:
            continue
        check_no_synthetic_in_results(paths, [s["meta"] for s in val + test])
        tried = [{"setting": g, "objective": objective(val, "gated", {**base, "gate": g["gate"], "coast_s": g["coast_s"]})}
                 for g in extended_gated_grid()]
        best_ext = min(tried, key=lambda t: t["objective"])
        paper = tune_classical(val, "gated", q, r)
        entry = {"paper_grid": {"setting": paper["setting"], "validation": paper["objective"]},
                 "extended_grid": {"setting": best_ext["setting"], "validation": best_ext["objective"]},
                 "extended_grid_scores": tried}
        for key, setting in (("paper_grid", paper["setting"]), ("extended_grid", best_ext["setting"])):
            p = {**base, "gate": setting["gate"], "coast_s": setting["coast_s"]}
            for regime in ("nominal", "burst", "all"):
                errs = []
                for s in test:
                    err, m = abs_error_mm(s, estimate(s, "gated", p))
                    errs.append(err if regime == "all" else err[s["regime"][m] == REGIME_CODE[regime]])
                e = np.concatenate(errs)
                entry[key][f"test_p95_{regime}_mm"] = float(np.percentile(e, 95)) if len(e) else None
        out["conditions"][cond] = entry
        print(f"{cond:9s} paper grid {paper['setting']['gate_confidence']}/{paper['setting']['coast_s']} s "
              f"-> extended {best_ext['setting']['gate_confidence']}/{best_ext['setting']['coast_s']} s; "
              f"test burst p95 {entry['paper_grid']['test_p95_burst_mm']:.2f} -> "
              f"{entry['extended_grid']['test_p95_burst_mm']:.2f} mm")
    write_json(paths.tables / "sensitivity_extended_grid.json", out)
    print(f"Wrote {paths.tables / 'sensitivity_extended_grid.json'} (deviation from paper; not used by evaluate.py)")
    return out


# ---------------------------------------------------------------------------
# Optional: estimate the run time
# ---------------------------------------------------------------------------
def _samples(streams) -> int:
    return int(sum(len(s["t_us"]) for s in streams))


def estimate_time(paths: Paths, condition: str, gru_kwargs: dict, n_lambdas: int, conditions: list[str],
                  epochs: int = 2) -> dict:
    """Train each GRU grid point for `epochs` epochs on one condition, time it, time the
    classical grids, and extrapolate to the whole tuning run and the crossover sweep."""
    import time  # noqa: PLC0415

    r = r_from_noise_model(paths) or default_params("kf")["r"]
    train = condition_streams(paths, condition, "train")
    val = condition_streams(paths, condition, "validation")
    if not train or not val:
        raise SystemExit(f"--estimate-time needs train and validation sessions for '{condition}'.")
    per_point = []
    for g in gru_kwargs["grid"]:
        t0 = time.perf_counter()
        train_gru(train, val, lambda est, vs: objective(vs, "gru", gru=est), grid=[g], max_epochs=epochs,
                  patience=epochs + 1, steps_per_epoch=gru_kwargs["steps_per_epoch"])
        per_epoch = (time.perf_counter() - t0) / epochs
        per_point.append({**g, "s_per_epoch": per_epoch, "val_samples": _samples(val)})
        print(f"  GRU {g}: {per_epoch:.1f} s per epoch ({_samples(val)} validation samples)")
    t0 = time.perf_counter()
    n_classical = 0
    for name in EVALUATED:
        for g in grid(name)[:2]:
            objective(val, name, {**base_params(name, 1.0, r), **{k: v for k, v in g.items() if k != "gate_confidence"
                                                                   and k != "coast_steps_10hz"}})
            n_classical += 1
    s_per_eval_sample = (time.perf_counter() - t0) / n_classical / _samples(val)
    evals_per_condition = sum(len(grid(n)) for n in EVALUATED)

    def condition_cost(val_samples: int) -> tuple[float, float]:
        # GRU: training time per epoch is fixed (steps x batch); validation scoring scales with val size.
        gru_epoch = sum(p["s_per_epoch"] * (0.5 + 0.5 * val_samples / max(1, p["val_samples"])) for p in per_point)
        classical = evals_per_condition * val_samples * s_per_eval_sample
        return gru_epoch, classical

    rows, worst, typical = [], 0.0, 0.0
    typical_epochs = min(gru_kwargs["max_epochs"], 3 * gru_kwargs["patience"])
    for cond in conditions:
        vs = condition_streams(paths, cond, "validation")
        if not vs:
            continue
        ge, cl = condition_cost(_samples(vs))
        worst += ge * gru_kwargs["max_epochs"] + cl
        typical += ge * typical_epochs + cl
        rows.append(cond)
    from evaluate import crossover_streams, sweep_base_config  # noqa: PLC0415

    xv = _samples(crossover_streams(paths, "validation", 0.5, sweep_base_config(paths)))
    ge, cl = condition_cost(xv)
    x_worst = n_lambdas * (ge * gru_kwargs["max_epochs"] + cl)
    x_typical = n_lambdas * (ge * typical_epochs + cl)
    est = {"measured_on": condition, "epochs_timed": epochs, "gru_grid": per_point,
           "classical_s_per_validation_sample": s_per_eval_sample, "conditions": rows,
           "tune_worst_case_h": worst / 3600, "tune_if_early_stop_at_%d_epochs_h" % typical_epochs: typical / 3600,
           "crossover_lambdas": n_lambdas, "crossover_worst_case_h": x_worst / 3600,
           "crossover_if_early_stop_at_%d_epochs_h" % typical_epochs: x_typical / 3600}
    print(f"\nEstimated time on this computer ({len(rows)} conditions, {n_lambdas} crossover points):")
    print(f"  tune.py           worst case (every grid point runs {gru_kwargs['max_epochs']} epochs): "
          f"{worst / 3600:.1f} h;  if early stopping halts near {typical_epochs} epochs: {typical / 3600:.1f} h")
    print(f"  crossover sweep   worst case: {x_worst / 3600:.1f} h;  if early stopping halts near "
          f"{typical_epochs} epochs: {x_typical / 3600:.1f} h")
    print("  (Early stopping usually ends sooner than the worst case; the second number is a rough guess. "
          "Use --quick for a shorter run.)")
    return est


def main(argv=None):
    ap = add_path_args(argparse.ArgumentParser(description=__doc__,
                                               formatter_class=argparse.RawDescriptionHelpFormatter))
    ap.add_argument("--conditions", default=",".join(CONDITIONS))
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--q", type=float, help="use this shared q instead of choosing it on Block B validation")
    add_gru_args(ap)
    ap.add_argument("--extra-seeds", type=int, default=2, help="re-injected copies of each training session for the GRU")
    ap.add_argument("--skip-gru", action="store_true")
    ap.add_argument("--extended-grid", action="store_true",
                    help="sensitivity check (deviation from paper): wider gated-KF grid, separate output file")
    ap.add_argument("--estimate-time", metavar="CONDITION", nargs="?", const="variance",
                    help="time 2 GRU epochs on one condition (default variance) and estimate the full run")
    args = ap.parse_args(argv)
    paths = Paths(args.data, args.results)
    conditions = args.conditions.split(",")
    gru_kwargs = gru_kwargs_from_args(args)
    if args.estimate_time:
        from evaluate import LAMBDAS  # noqa: PLC0415

        n_l = len(LAMBDAS["quick" if args.quick else "full"].split(","))
        estimate_time(paths, args.estimate_time, gru_kwargs, n_l, conditions)
        return
    if args.extended_grid:
        run_extended_grid(paths, conditions)
        return
    run(paths, conditions, args.seed, gru_kwargs, args.extra_seeds, args.q, args.skip_gru)


if __name__ == "__main__":
    main()
