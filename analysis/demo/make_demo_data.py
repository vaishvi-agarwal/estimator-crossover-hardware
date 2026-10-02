"""Make SYNTHETIC sessions that look like the firmware's output, for checking that
every analysis script runs end to end before the hardware exists.

    python analysis/demo/make_demo_data.py --out analysis/demo/data

THESE ARE NOT MEASUREMENTS. Every sidecar says "synthetic": true, every figure
made from them is stamped "SYNTHETIC DEMO DATA", and the analysis scripts
refuse to write them into results/. The numbers they produce say nothing about
the real sensor or the paper's questions.

What is simulated (deliberately simple):
  * a damped pendulum seen by a distance sensor (rest 280-320 mm, swing 50-90 mm),
  * sensor noise that grows with distance, with heavier tails than a Gaussian,
    rounded to whole mm, sample times of 33 ms with jitter,
  * Blocks C/D/F: the Python injector (identical to the firmware's),
  * Block E: "hand in front of the sensor" episodes with the button held,
    plus one sensor hang with recovery events,
  * Block G: outages reported by a pretend Spec 2 guard (events 3 -> 5),
  * the firmware's estimator columns, computed with the Python estimators.
"""
from __future__ import annotations

import argparse
import json
import shutil
import sys
from pathlib import Path

import numpy as np
import pandas as pd

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
from estimators import FIRMWARE_ESTIMATORS, default_params, make_estimator, run_estimator  # noqa: E402
from injector import GeConfig, inject_stream  # noqa: E402
from logger import session_seed  # noqa: E402

SYNTH_COMMIT = "SYNTHETIC-DEMO"
COLUMNS = ["t_us", "seq", "raw_mm", "clean_mm", "faulted_mm", "ge_state", "mode", "button", "recovery_event",
           "kf_pos", "kf_vel", "gated_pos", "gated_vel", "imm_pos", "imm_vel", "imm_p_bad", "kf_us", "gated_us",
           "imm_us"]


def sensor_sigma(d_mm):
    return 0.8 + 0.005 * np.asarray(d_mm)


def sample_times(rng, seconds):
    dt = 33000 + rng.integers(0, 900, size=int(seconds * 30))
    return 1_000_000 + np.cumsum(dt)


def noisy(rng, true_mm):
    # Student-t with 5 degrees of freedom: heavier tails than a Gaussian.
    e = sensor_sigma(true_mm) * rng.standard_t(5, size=len(true_mm)) / np.sqrt(5 / 3)
    return np.round(true_mm + e).astype(int)


def firmware_dict(cfg: GeConfig, inject: bool, seed: int) -> dict:
    fw = {"fw_commit": SYNTH_COMMIT, "build": "standalone", "inject": str(int(inject)), "mode": cfg.mode,
          "lambda": repr(cfg.lam), "seed": str(seed), "timing_budget_us": "33000", "kf_bytes": "48",
          "gated_bytes": "96", "imm_bytes": "232", "injector_bytes": "88"}
    for k in ("entry_far", "entry_near", "persist_far", "persist_near", "r_ref_mm", "d_src_mm", "sample_period_s",
              "sigma_nom_mm", "kappa", "bias_c_sigmas", "randbias_sigmas"):
        fw[k] = repr(getattr(cfg, k))
    for name in FIRMWARE_ESTIMATORS:
        for k, v in default_params(name).items():
            fw[f"{name}_{k}"] = repr(v)
    return fw


def write_session(out: Path, sid: str, block: str, cols: dict, fw: dict, user: dict, rng, events=None):
    df = pd.DataFrame(cols)
    for name in FIRMWARE_ESTIMATORS:
        est = run_estimator(make_estimator(name), df["t_us"].to_numpy(), df["faulted_mm"].to_numpy())
        df[f"{name}_pos"], df[f"{name}_vel"] = est["pos"], est["vel"]
        if name == "imm":
            df["imm_p_bad"] = est["p_bad"]
    n = len(df)
    # Pretend compute times (us). Synthetic: the real ones come from the ESP32.
    df["kf_us"] = rng.integers(6, 10, n)
    df["gated_us"] = rng.integers(7, 12, n)
    df["imm_us"] = rng.integers(38, 55, n)
    df["seq"] = np.arange(n)
    df = df[COLUMNS]
    fmt = {c: "%.7f" for c in ("kf_pos", "kf_vel", "gated_pos", "gated_vel", "imm_pos", "imm_vel")}
    lines = [",".join(COLUMNS)]
    for row in df.itertuples(index=False):
        vals = []
        for c, v in zip(COLUMNS, row):
            if c == "faulted_mm":
                vals.append(f"{v:.3f}")
            elif c in fmt:
                vals.append("nan" if not np.isfinite(v) else f"{v:.7f}")
            elif c == "imm_p_bad":
                vals.append("nan" if not np.isfinite(v) else f"{v:.6f}")
            else:
                vals.append(str(v))
        lines.append(",".join(vals))
    (out / f"{sid}.csv").write_text("\n".join(lines) + "\n")
    meta = {"session_id": sid, "block": block, "synthetic": True,
            "note": "SYNTHETIC demo session made by analysis/demo/make_demo_data.py - not a measurement",
            "start_utc": "2026-01-01T00:00:00+00:00", "duration_s": float((df.t_us.iloc[-1] - df.t_us.iloc[0]) / 1e6),
            "user": user, "firmware": fw, "events": events or [], "commands_sent": [], "logger_git_commit": SYNTH_COMMIT,
            "counts": {"rows": n, "lost_rows": 0}, "columns": COLUMNS}
    (out / f"{sid}.json").write_text(json.dumps(meta, indent=2) + "\n")


def pendulum(rng, t_s):
    L = rng.uniform(0.48, 0.52)
    d0, A = rng.uniform(280, 320), rng.uniform(50, 90)
    w = np.sqrt(9.81 / L) * (1 - A ** 2 / (16 * (L * 1000) ** 2))  # tiny amplitude effect
    gamma, phi = rng.uniform(0.01, 0.03), rng.uniform(0, 2 * np.pi)
    true = d0 + A * np.exp(-gamma * t_s) * np.sin(w * t_s + phi)
    return true, {"string_length_m": round(L, 3), "rest_distance_mm": round(d0), "amplitude_mm": round(A),
                  "card_size_cm": "10x10", "temp_c": round(rng.uniform(20, 25), 1), "supply_v": 3.3}


def base_cols(t_us, raw, clean, faulted, ge_state, mode, button=None, events=None):
    n = len(t_us)
    return {"t_us": t_us, "seq": np.arange(n), "raw_mm": raw, "clean_mm": clean, "faulted_mm": faulted,
            "ge_state": ge_state, "mode": [mode] * n, "button": np.zeros(n, int) if button is None else button,
            "recovery_event": np.zeros(n, int) if events is None else events}


def block_A(out, rng, distances, seconds):
    for d in distances:
        sid = f"A_demo_{d:03d}mm"
        t = sample_times(rng, seconds)
        x = noisy(rng, np.full(len(t), float(d)))
        cfg = GeConfig()
        write_session(out, sid, "A", base_cols(t, x, x, x.astype(float), np.zeros(len(t), int), "off"),
                      firmware_dict(cfg, False, session_seed(sid)), {"target_distance_mm": d, "temp_c": 22.0}, rng)


def block_motion(out, rng, block, i, seconds, cfg: GeConfig | None, ge_file=None):
    sid = f"{block}_demo_{i:02d}"
    t = sample_times(rng, seconds)
    t_s = (t - t[0]) / 1e6
    true, user = pendulum(rng, t_s)
    x = noisy(rng, true)
    seed = session_seed(sid)
    if cfg is None:
        faulted, bad, mode, inject = x.astype(float), np.zeros(len(t), bool), "off", False
        cfg = GeConfig()
    else:
        faulted, bad = inject_stream(x.astype(float), cfg, seed)
        mode, inject = cfg.mode, True
    fw = firmware_dict(cfg, inject, seed)
    if ge_file:
        user["ge_parameters_file"] = ge_file
    write_session(out, sid, block, base_cols(t, x, x, faulted, bad.astype(int), mode), fw, user, rng)


def block_E(out, rng, i, seconds):
    """Hand in front of the sensor (button held), and one I2C hang with recovery."""
    sid = f"E_demo_{i:02d}"
    t = sample_times(rng, seconds)
    t_s = (t - t[0]) / 1e6
    true, user = pendulum(rng, t_s)
    raw = noisy(rng, true)
    button = np.zeros(len(t), int)
    for start in rng.uniform(5, seconds - 5, size=3):
        dur = rng.uniform(1.0, 2.5)
        hand = (t_s >= start) & (t_s < start + dur)
        raw[hand] = noisy(rng, np.full(hand.sum(), rng.uniform(80, 150)))
        pressed = (t_s >= start + 0.2) & (t_s < start + dur + 0.1)  # human reaction time
        button[pressed] = 1
    clean = raw.copy()
    events = np.zeros(len(t), int)
    # One sensor hang: no readings for ~0.6 s, then a reset brings it back.
    h0 = int(np.searchsorted(t_s, seconds / 2))
    h1 = h0 + 18
    raw[h0:h1] = -1
    clean[h0:h1] = -1
    events[h0], events[h0 + 8], events[h1] = 3, 4, 5
    user["notes"] = "synthetic: hand occlusions with button, one sensor hang"
    write_session(out, sid, "E", base_cols(t, raw, clean, clean.astype(float), np.zeros(len(t), int), "off", button,
                                           events), firmware_dict(GeConfig(), False, session_seed(sid)), user, rng)


def block_G(out, rng, i, seconds, p_gb0, p_bg):
    """Integrated run: a pretend Spec 2 guard reports outages (events 3 -> 5)."""
    sid = f"G_demo_{i:02d}"
    t = sample_times(rng, seconds)
    t_s = (t - t[0]) / 1e6
    true, user = pendulum(rng, t_s)
    raw = noisy(rng, true)
    clean = raw.copy()
    events = np.zeros(len(t), int)
    k = 60
    while k < len(t) - 40:
        if rng.random() < p_gb0:
            n = int(rng.geometric(p_bg))
            n = min(n, len(t) - k - 2)
            clean[k:k + n] = -1
            raw[k:k + n] = -1
            events[k] = 1 if rng.random() < 0.3 else 3  # some latch-up trips, some lock-ups
            events[k + n] = 5
            k += n + 30
        k += 1
    fw = firmware_dict(GeConfig(), False, session_seed(sid))
    fw["build"] = "guard"
    write_session(out, sid, "G", base_cols(t, raw, clean, clean.astype(float), np.zeros(len(t), int), "off",
                                           None, events), fw, user, rng)


def make_ge_parameters_file(path: Path) -> Path:
    """A synthetic ge_parameters.json in the Spec 2 format (marked synthetic)."""
    p = {"format": "sel-sefi-sensor-guard/ge-parameters/v1", "p_gb0": 0.004, "p_bg": 0.09, "lambda": 1e9,
         "spec1_ge_command": "ge 0.004 0.09 1e+09", "sample_period_ms": 33.3,
         "ci95": {"p_gb0": [0.002, 0.007], "p_bg": [0.06, 0.13]},
         "provenance": {"synthetic": True, "firmware_commits": [SYNTH_COMMIT]}}
    path.write_text(json.dumps(p, indent=2) + "\n")
    return path


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", default=str(HERE / "data"))
    ap.add_argument("--n", type=int, default=10, help="sessions per motion block (default 10)")
    ap.add_argument("--seconds", type=float, default=60.0)
    ap.add_argument("--seed", type=int, default=1)
    args = ap.parse_args(argv)

    root = Path(args.out)
    if root.exists():
        shutil.rmtree(root)
    raw = root / "raw"
    raw.mkdir(parents=True)
    rng = np.random.default_rng(args.seed)
    block_A(raw, rng, [100, 175, 250, 325, 400], min(args.seconds, 30))
    for i in range(args.n):
        block_motion(raw, rng, "B", i, args.seconds, None)
        block_motion(raw, rng, "C", i, args.seconds, GeConfig(mode="variance"))
        block_motion(raw, rng, "D", i, args.seconds, GeConfig(mode="frozen"))
        block_E(raw, rng, i, args.seconds)
    from ge_params import load_ge_parameters  # noqa: PLC0415

    ge = load_ge_parameters(make_ge_parameters_file(root / "ge_parameters_SYNTHETIC.json"), allow_synthetic=True)
    measured = GeConfig(entry_far=ge["entry_10hz"], entry_near=ge["entry_10hz"], persist_far=ge["persist_10hz"],
                        persist_near=ge["persist_10hz"], mode="variance")
    for i in range(args.n):
        block_motion(raw, rng, "F", i, args.seconds, measured, ge_file="ge_parameters_SYNTHETIC.json")
        block_G(raw, rng, i, args.seconds, ge["p_gb0"], ge["p_bg"])
    print(f"Wrote {len(list(raw.glob('*.csv')))} SYNTHETIC sessions to {raw}")


if __name__ == "__main__":
    main()
