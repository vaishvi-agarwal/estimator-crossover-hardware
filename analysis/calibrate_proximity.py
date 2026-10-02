"""Calibrate the proximity geometry (d_src, r_ref) on Block B recordings.

    python analysis/calibrate_proximity.py --d-src 220 --r-ref 30,40,50,60

The paper chose r_ref = 8 m by calibration: it widened r_ref until a typical
pass gave enough BAD samples, with the proximity factor
    rho = r_ref^2 / (r^2 + r_ref^2)
between 0.65 and 0.99 at closest approach. Here r = |d - d_src| is the bob's
distance from a virtual source on the sensor axis at d_src (mm).

For every Block B session and every r_ref you list, this prints:
  * rho at the bob's closest approach (the 1st percentile of clean_mm),
  * the BAD fraction and bursts per minute that the paper's fault chain would
    give on that session (the Python injector replayed on its clean stream,
    averaged over --seeds seeds).

Pick d_src and r_ref so that rho at closest approach is about 0.65-0.99 for
your typical sessions and the BAD fraction gives "enough" BAD samples
(enough bursts per session for the burst-regime statistics), then set them
on the board with `prox <d_src_mm> <r_ref_mm>` (or change the defaults in
firmware/include/ge_injector.h) and tick the item in docs/HARDWARE.md.

Writes <results>/tables/proximity_calibration.csv.
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
import pandas as pd

sys.path.insert(0, str(Path(__file__).resolve().parent))
from common import Paths, add_path_args, check_no_synthetic_in_results, list_sessions, load_session  # noqa: E402
from injector import GeConfig, GeInjector, inject_stream  # noqa: E402


def run(paths: Paths, d_src: float, r_refs: list[float], seeds: int, period: float) -> pd.DataFrame:
    sessions = list_sessions(paths, "B")
    if not sessions:
        raise SystemExit(f"No Block B sessions in {paths.raw}.")
    rows, metas = [], []
    for sid in sessions:
        df, meta = load_session(paths, sid)
        metas.append(meta)
        clean = df["clean_mm"].to_numpy(dtype=float)
        ok = clean[clean >= 0]
        closest = float(np.percentile(ok, 1))
        minutes = (df["t_us"].iloc[-1] - df["t_us"].iloc[0]) / 60e6
        for r_ref in r_refs:
            cfg = GeConfig(d_src_mm=d_src, r_ref_mm=r_ref, sample_period_s=period)
            fr, bursts = [], []
            for k in range(seeds):
                _, bad = inject_stream(clean, cfg, 1000 + k)
                fr.append(bad.mean())
                bursts.append(int(np.sum(np.diff(np.r_[0, bad.astype(int)]) == 1)))
            rows.append({"session_id": sid, "closest_mm": closest, "d_src_mm": d_src, "r_ref_mm": r_ref,
                         "rho_at_closest": GeInjector(cfg).rho(closest), "bad_fraction": float(np.mean(fr)),
                         "bursts_per_min": float(np.mean(bursts) / minutes)})
    check_no_synthetic_in_results(paths, metas)
    table = pd.DataFrame(rows)
    paths.make_output_dirs()
    table.to_csv(paths.tables / "proximity_calibration.csv", index=False)
    summary = table.groupby("r_ref_mm").agg(rho_min=("rho_at_closest", "min"), rho_median=("rho_at_closest", "median"),
                                            rho_max=("rho_at_closest", "max"), bad_fraction=("bad_fraction", "mean"),
                                            bursts_per_min=("bursts_per_min", "mean"))
    print(f"d_src = {d_src} mm, sample period = {period:.4f} s, {len(sessions)} Block B sessions")
    print(summary.to_string(float_format=lambda v: f"{v:.3f}"))
    print("Target (paper): rho at closest approach about 0.65-0.99, and enough BAD samples per session.")
    return table


def main(argv=None):
    ap = add_path_args(argparse.ArgumentParser(description=__doc__,
                                               formatter_class=argparse.RawDescriptionHelpFormatter))
    ap.add_argument("--d-src", type=float, default=GeConfig().d_src_mm, help="virtual source position (mm)")
    ap.add_argument("--r-ref", default="20,30,40,50,60,80", help="r_ref values to try (mm)")
    ap.add_argument("--seeds", type=int, default=5)
    ap.add_argument("--period", type=float, default=1 / 30, help="sample period (s)")
    args = ap.parse_args(argv)
    run(Paths(args.data, args.results), args.d_src, [float(x) for x in args.r_ref.split(",")], args.seeds,
        args.period)


if __name__ == "__main__":
    main()
