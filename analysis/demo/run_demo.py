"""Run EVERY analysis script end to end on synthetic demo data.

    python analysis/demo/run_demo.py            # about 10-20 minutes
    python analysis/demo/run_demo.py --small    # a few minutes (used in CI)

Data goes to analysis/demo/data/, tables and figures to analysis/demo/output/.
Both folders are ignored by git. Nothing is written to data/ or results/.

The point is only to show that the code runs and the figures draw. The data
are made up (see make_demo_data.py), so the numbers mean nothing: every
figure is stamped "SYNTHETIC DEMO DATA".
"""
from __future__ import annotations

import argparse
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
ANALYSIS = HERE.parent
DATA = HERE / "data"
OUT = HERE / "output"


def step(title, script, *args):
    print(f"\n=== {title} ===", flush=True)
    t0 = time.time()
    subprocess.run([sys.executable, str(script), *map(str, args)], check=True)
    print(f"({time.time() - t0:.0f} s)")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--small", action="store_true", help="fewer sessions, a tiny GRU search (for CI)")
    ap.add_argument("--n", type=int, help="sessions per motion block")
    args = ap.parse_args(argv)
    n = args.n or (5 if args.small else 10)
    seconds = 40 if args.small else 60
    # GRU: a tiny grid and few epochs (the real default is 1-2 layers x 64-128 units, up to 250 epochs)
    gru = ["--gru-grid", "1:16:3e-3" if args.small else "1:32:3e-3,2:32:3e-3",
           "--gru-max-epochs", 3 if args.small else 12, "--gru-patience", 3, "--gru-steps", 10 if args.small else 20]
    n_boot = 300 if args.small else 2000
    lambdas = "0,0.5,1" if args.small else "0,0.25,0.5,0.75,0.9,1"
    paths = ["--data", DATA, "--results", OUT]

    step("1. Synthetic sessions", HERE / "make_demo_data.py", "--out", DATA, "--n", n, "--seconds", seconds)
    step("1b. Block F: read the (synthetic) Spec 2 GE parameters", ANALYSIS / "ge_params.py",
         DATA / "ge_parameters_SYNTHETIC.json", "--allow-synthetic")
    step("1c. Proximity calibration (Block B)", ANALYSIS / "calibrate_proximity.py", *paths, "--seeds", 2)
    step("2. Noise characterisation (Block A)", ANALYSIS / "noise_characterisation.py", *paths)
    step("3. Ground truth (Blocks B-G)", ANALYSIS / "ground_truth.py", *paths)
    step("4. Train / validation / test split", ANALYSIS / "make_splits.py", "--data", DATA)
    step("5. Tuning (validation) and GRU training (train)", ANALYSIS / "tune.py", *paths, *gru,
         "--extra-seeds", 1)
    step("6. Evaluation (test) and crossover sweep", ANALYSIS / "evaluate.py", *paths, "--n-boot", n_boot,
         "--crossover", "--lambdas", lambdas, *gru, "--fail-threshold-mm", 20)
    step("7. Figures", ANALYSIS / "figures.py", *paths)
    print(f"\nDone. SYNTHETIC demo figures: {OUT / 'figures'}")


if __name__ == "__main__":
    main()
