"""Read the Gilbert–Elliott parameters MEASURED in Spec 2 (Block F).

    python analysis/ge_params.py results/ge_parameters.json

The file comes from the sibling repository sel-sefi-sensor-guard
(`python analysis/fit_gilbert_elliott.py` there, after its soak test). Copy
it into this repository's results/ folder. This script checks it and prints
the serial command for the firmware, plus the commands for the ends of its
95 % confidence intervals (to check whether the estimator ranking depends on
them; see the guard's docs/spec1_integration.md, point 5).

Spec 2's probabilities are PER SAMPLE at its sample period (about 33.3 ms),
with no proximity dependence. This firmware's `ge` command takes the paper's
form: four probabilities per 0.1 s (entry and persistence, far and near). So
they are converted to 10 Hz (the firmware converts them back to its own
sample period) and the far and near values are set equal:
    entry_10Hz   = 1 - (1 - p_gb0)^(0.1 / T)
    persist_10Hz = (1 - p_bg)^(0.1 / T)
NOTE: the file's own `spec1_ge_command` is in the OLD 3-number format and is
rejected by this firmware; use the command printed here.

logger.py --ge-params uses the same checks, so Block F cannot start with a
wrong or synthetic file.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

FORMAT = "sel-sefi-sensor-guard/ge-parameters/v1"


def rescale(p: float, from_ms: float, to_ms: float) -> float:
    """A per-sample probability at one sample period, converted to another:
    p2 = 1 - (1 - p)^(T2 / T1)   (the chance of at least one event in the longer/shorter step)."""
    return 1.0 - (1.0 - p) ** (to_ms / from_ms)


def to_10hz(p_gb0: float, p_bg: float, period_ms: float) -> tuple[float, float]:
    """Spec 2's per-sample (GOOD->BAD, BAD->GOOD) at period_ms -> (entry, persistence) per 0.1 s."""
    k = 100.0 / period_ms
    return 1.0 - (1.0 - p_gb0) ** k, (1.0 - p_bg) ** k


def ge_command(entry10: float, persist10: float) -> str:
    """Firmware command with no proximity dependence (far = near)."""
    return f"ge {entry10:.12g} {entry10:.12g} {persist10:.12g} {persist10:.12g}"


def load_ge_parameters(path: Path | str, allow_synthetic: bool = False) -> dict:
    """Check the file and return the per-sample values, their 10 Hz equivalents,
    the firmware command and the commands for the ends of the confidence intervals."""
    path = Path(path)
    if not path.exists():
        raise SystemExit(f"{path} not found. Copy results/ge_parameters.json from sel-sefi-sensor-guard.")
    d = json.loads(path.read_text())
    if d.get("format") != FORMAT:
        raise SystemExit(f"{path}: format is {d.get('format')!r}, expected {FORMAT!r}")
    if d.get("provenance", {}).get("synthetic", False) and not allow_synthetic:
        raise SystemExit(f"{path} is marked synthetic (demo data from Spec 2). Block F needs the measured file.")
    for k in ("p_gb0", "p_bg", "lambda"):
        if not isinstance(d.get(k), (int, float)):
            raise SystemExit(f"{path}: {k} missing")
    if not (0 <= d["p_gb0"] <= 1 and 0 < d["p_bg"] <= 1):
        raise SystemExit(f"{path}: probabilities out of range")

    p_gb0, p_bg = float(d["p_gb0"]), float(d["p_bg"])
    if float(d["lambda"]) < 1e6:
        print(f"WARNING: {path} has lambda = {d['lambda']} (a proximity dependence); it is ignored here.")
    ci = d.get("ci95") or {}
    period = float(d.get("sample_period_ms", 33.3))
    entry10, persist10 = to_10hz(p_gb0, p_bg, period)
    out = {"p_gb0": p_gb0, "p_bg": p_bg, "sample_period_ms": period, "entry_10hz": entry10,
           "persist_10hz": persist10, "command": ge_command(entry10, persist10),
           "ci95": ci, "warnings": d.get("warnings", []), "source": str(path),
           "synthetic": bool(d.get("provenance", {}).get("synthetic", False))}
    if "p_gb0" in ci and "p_bg" in ci:
        out["ci_commands"] = {
            "fewest_bursts": ge_command(*to_10hz(ci["p_gb0"][0], ci["p_bg"][1], period)),
            "most_bursts": ge_command(*to_10hz(ci["p_gb0"][1], ci["p_bg"][0], period)),
        }
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("path", nargs="?", default="results/ge_parameters.json")
    ap.add_argument("--allow-synthetic", action="store_true", help="only for the demo")
    args = ap.parse_args(argv)
    p = load_ge_parameters(args.path, args.allow_synthetic)
    print(f"Spec 2, per {p['sample_period_ms']} ms sample: p_gb0 = {p['p_gb0']:.6g}, p_bg = {p['p_bg']:.6g}")
    print(f"As 10 Hz values: entry = {p['entry_10hz']:.6g}, persistence = {p['persist_10hz']:.6g}")
    print(f"Firmware command (Block F):  {p['command']}")
    for name, cmd in p.get("ci_commands", {}).items():
        print(f"  CI end '{name}':  {cmd}")
    for w in p["warnings"]:
        print(f"WARNING from Spec 2: {w}")
    if p["synthetic"]:
        print("NOTE: this file is SYNTHETIC (demo only).")


if __name__ == "__main__":
    main()
