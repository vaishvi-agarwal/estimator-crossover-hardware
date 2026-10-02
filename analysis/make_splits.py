"""Assign sessions to train / validation / test and save data/splits.json (SPEC.md 4.3).

    python analysis/make_splits.py

Rules that prevent "leakage" (information from the test set sneaking into
tuning or training):
  * Whole sessions are assigned, never single samples.
  * About 60 % train, 20 % validation, 20 % test, separately inside each
    block (B-G), so every fault type appears in every split.
  * The assignment is random but fixed by --seed, and it is APPEND-ONLY:
    once a session is in a split it never moves, even when you record more
    sessions later and run this again. New sessions are assigned so that each
    block's proportions stay as close to 60/20/20 as possible.
  * Block A (static noise) is not split: it is only used to describe the noise.

The GRU trains on "train" only; tune.py uses "validation" only; every number
reported in the paper comes from "test" only.
"""
from __future__ import annotations

import argparse
import json
import random
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from common import DEFAULT_DATA, Paths, list_sessions, write_json  # noqa: E402

FRACTIONS = {"train": 0.6, "validation": 0.2, "test": 0.2}
SPLIT_BLOCKS = "BCDEFG"


def assign(existing: dict[str, str], new_sessions: list[str], rng: random.Random) -> dict[str, str]:
    """Add new_sessions to `existing` ({session: split}), keeping each split near its target share.

    Each new session goes to the split that is furthest below its target
    count; ties are broken at random. Existing assignments never change.
    """
    out = dict(existing)
    pending = list(new_sessions)
    rng.shuffle(pending)
    for s in pending:
        n = len(out) + 1
        counts = {k: sum(1 for v in out.values() if v == k) for k in FRACTIONS}
        deficit = {k: FRACTIONS[k] * n - counts[k] for k in FRACTIONS}
        best = max(deficit.values())
        choices = sorted(k for k, d in deficit.items() if abs(d - best) < 1e-9)
        out[s] = rng.choice(choices)
    return out


def make_splits(paths: Paths, seed: int = 20260101) -> dict:
    old = json.loads(paths.splits.read_text()) if paths.splits.exists() else {}
    old_map = {s: k for k in FRACTIONS for s in old.get(k, [])}
    rng = random.Random(seed)
    result_map: dict[str, str] = {}
    per_block = {}
    for block in SPLIT_BLOCKS:
        sessions = list_sessions(paths, block)
        existing = {s: old_map[s] for s in sessions if s in old_map}
        new = sorted(s for s in sessions if s not in old_map)
        block_map = assign(existing, new, random.Random(f"{seed}-{block}-{len(existing)}"))
        result_map.update(block_map)
        per_block[block] = {k: sum(1 for v in block_map.values() if v == k) for k in FRACTIONS}
    # Sessions that were in the old file but whose files are gone are kept, so
    # nothing silently changes split.
    for s, k in old_map.items():
        result_map.setdefault(s, k)
    out = {k: sorted(s for s, v in result_map.items() if v == k) for k in FRACTIONS}
    out["seed"] = seed
    out["per_block_counts"] = per_block
    out["rule"] = "append-only; whole sessions; ~60/20/20 inside each block B-G; Block A not split"
    write_json(paths.splits, out)
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--data", default=str(DEFAULT_DATA))
    ap.add_argument("--seed", type=int, default=20260101)
    args = ap.parse_args(argv)
    out = make_splits(Paths(args.data), args.seed)
    for b, c in out["per_block_counts"].items():
        if sum(c.values()):
            print(f"Block {b}: {c}")
    print(f"Wrote {Paths(args.data).splits}")


if __name__ == "__main__":
    main()
