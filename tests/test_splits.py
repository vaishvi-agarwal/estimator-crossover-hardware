"""make_splits.py: whole sessions, ~60/20/20 per block, append-only."""
import json
import random

from common import Paths
from make_splits import assign, make_splits


def fake_sessions(paths, block, n, start=0):
    paths.raw.mkdir(parents=True, exist_ok=True)
    for i in range(start, start + n):
        sid = f"{block}_{i:03d}"
        (paths.raw / f"{sid}.csv").write_text("t_us\n0\n")
        (paths.raw / f"{sid}.json").write_text(json.dumps({"session_id": sid, "block": block}))


def test_proportions():
    m = assign({}, [f"s{i}" for i in range(20)], random.Random(0))
    counts = {k: list(m.values()).count(k) for k in ("train", "validation", "test")}
    assert counts == {"train": 12, "validation": 4, "test": 4}


def test_append_only_and_per_block(tmp_path):
    paths = Paths(data=tmp_path)
    fake_sessions(paths, "C", 15)
    fake_sessions(paths, "D", 15)
    fake_sessions(paths, "A", 5)
    first = make_splits(paths)
    all_first = {s: k for k in ("train", "validation", "test") for s in first[k]}
    assert not any(s.startswith("A_") for s in all_first)          # Block A is not split
    assert first["per_block_counts"]["C"] == {"train": 9, "validation": 3, "test": 3}
    fake_sessions(paths, "C", 5, start=15)                           # record more later
    second = make_splits(paths)
    all_second = {s: k for k in ("train", "validation", "test") for s in second[k]}
    assert all(all_second[s] == k for s, k in all_first.items())   # nothing moved
    assert second["per_block_counts"]["C"] == {"train": 12, "validation": 4, "test": 4}
    # no session in two splits
    sets = [set(second[k]) for k in ("train", "validation", "test")]
    assert not (sets[0] & sets[1] or sets[0] & sets[2] or sets[1] & sets[2])
