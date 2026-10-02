"""logger.py: line parsing and session saving (no serial port needed)."""
import json

from common import Paths
from logger import build_commands, classify_line, parse_meta, row_ok, save_session, session_seed, summarise


def test_classify_line():
    assert classify_line("# fw_commit=abc") == "meta"
    assert classify_line("t_us,seq,raw_mm") == "header"
    assert classify_line("123,4,250") == "row"
    assert classify_line("ets Jun  8 2016 00:22:57") == ""
    assert classify_line("") == ""


def test_parse_meta():
    assert parse_meta("# params mode=frozen seed=3") == ("params", {"mode": "frozen", "seed": "3"})
    assert parse_meta("# fw_commit=abc") == ("", {"fw_commit": "abc"})


def test_row_ok_and_lost_rows():
    assert row_ok("1,2,3", 3) and not row_ok("1,2", 3)
    s = summarise(["t_us", "seq", "raw_mm"], ["0,0,1", "1,1,1", "2,4,1"])
    assert s == {"rows": 3, "lost_rows": 2}


def test_save_session_refuses_overwrite(tmp_path):
    paths = Paths(data=tmp_path)
    metas = [{"host_t": 0, "line": "# fw_commit=abc"}, {"host_t": 1, "line": "# event type=3 reason=timeout"}]
    csv, js = save_session(paths, "B_test", ["t_us", "seq", "raw_mm"], ["0,0,250"], metas,
                           {"session_id": "B_test", "block": "B", "synthetic": False})
    info = json.loads(js.read_text())
    assert info["firmware"]["fw_commit"] == "abc"
    assert info["events"][0]["type"] == "3"
    assert csv.read_text().splitlines() == ["t_us,seq,raw_mm", "0,0,250"]
    try:
        save_session(paths, "B_test", ["t_us"], [], [], {})
    except SystemExit:
        pass
    else:
        raise AssertionError("overwrote raw data")


def test_block_presets():
    c = build_commands("D", "D_x", ["sigma 40"], None)
    assert c[0] == "mode frozen" and "sigma 40" in c and c[-1] == "inject on"
    assert c[-2] == f"seed {session_seed('D_x')}"
    assert build_commands("B", "B_x", [], None)[-1] == "inject off"
    f = build_commands("F", "F_x", [], "ge 0.0001 0.08 1e+09")
    assert "ge 0.0001 0.08 1e+09" in f and f[-1] == "inject on"
    assert session_seed("a") != session_seed("b") and session_seed("a") == session_seed("a")
