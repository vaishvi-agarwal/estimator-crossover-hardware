"""ge_params.py: reading Spec 2's measured GE parameters (Block F)."""
import json

import pytest

from ge_params import load_ge_parameters, rescale, to_10hz
from injector import GeConfig, GeInjector


def write(tmp_path, **over):
    d = {"format": "sel-sefi-sensor-guard/ge-parameters/v1", "p_gb0": 0.0001, "p_bg": 0.09, "lambda": 1e9,
         "sample_period_ms": 33.3, "ci95": {"p_gb0": [0.00007, 0.00016], "p_bg": [0.06, 0.13]},
         "provenance": {"synthetic": False}}
    d.update(over)
    p = tmp_path / "ge.json"
    p.write_text(json.dumps(d))
    return p


def test_reads_measured_file(tmp_path):
    p = load_ge_parameters(write(tmp_path))
    words = p["command"].split()
    assert words[0] == "ge" and len(words) == 5 and words[1] == words[2] and words[3] == words[4]
    assert "most_bursts" in p["ci_commands"]


def test_round_trip_through_the_firmware_conversion(tmp_path):
    # Spec 2 per-sample values -> 10 Hz -> the injector's per-sample values at the same period
    p = load_ge_parameters(write(tmp_path))
    cfg = GeConfig(entry_far=p["entry_10hz"], entry_near=p["entry_10hz"], persist_far=p["persist_10hz"],
                   persist_near=p["persist_10hz"], sample_period_s=0.0333)
    inj = GeInjector(cfg)
    assert abs(inj.p_entry(300) - 0.0001) < 1e-12
    assert abs((1 - inj.p_persist(300)) - 0.09) < 1e-12


def test_refuses_synthetic(tmp_path):
    with pytest.raises(SystemExit):
        load_ge_parameters(write(tmp_path, provenance={"synthetic": True}))
    assert load_ge_parameters(write(tmp_path, provenance={"synthetic": True}), allow_synthetic=True)["synthetic"]


def test_refuses_wrong_format(tmp_path):
    with pytest.raises(SystemExit):
        load_ge_parameters(write(tmp_path, format="something else"))


def test_rescale_and_to_10hz():
    assert abs(rescale(0.1, 33.3, 66.6) - 0.19) < 1e-12        # two steps: 1 - 0.9^2
    e, s = to_10hz(0.1, 0.2, 50.0)                             # two 50 ms steps per 0.1 s
    assert abs(e - 0.19) < 1e-12 and abs(s - 0.64) < 1e-12
