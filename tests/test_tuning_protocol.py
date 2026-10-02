"""tune.py follows the paper's tuning protocol (grids and fixed values)."""
from tune import KAPPA_HAT_GRID, grid


def test_gated_grid_is_the_papers():
    g = grid("gated")
    assert len(g) == 15  # 3 confidences x 5 timeouts
    thresholds = sorted({round(x["gate"], 3) for x in g})
    assert thresholds == [3.841, 6.635, 10.828]  # chi-square, 1 DOF, at 95 / 99 / 99.9 %
    assert sorted({x["coast_s"] for x in g}) == [0.3, 0.5, 1.0, 2.0, 5.0]  # 3, 5, 10, 20, 50 steps at 10 Hz


def test_imm_grid_and_nothing_else_tuned():
    assert KAPPA_HAT_GRID == (5.0, 50.0, 100.0, 200.0)
    for name in ("imm_oracle", "imm_est"):
        assert [set(x) for x in grid(name)] == [{"r_bad_factor"}] * 4  # only kappa_hat
    assert grid("kf") == [{}]  # R and q fixed: nothing to tune


def test_extended_grid_is_a_superset_beyond_the_paper():
    from tune import extended_gated_grid

    ext = extended_gated_grid()
    paper = {(round(x["gate"], 6), x["coast_s"]) for x in grid("gated")}
    assert paper <= {(round(x["gate"], 6), x["coast_s"]) for x in ext}
    assert max(x["gate_confidence"] for x in ext) == 0.9999
    assert min(x["coast_s"] for x in ext) < 0.3


def test_quick_preset_and_overrides():
    import argparse

    from evaluate import add_gru_args, gru_kwargs_from_args

    ap = add_gru_args(argparse.ArgumentParser())
    full = gru_kwargs_from_args(ap.parse_args([]))
    quick = gru_kwargs_from_args(ap.parse_args(["--quick"]))
    assert len(full["grid"]) == 4 and full["max_epochs"] == 250 and full["patience"] == 30
    assert len(quick["grid"]) == 1 and quick["max_epochs"] == 60
    assert gru_kwargs_from_args(ap.parse_args(["--quick", "--gru-max-epochs", "5"]))["max_epochs"] == 5
