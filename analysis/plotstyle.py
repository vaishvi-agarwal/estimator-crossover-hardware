"""One look for every figure: colours, names and saving.

Each estimator keeps the same colour in every figure. The colours are the
Okabe-Ito palette, which stays distinguishable for colour-blind readers.
"""
from __future__ import annotations

from pathlib import Path

import matplotlib

matplotlib.use("Agg")  # draw to files, no window needed
import matplotlib.pyplot as plt  # noqa: E402

COLORS = {
    "kf": "#0072B2",      # blue
    "gated": "#E69F00",   # orange
    "imm": "#009E73",     # green
    "imm_oracle": "#009E73",  # green
    "imm_est": "#56B4E9", # sky blue
    "gru": "#CC79A7",     # pink
    "genie": "#555555",   # grey (reference only)
    "truth": "#000000",
    "meas": "#999999",
    "burst": "#D55E00",   # shading for BAD periods
}
NAMES = {
    "kf": "KF",
    "gated": "Gated KF",
    "imm": "IMM",
    "imm_oracle": "IMM-oracle",
    "imm_est": "IMM-est",
    "gru": "GRU",
    "genie": "Genie KF (bound)",
}
REGIME_NAMES = {"nominal": "Nominal", "burst": "Burst", "recovery": "Recovery"}

plt.rcParams.update({
    "figure.dpi": 110,
    "savefig.dpi": 200,
    "font.size": 9,
    "axes.spines.top": False,
    "axes.spines.right": False,
    "axes.grid": True,
    "grid.alpha": 0.3,
    "legend.frameon": False,
})


def synthetic_banner(fig, synthetic: bool):
    """Stamp 'SYNTHETIC DEMO DATA' across a figure made from demo data."""
    if synthetic:
        fig.text(0.5, 0.5, "SYNTHETIC DEMO DATA\nnot a result", ha="center", va="center", fontsize=28,
                 color="red", alpha=0.18, rotation=20, transform=fig.transFigure, zorder=100)


def save(fig, folder: Path, name: str, synthetic: bool = False) -> list[Path]:
    """Save as PNG (for the README) and PDF (for papers)."""
    synthetic_banner(fig, synthetic)
    folder.mkdir(parents=True, exist_ok=True)
    out = []
    for ext in ("png", "pdf"):
        p = folder / f"{name}.{ext}"
        fig.savefig(p, bbox_inches="tight")
        out.append(p)
    plt.close(fig)
    return out
