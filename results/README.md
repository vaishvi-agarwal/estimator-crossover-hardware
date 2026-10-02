# results/

Everything in this folder is **generated** by the scripts in `analysis/`
from the real recordings in `data/raw/`. Do not edit files here by hand;
re-run the scripts instead (README, "Reproducing every figure").

* `figures/`: Figures 1-8 (PNG and PDF)
* `tables/`: noise summary, truth quality, tuned parameters, metrics,
  paired bootstrap, crossover, compute cost, test-card values, and a
  provenance file recording the sessions and commits behind them

Empty until the hardware experiments are done. Synthetic demo output goes
to `analysis/demo/output/` instead; the scripts refuse to write anything
made from synthetic data into this folder.

Block F also reads `ge_parameters.json` from here, copied from the Spec 2
repository (`sel-sefi-sensor-guard/results/ge_parameters.json`).
