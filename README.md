**Status: code complete; hardware validation pending.**

# Sim-to-Silicon: does the estimator crossover survive real hardware?

**A hardware test of a simulation result: which state estimator is best
depends on the kind of sensor fault, and does that still hold when the noise
is real?**

In simulation, Vaishvi Agarwal's paper *Robust, Fault-Tolerant State
Estimation for Autonomous Drones During Safety-Critical Operations* (CREST
Gold; _[link/DOI to be added]_) compared a Kalman filter (KF), an
innovation-gated filter, interacting multiple-model (IMM) filters and a
GRU-based learned estimator under burst faults modelled on radiation-induced
single-event effects. Under inflated-variance bursts the IMM was best;
under frozen-output bursts the GRU was best: a **crossover** that depends on
the fault type, not on the estimator. This repository repeats that study on
an ESP32 with a real VL53L0X time-of-flight sensor watching a pendulum,
with real (quantised, non-Gaussian, temperature-dependent) noise, faults
injected into the real readings, physically caused faults, and real
lock-ups and latch-ups from the companion project
[`sel-sefi-sensor-guard`](../sel-sefi-sensor-guard) (Spec 2).

All firmware, analysis, tests and documentation are written. **No hardware
test has been run yet, so this repository contains no results.** The plots
you can make today come from a clearly labelled synthetic demo.

---

## Research questions

* **RQ1.** On real sensor data with injected Gilbert-Elliott (GE) faults, does the IMM still beat the KF and gated filter under inflated-variance bursts?
* **RQ2.** Under frozen-output bursts on real data, does the GRU still beat the classical filters?
* **RQ3.** Does the crossover point move when the nominal noise is real rather than Gaussian? How does the real noise differ from the simulation's assumption?
* **RQ4.** On physically caused faults (occlusion, strong IR light, genuine I2C hangs), which estimator is best, and which simulated fault model do they resemble?
* **RQ5.** On the protected sensor node from Spec 2, with real lock-ups and power cycles, does the ranking match the ranking under injected faults? Does using the GE parameters measured in Spec 2 change the result?
* **RQ6.** What does each classical estimator cost on the ESP32 (microseconds per update, memory)?

Results may confirm or contradict the paper. Both outcomes are reported as they come out.

---

## The rig

_Photo of the rig: to be added once it is built (`docs/hardware_setup.md`)._

A pendulum (string about 0.5 m, white card on the bob) swings towards and
away from a VL53L0X on a heavy base, 25-35 cm away. The ESP32 reads the
sensor about 30 times per second, injects faults into a copy of each
reading, runs the KF, gated KF and IMM, and streams everything to a PC.

| VL53L0X pin | ESP32 pin | Notes |
|---|---|---|
| VIN | 3V3 | |
| GND | GND | |
| SDA | GPIO 21 | I2C data |
| SCL | GPIO 22 | I2C clock |
| XSHUT | GPIO 19 | sensor reset (recovery from hangs) |

| Other | ESP32 pin | Notes |
|---|---|---|
| Tactile button to GND | GPIO 18 (INPUT_PULLUP) | held while a physical fault is present |
| On-board LED | GPIO 2 | lit while a fault is injected |

Step-by-step wiring, expected multimeter readings and serial output:
[`docs/BRING_UP.md`](docs/BRING_UP.md).

---

## Quick start

```bash
python3.11 -m venv .venv
```

```bash
source .venv/bin/activate
```

```bash
pip install -r requirements.txt platformio
```

Check everything on the PC (no hardware needed):

```bash
make test
```

```bash
pio run
```

Run every analysis script on synthetic data (output in `analysis/demo/output/`, stamped "SYNTHETIC DEMO DATA"):

```bash
python analysis/demo/run_demo.py
```

Flash the board and record a session:

```bash
pio run -e esp32dev -t upload
```

```bash
python analysis/logger.py --port /dev/cu.usbserial-0001 --block B
```

### Repository layout

```
firmware/src/main.cpp          the ESP32 program (two builds: esp32dev, esp32dev_guard)
firmware/include/              Arduino-free logic, unit-tested on the PC:
    ge_injector.h  kalman1d.h  gated_kf.h  imm1d.h  sensor_guard.h  serial_commands.h
lib/sel_guard/                 copy of the Spec 2 guard library (Block G), see ORIGIN.md
analysis/                      logger, noise, ground truth, estimators (+GRU), tune, evaluate, figures
analysis/demo/                 synthetic demo data generator and end-to-end runner
tests/                         C++ host tests, pytest, C++/Python reference files
data/                          raw recordings, splits.json, data dictionary (data/README.md)
results/                       generated figures and tables (empty until real data exist)
docs/                          bring-up guide, methods, hardware setup, test card, hardware to-do list
```

---

## Reproducing every figure

After recording the blocks in SPEC.md section 5, run once:

```bash
python analysis/ground_truth.py
```

```bash
python analysis/make_splits.py
```

```bash
python analysis/calibrate_proximity.py
```

```bash
python analysis/tune.py
```

```bash
python analysis/evaluate.py --crossover
```

**How long will it take?** The full protocol (the paper's GRU search for
every condition, plus a 13-point crossover sweep) can take many hours on a
laptop CPU. Measure it first:

```bash
python analysis/tune.py --estimate-time
```

**Laptop preset.** `--quick` uses a smaller GRU search (1 layer × 64 units,
up to 60 epochs) and 5 crossover points instead of 13. It is for a first
look, not the paper's protocol; final results use the full run.

```bash
python analysis/tune.py --quick
```

```bash
python analysis/evaluate.py --crossover --quick
```

**Sensitivity check (deviation from paper, optional).** A wider gated-KF
grid, written only to `results/tables/sensitivity_extended_grid.json`:

```bash
python analysis/tune.py --extended-grid
```

Then each figure is one command:

| Figure | Command |
|---|---|
| 1. Real noise vs Gaussian (RQ3) | `python analysis/noise_characterisation.py` |
| 2. Example session | `python analysis/figures.py --only 2` |
| 3. Median and p95 error per regime (RQ1, RQ2) | `python analysis/figures.py --only 3` |
| 4. NEES, KF vs IMM | `python analysis/figures.py --only 4` |
| 5. Crossover (RQ3) | `python analysis/figures.py --only 5` |
| 6. Physical faults (RQ4) | `python analysis/figures.py --only 6` |
| 7. Assumed vs measured vs real faults (RQ5) | `python analysis/figures.py --only 7` |
| 8. Compute time on the ESP32 (RQ6) | `python analysis/figures.py --only 8` |

`python analysis/figures.py` makes all of them.

---

## The paper's simulation results (for comparison)

These are the published **simulation** values (2D drone at 10 Hz, in metres),
not hardware results. The hardware results are compared with them by
ranking, not magnitude.

| Condition | Paper (burst-regime 95th-percentile error) |
|---|---|
| Inflated variance (κ = 100) | IMM 0.63 m against the genie-aided bound 0.60 m |
| Frozen output | GRU 0.77 m against 1.11 m for the re-tuned Robust KF (the best classical filter) |
| Crossover (λ from inflated variance to bias) | λ ≈ 0.81: classical ahead at 0.75, learned ahead at 0.90 |

The paper's fault chain, its five corruption conditions (inflated variance
and frozen output, plus bias-dominated, random per-burst bias and
systematic proximity bias), its recovery window, its tuning protocol (R
fixed, one fixed q; gate confidence, coasting timeout and κ̂ re-tuned per
condition) and its GRU search are used here, converted from 10 Hz to the
sensor's ≈ 30 Hz so that the behaviour in seconds is the same
([`docs/methods.md`](docs/methods.md), section 4). As in the paper, the
classical filters are reported both with settings carried over from the
inflated-variance condition and re-tuned per condition; the GRU is always
trained afresh on each condition's own training sessions.

---

## Results

> **No results yet.** This section is filled in only from Vaishvi's real
> test-set data, after the hardware experiments. Nothing here is estimated
> or illustrated in advance.

### Real sensor noise (RQ3)

_To be added: Figure 1 and the noise summary (sigma vs distance, kurtosis, autocorrelation, quantisation step)._

### Variance bursts (RQ1)

_To be added: Figure 3 (left), burst-regime p95 per estimator with 95 % intervals, paired bootstrap results._

### Frozen bursts (RQ2)

_To be added: Figure 3 (right), with paired bootstrap results._

### Overconfidence

_To be added: Figure 4 and mean NEES per estimator._

### Crossover (RQ3)

_To be added: Figure 5, the hardware crossover λ with its interval, compared with the simulation's λ ≈ 0.81._

### Secondary corruption conditions

_To be added: bias-dominated, random per-burst bias and systematic proximity bias (replayed on Blocks B-D)._

### Carried-over vs re-tuned settings

_To be added: `results/tables/carried_over_vs_retuned.csv`._

### Physical faults (RQ4)

_To be added: Figure 6 and which fault model they resemble._

### Assumed vs measured parameters, injected vs real faults (RQ5)

_To be added: Figure 7._

### Embedded cost (RQ6)

_To be added: Figure 8 and memory per estimator._

Test conditions: [`docs/test_card.md`](docs/test_card.md) (to be filled in).

---

## Limitations

* **The truth is a fitted model, not an independent instrument.** It is a
  damped sine fitted to the clean readings in 10 s windows; errors smaller
  than the fit residual cannot be resolved.
* **Faults are injected in software on real data, except in Block E**
  (physical faults) and Block G (real lock-ups and latch-ups through the
  Spec 2 guard).
* **One sensor type** (one VL53L0X), one motion (a pendulum), 1D at ≈ 30 Hz,
  where the paper simulated a 2D drone at 10 Hz. Compare rankings, not
  magnitudes.
* Reported accuracies come from replaying the recordings through Python
  estimators that are bit-identical to the firmware's, with tuned settings.

Every deviation from the paper is listed in [`docs/methods.md`](docs/methods.md).

---

## Citation

If you use this code or data, please cite it (see [`CITATION.cff`](CITATION.cff)):

> Agarwal, V. (2026). *estimator-crossover-hardware: Sim-to-Silicon test of
> the estimator crossover on real hardware* (version 0.1.0) [Software].
> _[DOI to be added after archiving on Zenodo]_

and the original simulation paper:

> Agarwal, V. *Robust, Fault-Tolerant State Estimation for Autonomous Drones
> During Safety-Critical Operations.* CREST Gold research paper.
> _[link/DOI to be added]_

## Licence

Code: MIT ([`LICENSE`](LICENSE)). Data, figures and documentation: CC BY 4.0 ([`LICENSE-data`](LICENSE-data)).
