# Methods

This page describes how the hardware study is done, how it mirrors the
simulation paper (*Robust, Fault-Tolerant State Estimation for Autonomous
Drones During Safety-Critical Operations*), and **every place where it
differs** from the paper, with the reason.

Section 6 lists choices that change the scientific meaning and still need
Vaishvi's confirmation before the final analysis (SPEC.md section 9, rule 5).

---

## 1. Pipeline

```
ESP32 (firmware/)                                PC (analysis/)
VL53L0X -> sensor guard -> clean reading -+-> logger.py -> data/raw/<id>.csv + .json
                                          |
              Gilbert-Elliott injector <--+   noise_characterisation.py  (Block A)
                     |                        ground_truth.py            (truth from the clean stream)
                faulted reading               make_splits.py             (train / validation / test)
                     |                        tune.py                    (validation only)
         KF / gated KF / IMM (timed)          evaluate.py                (test only, + crossover)
                     |                        calibrate_proximity.py     (d_src, r_ref from Block B)
                     |                        figures.py
                 CSV row over USB
```

## 2. What is kept the same as the paper

* **Estimators:** a Kalman filter (KF), an innovation-gated ("robust") KF,
  two IMM variants (IMM-oracle with the true mode-switch probabilities,
  IMM-est with probabilities estimated from training data), and a GRU-based
  learned estimator, all on a double-integrator motion model (constant velocity
  driven by white-noise acceleration).
* **Fault model:** the paper's proximity-coupled Gilbert-Elliott chain and its
  parameters (section 4), and all five corruption conditions: inflated
  variance (κ = 100) and frozen output (primary), bias-dominated, random
  per-burst bias and systematic proximity bias (secondary), plus the λ blend
  for the crossover sweep.
* **Tuning protocol:** R fixed (known nominal noise), one fixed shared q;
  gate confidence and coasting timeout (gated KF) and κ̂ (both IMMs) re-tuned
  per condition on validation; results reported both carried over and
  re-tuned.
* **Metrics:** median and 95th-percentile absolute position error as the
  primary metrics, RMSE as secondary, per regime.
* **Regimes:** burst = BAD state; recovery = within N = 21 steps (2.1 s) after
  a burst ends; nominal = everything else. A burst that starts inside a
  recovery window counts as burst.
* **Tuning:** on validation data disjoint from the test data, with a fixed
  budget for every estimator.
* **Statistics:** paired bootstrap comparisons between estimators.
* **Crossover:** sweep λ and re-tune both families at every point.
* **Overconfidence check:** NEES (normalised estimation error squared).

## 3. Deviations from the paper, and why

| # | Paper (simulation) | Here (hardware) | Why |
|---|---|---|---|
| 1 | 2D double integrator | **1D** (one ranging axis) | One VL53L0X measures one distance. A second axis would need a second sensor and a 2D motion with known truth. |
| 2 | 10 Hz updates | **≈ 30 Hz** (33 ms timing budget) | The VL53L0X's standard timing budget. Slower budgets reduce noise but blur the motion. Every per-step quantity of the paper is converted so that the behaviour in **seconds** is the same (section 4). |
| 3 | Drone flyby past a radiation source | **Pendulum** past a virtual source on the sensor axis | A pendulum gives smooth, repeatable motion whose truth can be reconstructed from physics. The bob's closest approach to the virtual source plays the role of the drone's closest approach (section 4.2). |
| 4 | Gaussian measurement noise | **Real noise**: quantised to 1 mm, possibly heavy-tailed, correlated, distance- and temperature-dependent | This is the point of the study (RQ3). Block A measures how it differs. |
| 5 | True state known exactly | **Truth is a fitted model** (damped sine in 10 s windows), not an independent instrument | No second instrument is available. The fit residual is compared with the static noise in every session, and an RTS smoother is used as a model-free check. In nominal periods the truth comes from the same readings the filters see, so nominal errors are partly fit errors. |
| 6 | Faults simulated | Faults **injected in software on real readings** (Blocks C, D, F); **real faults** in Blocks E (occlusion, light, I2C hangs) and G (lock-ups and latch-ups through the Spec 2 guard) | Injection keeps control over the fault statistics while the noise is real; E and G test whether the conclusions survive real faults. |
| 7 | Errors in metres (drone scale) | Errors reported in **mm** | The pendulum moves centimetres. Compare **rankings** and ratios with the paper, not magnitudes. |
| 8 | BAD noise variance κR replaces R | The real reading already carries R, so the injector **adds** (κ − 1)R | The total BAD variance is then κR, as in the paper. |
| 9 | q = 0.01 in the paper's units | q chosen on the Block B validation sessions, then fixed | The paper's q belongs to a drone; the pendulum needs its own value. It is still one shared, fixed value (section 4.5). |
| 10 | Two IMM variants | The firmware runs one IMM (fixed probabilities); IMM-oracle and IMM-est are the same code with different switch probabilities, replayed in Python | The ESP32's compute time for the IMM stands for both. |
| 11 | Secondary conditions simulated | Replayed on the real clean streams of Blocks B-D (they can also be recorded live) | Avoids three more recording blocks; the noise is still real. |

## 4. The paper's parameters on the hardware

### 4.1 Converting 10 Hz values

The paper's chain and filters were specified per step of 0.1 s. The
hardware sample period is T ≈ 1/30 s (the firmware setting
`sample_period_s`, to be set to the measured mean period: `docs/HARDWARE.md`).
To keep the same behaviour in seconds:

| Kind of quantity | Rule | k = T / 0.1 |
|---|---|---|
| probability of staying in a state | p_new = p^k | 1/3 at 30 Hz |
| probability of entering a state | p_new = 1 − (1 − p)^k | 1/3 at 30 Hz |
| a number of steps | n_new = n · 0.1 / T | × 3 at 30 Hz |

These rules make the probability of surviving (or of no event in) any
duration t identical: (p_new)^(t/T) = p^(t/0.1).

All conversions in this study (T = 1/30 s; the firmware recomputes them from
`sample_period_s`):

| Quantity | Paper (10 Hz) | Rule | Hardware (per sample at 30 Hz) | Where |
|---|---|---|---|---|
| Entry P(GOOD→BAD) at ρ = 0 | 0.002 | entry | 0.000667 | `ge_injector.h` `entry_far` |
| Entry P(GOOD→BAD) at ρ = 1 | 0.12 | entry | 0.041716 | `entry_near` |
| Persistence P(BAD→BAD) at ρ = 0 | 0.5 | stay | 0.793701 (exit 0.206299) | `persist_far` |
| Persistence P(BAD→BAD) at ρ = 1 | 0.9 | stay | 0.965489 (exit 0.034511) | `persist_near` |
| IMM-est matrix, P(nominal→bad) (paper's estimate, for comparison and as the firmware default) | 0.01 | entry | 0.003345 | `imm1d.h` `p_nb` |
| IMM-est matrix, P(bad→bad) | 0.67 | stay | 0.875034 (P(bad→nominal) 0.124966) | `imm1d.h` `p_bn` |
| Gated KF coasting timeout, default | 10 steps | steps | 30 samples = **1.0 s** (implemented in seconds) | `gated_kf.h` `coast_s` |
| Gated KF coasting timeout, tuning grid | 3, 5, 10, 20, 50 steps | steps | 9, 15, 30, 60, 150 samples = 0.3, 0.5, 1.0, 2.0, 5.0 s | `tune.py` |
| Recovery window N | 21 steps | steps | 63 samples = **2.1 s** (implemented in seconds) | `evaluate.py` `RECOVERY_S` |

Quantities that are not per step are unchanged: κ = 100, the κ̂ grid
{5, 50, 100, 200}, the gate confidences {0.95, 0.99, 0.999}, c = 4σ,
σ_b = 10σ, ρ (a function of distance, not time). IMM-oracle's probabilities
are the injector's own per-sample values, so they need no conversion.

**Order of operations for the fault chain:** the two probabilities are first
interpolated linearly in ρ **at 10 Hz** (as in the paper), then converted.
So at 30 Hz the per-sample probabilities are not exactly linear in ρ.

**Block F (measured parameters from Spec 2):** Spec 2 reports per-sample
probabilities at its own period (≈ 33.3 ms) without proximity dependence.
`analysis/ge_params.py` converts them to 10 Hz (entry: 1 − (1 − p_gb0)^(0.1/T₂);
persistence: (1 − p_bg)^(0.1/T₂)) and sets the far and near values equal; the
firmware then converts them back to its own period.

**Counted transitions (IMM-est):** counted directly at the hardware rate from
the training sessions' fault sequences (no conversion needed).

### 4.2 Proximity on the pendulum

The paper's proximity factor is ρ(r) = r_ref² / (r² + r_ref²), with r the
drone's distance from the source and r_ref = 8 m chosen by calibration
(widened until a typical pass gave enough BAD samples, with ρ between 0.65
and 0.99 at closest approach). On the pendulum, r = |d − d_src| is the bob's
distance from a **virtual source on the sensor axis** at d_src. With the
source just in front of the bob's closest approach, ρ is high at the near end
of each swing and falls off towards the far end, as during a flyby. d_src
and r_ref are settings (`prox` command), calibrated on Block B recordings
with `analysis/calibrate_proximity.py` so that ρ at closest approach is
about 0.65-0.99 and a typical session has enough BAD samples: a hardware
calibration item in `docs/HARDWARE.md`. Placeholders: d_src = 220 mm,
r_ref = 40 mm.

### 4.3 Corruption conditions

R = σ_nom² is the nominal reading variance from Block A (`sigma_nom_mm`, a
hardware item). The real reading already carries R, so wherever the paper
specifies a total BAD variance of κR, the injector **adds** (κ − 1)R.
b_k = c · ρ(r_k) · (+1) with c = 4 σ_nom is the paper's proximity-scaled bias
(+1 = away from the sensor), using the same ρ as the fault chain.

| Condition | Paper's model | Here | Data |
|---|---|---|---|
| **Inflated variance** (primary) | z = Hx + v, v ~ N(0, κR), κ = 100 | `variance`: clean + N(0, (κ − 1)R) | Block C (recorded) |
| **Frozen output** (primary) | the last good reading, repeated | `frozen` | Block D (recorded) |
| **Bias-dominated** (λ = 1) | z = Hx + b_k + v, v ~ N(0, R) | `bias`: clean + b_k (nothing added: nominal variance) | Blocks B-D replayed |
| **Random per-burst bias** | z = Hx + b_j + v, b_j ~ N(0, σ_b²) drawn once per burst and held, σ_b = 10 σ_nom; v ~ N(0, κR) | `randbias`: clean + b_j + N(0, (κ − 1)R) | Blocks B-D replayed |
| **Systematic proximity bias** | z = Hx + b_k + v, v ~ N(0, κR) | `sysbias`: clean + b_k + N(0, (κ − 1)R) | Blocks B-D replayed |
| **λ blend** (crossover sweep) | λ = 0: inflated variance; λ = 1: bias-dominated | `blend`: clean + N(0, (1 − λ)(κ − 1)R) + λ b_k | Blocks B-D replayed |

The paper's σ_b = 10 m (with σ = 1 m) and c = 4σ are scaled with the
hardware's σ_nom (settings `randbias_sigmas` = 10 and `bias_c_sigmas` = 4).

**The λ blend between the endpoints is assumed linear; endpoints match the
paper; to verify against the original simulation code.** (The paper defines
λ = 0 and λ = 1 but its text does not state the in-between formula.)

**Recorded or replayed.** Blocks C and D record the two primary conditions
live. The three secondary conditions (and the crossover sweep) replay the
clean streams of all Block B-D sessions through the Python injector, which
is bit-identical to the firmware's, with the fault-chain settings of the
Block C/D recordings and a seed per session. They can also be recorded live
(`mode bias`, `mode randbias`, `mode sysbias`).

**Same bursts in every mode.** Every BAD reading draws the same random
numbers whatever the mode (one for the noise, one for a new burst's random
bias), so for a given seed the burst pattern is identical across modes.

### 4.4 Filters

* **KF:** constant-velocity model, q shared and fixed, R fixed (Block A).
* **Gated (robust) KF:** a reading whose normalised innovation squared
  exceeds the χ² gate (1 DOF) is ignored and the filter coasts (predicts).
  The gate confidence is **tuned** from {0.95, 0.99, 0.999} (thresholds
  3.841, 6.635, 10.83); the paper found 0.95 clearly worse. **Coasting
  timeout:** after the timeout without an accepted reading (outages count
  too), the next reading is accepted whatever the gate says, with a normal
  update (as in the paper). The timeout is **tuned** from the paper's
  {3, 5, 10, 20, 50} steps at 10 Hz = {0.3, 0.5, 1.0, 2.0, 5.0} s; the default
  is 1.0 s. Without the timeout the filter can lock itself out
  (`tests/test_host.cpp`, `gated_coasting_timeout_reacquires`).
* **IMM-oracle:** two modes, nominal R and κ̂R. Its switch probabilities
  are the injector's **true** ones at every sample: p_nb = the entry
  probability and p_bn = 1 − persistence at that sample's distance (they
  change with ρ). They are known on hardware because the injector
  generates them; on rows without a reading (the injector does not move)
  both are 0. Only available where faults are injected (not Blocks E, G).
* **IMM-est:** the same IMM, with p_nb, p_bn **counted** from the training
  sessions' BAD sequences of the condition (injector state for C/D/F, the
  replays and the crossover, button and outages for E, guard events for G).
  The paper's estimate was [[0.99, 0.01], [0.33, 0.67]] at 10 Hz.
* Both IMMs: κ̂ **tuned** from {5, 50, 100, 200} (default 100). Under frozen
  output the paper's re-tuning moved κ̂ from 100 to 5. Likelihoods are
  combined in log space.
* **The firmware's live IMM** uses fixed probabilities (the paper's
  estimate converted to 30 Hz). It is only used for timing and to check the
  Python replay; its compute time stands for both variants (same code).
* **Genie-aided bound:** a KF told the true GE state; during BAD readings it
  uses the true variance R + (κ − 1)R (variance mode) or ignores the reading
  (other modes). Reference only.

### 4.5 Tuning protocol

As in the paper:

* **R is not tuned.** It is the known nominal noise, measured in Block A
  (`noise_model.json`, σ at 300 mm), the same for every filter.
* **q is fixed once:** chosen by a 1-D search (26 log-spaced values,
  10⁻² to 10³ m²/s³) with the plain KF on the validation sessions of Block B
  (clean swings), then fixed for every Kalman-based filter in every
  condition (the paper fixed q = 0.01 in its own units). `tune.py --q` fixes
  it by hand instead.
* **Re-tuned per condition** (all eight conditions and every crossover λ),
  by exhaustive grid search on the validation sessions, minimising the mean
  over sessions of the per-session p95 error:

| Filter | Tuned | Grid |
|---|---|---|
| KF | nothing | - |
| Gated KF | gate confidence, coasting timeout | {0.95, 0.99, 0.999} × {0.3, 0.5, 1.0, 2.0, 5.0} s |
| IMM-oracle | κ̂ | {5, 50, 100, 200} |
| IMM-est | κ̂ | {5, 50, 100, 200} |
| GRU | layers, hidden size, learning rate, epoch | section 4.6 |

  Every grid point's validation score is saved in `tuned_params.json`.
* **Carried-over vs re-tuned (both reported, as in the paper), classical
  filters only.** "Carried over" = the settings tuned on the primary
  condition (inflated variance, Block C validation sessions), applied
  unchanged to every other condition. "Re-tuned" = tuned on that condition's
  own validation sessions. `evaluate.py` scores the classical filters both
  ways (`tuning` column in the metrics tables) and writes their
  burst-regime p95 side by side in `carried_over_vs_retuned.csv`. The
  figures show the re-tuned results.
* **The GRU is never carried over.** As in the paper, a fresh GRU is trained
  on each condition's own training sessions and selected on that
  condition's own validation sessions; it appears only in the re-tuned
  results.

### 4.6 GRU

One or more GRU layers and a linear output. Inputs per sample: normalised
reading (0 if none), a "reading present" flag, and the time step. It runs
causally. Trained with the mean squared error against the fitted truth on
training sessions only (plus re-injected copies of each training session for
injected-fault conditions), Adam, up to 250 epochs of 25 batches with
early stopping (patience 30 epochs) on the validation objective.

The paper's final model was 2 layers, hidden size 256, learning rate 1e-3,
up to 250 epochs, patience 30. **Grid searched here by default** (to fit a
laptop CPU): layers {1, 2} × hidden size {64, 128} × learning rate {1e-3}.
`--gru-grid` can add 256 (e.g. `2:256:1e-3`). Every grid point, its best
validation score and the number of epochs it ran are written to
`tuned_params.json` (`grid_searched`).

### 4.7 Run time, the `--quick` preset, and the extended grid

* **Run time.** `python analysis/tune.py --estimate-time` trains each GRU
  grid point for 2 epochs on one condition, times the classical grids, and
  prints an estimate for the whole tuning run and the crossover sweep on
  this computer (a worst case with every grid point running all 250 epochs,
  and a rough guess if early stopping halts near 3 × patience epochs).
* **`--quick` (not the paper's protocol).** For a first look on a laptop,
  `tune.py --quick` and `evaluate.py --quick` use a GRU grid of 1 layer × 64
  units (lr 1e-3), up to 60 epochs with patience 10, and crossover points
  λ = 0, 0.5, 0.75, 0.9, 1.0. The classical grids are unchanged (they are
  cheap). Final results use the full protocol.
* **Extended grid: deviation from paper (sensitivity check only).**
  `python analysis/tune.py --extended-grid` re-tunes the gated KF with gate
  confidence {0.95, 0.99, 0.999, 0.9999} and coasting timeout
  {0.1, 0.2, 0.3, 0.5, 1.0, 2.0, 5.0} s (adds 0.9999 and timeouts below
  0.3 s), on the validation sessions, and scores both the paper-grid choice
  and the extended-grid choice on the test sessions. It writes only
  `results/tables/sensitivity_extended_grid.json`, labelled "deviation from
  paper"; it is never used by default, and `evaluate.py` and the figures
  always use the paper's grids. Its purpose is to show whether a
  paper-grid choice at the edge of the grid (the highest confidence or the
  shortest timeout) is limiting the gated KF.

## 4b. Other implementation details that affect results

* **Missing readings.** Rows with no usable reading (sensor outage, or a
  reading rejected by the sensor guard) carry `-1`. Every estimator then
  only predicts. The injector is not advanced on such rows.
* **Sensor guard before the estimators.** Readings of 65535 (hung sensor),
  stuck readings and out-of-range readings are discarded before the
  injector and the estimators, as a real system would. In Block G the Spec 2
  guard does this instead of `sensor_guard.h` (it owns the sensor, so it can
  power-cycle it).
* **Same code on the ESP32 and the PC.** The Python estimators and injector
  are line-by-line copies of the C++ headers and produce bit-identical
  numbers (`tests/test_estimators.py`, `tests/test_injector.py`; compiled
  with `-ffp-contract=off`). The firmware runs the estimators live with its
  default settings, mainly to measure compute time (RQ6). **Accuracy results
  come from replaying the recorded faulted stream through the Python
  estimators with the tuned settings.** `evaluate.py` re-checks on every
  recording that the Python replay matches the firmware's own output
  (`firmware_check.csv`).
* **Double precision on the ESP32.** The estimators use `double` so that the
  C++ and Python versions agree exactly. The ESP32 has no double-precision
  hardware, so the measured compute times are for software doubles; `float`
  would be faster. This is stated with Figure 8.
* **Time step.** Each update uses the measured time since the previous row
  (`t_us`), not a fixed 33 ms.
* **Warm-up.** The first 2 s of every session are not scored.
* **Summary statistics.** Median and p95 are computed over all test samples
  of a regime; their 95 % intervals come from a cluster bootstrap that
  resamples whole sessions (10,000 resamples). Paired comparisons use the
  per-session p95 differences between two estimators, resampled by session
  (10,000 resamples); `p_two_sided` is twice the share of resampled mean
  differences on the other side of zero.
* **Crossover sweep.** The clean streams of all Block B-D sessions are
  re-injected with the Python injector (identical to the firmware's) in
  blend mode at each λ (default grid 0, 0.1, ..., 0.7, 0.75, 0.8, 0.85, 0.9,
  1.0), with the fault-chain settings of the Block C/D recordings. At each λ
  the four filters are re-tuned on validation sessions with the grids of
  section 4.5 (R and q stay fixed), IMM-est's transitions are re-counted, and
  the GRU is retrained on training sessions; the best classical filter is **chosen on validation** and both
  are scored on test sessions by the burst-regime p95. The crossover is the
  first λ where the GRU's error drops below the best classical filter's
  (linear interpolation); its interval comes from resampling test sessions.
  Figure 5 marks the paper's crossover, λ ≈ 0.81 (classical ahead at 0.75,
  learned ahead at 0.90).
* **Physical faults (Block E).** BAD = the button is held, or the sensor
  guard reports an outage. Readings near a button press are excluded from
  the truth fit; truth there is bridged by the damped-sine fit.
* **Real outages (Block G).** BAD = from a guard `SENSOR_FAULT` or
  `SEL_TRIP` event until the next `SENSOR_RECOVERED`, labelled automatically
  from the `recovery_event` column.

## 5. Experiment blocks

See SPEC.md section 5. For each block the logger sends the matching
firmware settings and a per-session seed derived from the session ID, so
every injected session can be regenerated exactly.

## 6. Choices to confirm (they change the scientific meaning)

The paper's published parameters, corruption conditions and tuning
protocol are now used throughout (section 4). What remains:

| Choice | Current setting | Status | Where |
|---|---|---|---|
| How λ interpolates between the two ends | added variance (1 − λ)(κ − 1)R, offset λ b_k | **assumed linear; endpoints match the paper; to verify against the original simulation code** | `ge_injector.h` BLEND, `injector.py` |
| Crossover metric | burst-regime p95, best classical chosen on validation | design choice | `evaluate.py` `crossover_sweep` |
| Warm-up excluded from scoring | 2 s | design choice | `evaluate.py` `WARMUP_S` |
| Truth-model adequacy threshold | residual ≤ 2 × static noise | design choice | `ground_truth.py --max-ratio` |
| Test-card failure criterion | set with `--fail-threshold-mm` | your choice | `evaluate.py` |

Settled by the paper: fault-chain values and proximity law, κ = 100, the
five corruption conditions (with c = 4σ and σ_b = 10σ), recovery window
N = 21 steps (2.1 s), R fixed from the nominal noise, one fixed shared q,
gate confidence and coasting timeout tuned (grids above, force-accept with
a normal update at the timeout), κ̂ tuned from {5, 50, 100, 200} for
IMM-oracle (true probabilities) and IMM-est (counted), the GRU search, the
crossover parameter λ and the paper's crossover (λ ≈ 0.81), carried-over
(classical filters, from the inflated-variance condition) and re-tuned
results both reported, a fresh GRU per condition, and the reference values (frozen: re-tuned
Robust KF 1.11 m vs GRU 0.77 m; variance: IMM 0.63 m vs genie bound 0.60 m).

## 7. Limitations

* **The truth is a fitted model, not an independent instrument.** Errors
  smaller than the fit residual cannot be resolved, and in nominal periods
  truth and readings share the same noise.
* **Faults are injected in software on real data, except in Blocks E and G.**
  The injected faults have the statistics we chose, not necessarily those of
  real radiation effects.
* **One sensor type, one unit** (one VL53L0X), one motion (a pendulum), one
  room. Results may not carry over to other sensors, motions or temperatures.
* **1D only**, at ≈ 30 Hz: the drone setting of the paper is 2D at 10 Hz.
* The pendulum is not exactly a damped sine (large-amplitude period change,
  twisting of the card, elliptical swing); the residual check flags this.
* Block E faults are caused by hand, so their timing and strength vary, and
  the button marks them with a human reaction delay.
* The live firmware estimates use default settings; reported accuracies use
  the tuned settings replayed in Python (bit-identical code).
