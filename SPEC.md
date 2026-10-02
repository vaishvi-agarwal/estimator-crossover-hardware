# SPEC 1 — Sim-to-Silicon: Does the Estimator Crossover Survive Real Hardware?

Repository name: `estimator-crossover-hardware`
Rank: 1 of 2 (flagship). Built on the same hardware as Spec 2, which supplies its real fault data.
Owner: Vaishvi Agarwal
Platform: ESP32 DevKit V1 (Arduino framework via PlatformIO) + VL53L0X time-of-flight ranging sensor + Python 3.11

---

## 0. How to use this spec

1. Create an empty folder named `estimator-crossover-hardware`, run `git init`, and copy this file into it as `SPEC.md`.
2. Start Claude Code in that folder and say: **"Read SPEC.md. Implement Milestone 1 only, then stop and tell me exactly what to test on the hardware."**
3. Test on the real hardware, report back what happened, then ask for the next milestone. Do not let Claude Code run ahead of the hardware.
4. Commit after every milestone with a clear message. A steady commit history is part of the proof that the work happened over time.

---

## 1. Context (Claude Code: read this before writing anything)

Vaishvi's CREST Gold research paper, *Robust, Fault-Tolerant State Estimation for Autonomous Drones During Safety-Critical Operations*, compared five estimators in **simulation**: a Kalman filter (KF), an innovation-gated robust filter, two interacting multiple-model (IMM) variants, and a GRU-based learned estimator. Sensor faults were modelled as **proximity-coupled Gilbert–Elliott (GE) bursts**, motivated by radiation-induced single-event functional interrupts and latch-up.

Key simulation findings to test on hardware:
- Under **inflated-variance** burst faults, the IMM was strongest (burst-regime 95th-percentile error 0.63 m against a genie-aided bound of 0.60 m).
- The standard KF was overconfident by more than an order of magnitude during bursts.
- Under **frozen-output** faults (precise but outdated readings), the ordering reversed and the GRU was best (0.77 m against 1.11 m).
- There is a **crossover** between the two fault types: the best estimator depends on the corruption type, not on the estimator.

Her paper used a 2D double integrator, a 10 Hz update rate, median and 95th-percentile error as primary metrics (RMSE secondary), nominal/burst/recovery regime labelling, tuning on disjoint validation seeds, and paired bootstrap comparisons. **Reuse these methods so the hardware results are directly comparable.**

Her personal theme is *characterising how and when systems stop being predictable*. Her Physics IA found that a model is valid only inside an interval. This project asks the same question of her own paper: **do the simulation conclusions hold when the noise is real (quantised, non-Gaussian, temperature-dependent) rather than idealised?**

---

## 2. Research questions

- **RQ1.** On real sensor data with injected GE faults, does the IMM still beat the KF and gated filter under inflated-variance bursts?
- **RQ2.** Under frozen-output bursts on real data, does the GRU still beat the classical filters?
- **RQ3.** Does the crossover point move when the nominal noise is real rather than Gaussian? Characterise the real sensor noise (distribution, autocorrelation, quantisation) and report how it differs from the simulation's assumption.
- **RQ4 (physical faults).** On *physically caused* faults (occlusion, strong IR light, genuine I2C sensor hangs), which estimator performs best, and which simulated fault model do these real faults resemble?
- **RQ5 (closing the loop).** When the estimators run on the protected sensor node from Spec 2, with **real** lock-ups and power cycles instead of injected ones, does the estimator ranking match the ranking under injected faults? And does injecting faults with the Gilbert–Elliott parameters *measured* in Spec 2 (instead of the paper's assumed parameters) change the result?
- **RQ6 (embedded cost).** What does each classical estimator cost to run on the ESP32 (microseconds per update, memory)?

Results may confirm or contradict the paper. **Both outcomes are valid. Never adjust analysis to force agreement.**

---

## 3. Physical setup

### 3.1 Motion with known ground truth: a pendulum
A pendulum gives smooth, physically predictable motion, so ground truth can be reconstructed from physics.

- String length about 0.5 m (period ≈ 1.4 s), hung from a rigid support (doorframe hook, retort stand, or shelf).
- Bob: a small weight with a stiff **white card about 10 × 10 cm** glued flat, facing the sensor.
- VL53L0X fixed to a heavy base, pointing horizontally at the card **along the swing direction**, with a rest distance of about 25–35 cm and a swing amplitude of about 5–10 cm.
- Measure and record string length, rest distance, card size, and room temperature for every session.

### 3.2 Wiring (ESP32 DevKit V1)

| VL53L0X pin | ESP32 pin | Notes |
|---|---|---|
| VIN | 3V3 | |
| GND | GND | |
| SDA | GPIO21 | I2C data |
| SCL | GPIO22 | I2C clock |
| XSHUT | GPIO19 | sensor reset (used for recovery from hangs) |

| Other | ESP32 pin | Notes |
|---|---|---|
| Tactile button to GND | GPIO18 (INPUT_PULLUP) | Vaishvi presses it to mark physical-fault periods |
| On-board LED | GPIO2 | lit while a fault is being injected |

### 3.3 Sensor configuration
Use the Pololu `VL53L0X` Arduino library. Continuous mode, timing budget 33 ms (≈30 Hz). Detect timeouts and the 65535 / out-of-range reading. On a hang, attempt recovery: XSHUT low for 10 ms, then re-initialise. Log every recovery event.

---

## 4. What to build

### 4.1 Firmware (`firmware/`, PlatformIO, Arduino framework)
1. **Sensor driver wrapper** with timeout detection, hang detection (N identical readings or 65535), and XSHUT recovery.
2. **Gilbert–Elliott fault injector** applied to a *copy* of each clean reading:
   - Two states, GOOD and BAD, with transition probabilities `p_gb` and `p_bg`.
   - **Proximity coupling**: `p_gb` rises as measured distance falls (the bob is "closest to the source"), using `p_gb(d) = p_gb0 * exp(-(d - d_min)/lambda)` clipped to [0, 1]. Parameters are settable over serial.
   - BAD-state corruption modes: `variance` (add zero-mean Gaussian noise with inflated σ), `frozen` (repeat the last GOOD reading), and `mix(alpha)` (a blend for sweeping the crossover).
   - Seeded RNG so every session is repeatable.
3. **Real-time estimators** for the 1D double integrator (state = [position, velocity]), each fed the *faulted* stream:
   - KF
   - innovation-gated KF (chi-square gate, 1 DOF)
   - 2-mode IMM (nominal R and inflated R), with Markov transition matrix settable over serial
4. **Logging** over serial (921600 baud) as CSV, one row per sample:
   `t_us, seq, raw_mm, clean_mm, faulted_mm, ge_state, mode, button, recovery_event, kf_pos, kf_vel, gated_pos, gated_vel, imm_pos, imm_vel, imm_p_bad, kf_us, gated_us, imm_us`
   where `*_us` is the measured compute time of each estimator update.
5. **Serial commands**: `mode <variance|frozen|mix> [alpha]`, `ge <p_gb0> <p_bg> <lambda>`, `seed <n>`, `inject <on|off>`, `params` (print all settings as a `#` metadata line), `help`.
6. Estimator and injector logic must live in **header-only C++ with no Arduino dependencies**, so it can be unit-tested on a PC.

### 4.2 Python (`analysis/`)
1. `logger.py`: records sessions to `data/raw/<session_id>.csv` with a JSON sidecar of metadata (all settings, string length, distances, temperature, notes typed by Vaishvi).
2. `ground_truth.py`: reconstructs truth from the **clean** stream:
   - Primary: fit a damped sinusoid `d(t) = d0 + A·exp(-γt)·sin(ωt + φ)` in sliding windows (about 10 s) with `scipy.optimize.curve_fit`.
   - Check: a Rauch–Tung–Striebel smoother on the clean stream.
   - Report the fit residual RMS and compare it with the sensor's measured static noise. If the residual is much larger than the static noise, warn that the truth model is inadequate for that session.
3. `estimators.py`: Python versions of KF, gated KF, and IMM that are **numerically identical** to the firmware, plus a **GRU estimator** (PyTorch) trained only on training sessions.
4. `noise_characterisation.py`: static-target recordings at several distances → noise σ vs distance, histogram with Gaussian overlay, Q–Q plot, autocorrelation, quantisation step.
5. `evaluate.py`:
   - Per-sample regime labels (nominal / burst / recovery) exactly as in the paper: burst = BAD state, recovery = within N steps after a burst ends, nominal = otherwise.
   - Metrics: median and 95th-percentile absolute position error per regime (primary), RMSE (secondary), and normalised estimation error squared (NEES) to measure over- or under-confidence.
   - Paired bootstrap (10,000 resamples) on per-session differences between estimators.
   - **Crossover sweep**: replay recorded clean sessions through the Python injector at `mix(alpha)` for alpha from 0 to 1, re-tuning both families at each alpha (as the paper did), and locate the crossover.
6. `tune.py`: tunes every estimator's parameters on the **validation sessions only**, with a fixed search budget, minimising mean per-session 95th-percentile error.
7. `figures.py`: produces all figures in Section 6.

### 4.3 Data split (prevents leakage)
Sessions are split by session, never by sample: **train 60 % / validation 20 % / test 20 %**, fixed in `data/splits.json`. The GRU sees only training sessions. Report all final numbers on test sessions only.

### 4.4 Tests (`tests/`)
- C++ host tests (g++): KF converges on a constant; gate rejects a large outlier; IMM mode probability rises during an injected burst; GE injector's empirical state frequencies match theory within tolerance; frozen mode repeats the last GOOD value.
- Python tests (pytest): Python and C++ estimators give identical outputs on a fixed input file (generate the reference file from the C++ test binary); regime labelling matches hand-worked examples; bootstrap function returns sensible intervals on synthetic data.
- GitHub Actions workflow running both test suites on every push.

---

## 5. Experiment protocol (Vaishvi runs these)

| Block | Sessions | What |
|---|---|---|
| A. Static noise | 5 distances × 60 s | Card fixed, no motion. Characterises real noise. |
| B. Clean motion | ≥ 20 × 60 s | Pendulum swinging, injection off. Truth model validation. |
| C. Variance bursts | ≥ 15 × 60 s | Injection on, `mode variance`. |
| D. Frozen bursts | ≥ 15 × 60 s | Injection on, `mode frozen`. |
| E. Physical faults | ≥ 10 × 60 s | No injection. Hand occlusion, strong lamp or sunlight, tapping the I2C wires. Press the button while the fault is present. |
| F. Measured-parameter bursts | ≥ 10 × 60 s | Injection on, using `results/ge_parameters.json` copied from the Spec 2 repository. |
| G. Integrated live run | ≥ 10 × 60 s | Sensor powered through the Spec 2 guard circuit. No software injection. Trigger real lock-ups and simulated latch-ups during the swing; the guard's recovery events become the bursts. |

Vary the release amplitude slightly between sessions. Record room temperature each session. Blocks B–D also feed the offline crossover sweep.

---

## 6. Required figures

1. Real noise vs Gaussian: histogram, Q–Q plot, autocorrelation (RQ3).
2. Example session: truth, faulted measurements, and all estimators, with burst periods shaded.
3. Median and p95 error per regime, per estimator, for variance and frozen modes (RQ1, RQ2), with bootstrap intervals.
4. NEES over time for KF vs IMM (overconfidence check).
5. Crossover plot: p95 error vs alpha for best classical vs GRU, **with the simulation paper's crossover marked for comparison** (RQ3).
6. Physical-fault results (RQ4).
7. Assumed vs measured GE parameters, and injected vs real (Block G) faults: estimator ranking side by side (RQ5).
8. Compute time per update on the ESP32, per estimator (RQ6).

---

## 7. Repository structure (final)

```
estimator-crossover-hardware/
├── README.md
├── SPEC.md
├── LICENSE                      # MIT (code)
├── LICENSE-data                 # CC BY 4.0 (data, figures, docs)
├── CITATION.cff
├── .zenodo.json
├── .gitignore
├── requirements.txt
├── platformio.ini
├── firmware/
│   ├── src/main.cpp
│   └── include/  ge_injector.h  kalman1d.h  gated_kf.h  imm1d.h  sensor_guard.h
├── analysis/   logger.py  ground_truth.py  estimators.py  noise_characterisation.py
│               tune.py  evaluate.py  figures.py
├── tests/      test_host.cpp  test_estimators.py  test_regimes.py  reference_io/
├── data/       raw/  processed/  splits.json  README.md   # data dictionary
├── results/    figures/  tables/                         # generated only
├── docs/
│   ├── hardware_setup.md        # photos, wiring diagram, pendulum dimensions
│   ├── test_card.md             # MRS-style test conditions (see Section 8)
│   └── methods.md               # mirrors the paper's protocol, notes every deviation
└── .github/workflows/tests.yml
```

---

## 8. Definition of done

- [ ] All tests pass locally and in GitHub Actions.
- [ ] README contains: one-paragraph summary; link to the original paper; research questions; photo of the rig (placeholder until Vaishvi adds it); wiring table; quick start; how to reproduce every figure with one command each; results section; limitations; citation.
- [ ] **Results section starts empty with clear placeholders.** It is filled only from Vaishvi's real test-set data. Claude Code must never invent, estimate, or "illustrate" results.
- [ ] `docs/test_card.md` reports test conditions using the seven fields of Vaishvi's Minimum Reporting Standard, adapted from radiation testing to fault injection: (1) fault type and source (injected GE / physical); (2) fault rate (`p_gb0`, `p_bg`, lambda); (3) total exposure (number of bursts, total BAD time, units stated); (4) operating state (sensor mode, timing budget, supply voltage); (5) ambient temperature; (6) failure criterion (p95 error above a stated threshold); (7) device identity (part numbers, board versions, library versions, firmware commit hash).
- [ ] `docs/methods.md` lists every deviation from the simulation paper (1D instead of 2D, ≈30 Hz instead of 10 Hz, pendulum instead of flyby) and why.
- [ ] Limitations stated plainly, including: truth is a fitted model, not an independent instrument; faults are injected in software on real data except in Block E; one sensor type tested.
- [ ] `CITATION.cff` and `.zenodo.json` filled in (author: Vaishvi Agarwal; licence; keywords; related identifier linking the paper if it has a DOI).

---

## 9. Rules for Claude Code

1. Work **one milestone at a time**. After each, stop and give Vaishvi a short, numbered hardware test checklist with the expected result.
2. Vaishvi is new to hardware. Explain wiring in plain words, warn before anything that could damage a component, and comment code so a beginner can follow it.
3. Keep all algorithm code free of Arduino dependencies and unit-tested on a PC.
4. Never fabricate data, results, photos, or citations. Simulated data may be used **only** for tests, clearly labelled as synthetic, and never in `results/`.
5. If a design choice changes the scientific meaning (truth model, regime definitions, tuning protocol), ask Vaishvi before deciding.
6. Prefer simple, correct code over clever code.

## 10. Milestones

1. PlatformIO project; read the VL53L0X at ≈30 Hz; CSV logging; `logger.py`. **Hardware check:** readings change smoothly as a hand moves.
2. Sensor guard: timeout, hang detection, XSHUT recovery; button input. **Hardware check:** unplug and replug SDA and confirm recovery is logged.
3. Static noise block and `noise_characterisation.py`. **Hardware check:** run Block A.
4. Pendulum, `ground_truth.py`, Block B. **Check:** fit residual close to static noise.
5. GE injector and host tests. **Check:** injected burst statistics match settings.
6. KF, gated KF, IMM in firmware and Python, with the C++/Python equivalence test and compute timing.
7. Blocks C–E data collection support; `splits.json`.
8. GRU, tuning on validation sessions, evaluation, bootstrap, crossover sweep, figures.
9. Integration with Spec 2: read `ge_parameters.json` (Block F); log the guard's recovery events in the same CSV (Block G); label real outages as bursts automatically from those events.
10. README, test card, methods, citation files, CI. Vaishvi fills in the results.
