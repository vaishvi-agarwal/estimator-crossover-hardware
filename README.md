# Sim-to-Silicon: does the estimator crossover survive real hardware?

In my simulation paper, *Robust, Fault-Tolerant State Estimation for Autonomous Drones During Safety-Critical Operations*, I compared a Kalman filter (KF), an innovation-gated filter, two interacting multiple-model (IMM) filters and a GRU-based learned estimator under burst sensor faults modelled on radiation-induced single-event effects. The IMM was best when faulty readings were noisy. The GRU was best when they were frozen. Which estimator won depended on the type of fault, not on the estimator itself.

That result came from simulated noise, which is Gaussian, well behaved and exactly known. Real sensors are not like that. So I tested the same comparison on hardware: an ESP32 reading a VL53L0X time-of-flight sensor that watches a swinging pendulum. The faults were injected into real readings, caused physically, and produced by real lock-ups and latch-ups through the protection circuit in my companion project, [`sel-sefi-sensor-guard`](https://github.com/vaishvi-agarwal/sel-sefi-sensor-guard).

---

## Research questions

* **RQ1.** On real sensor data with injected Gilbert–Elliott (GE) faults, does the IMM still beat the KF and gated filter under inflated-variance bursts?
* **RQ2.** Under frozen-output bursts, does the GRU still beat the classical filters?
* **RQ3.** Does the crossover point move when the noise is real rather than Gaussian, and how does the real noise differ from my simulation's assumption?
* **RQ4.** On physically caused faults (occlusion, strong IR light, genuine I2C hangs), which estimator is best, and which simulated fault model do these faults resemble?
* **RQ5.** On the protected sensor node, with real lock-ups and power cycles, does the ranking match the ranking under injected faults? Does using measured fault parameters instead of assumed ones change the result?
* **RQ6.** What does each classical estimator cost to run on the ESP32?

---

## How I set it up

A pendulum (string about 0.5 m, white card on the bob) swung towards and away from the VL53L0X, which sat on a heavy base 25–35 cm away. I chose a pendulum because its motion is predictable from physics, which gave me a ground truth to measure errors against. The ESP32 read the sensor about 30 times per second, injected faults into a copy of each reading, ran the KF, gated KF and IMM in real time, and streamed everything to my laptop. I trained the GRU and replayed every recording offline in Python, using estimators that give bit-identical results to the firmware.

| VL53L0X pin | ESP32 pin | Notes |
|---|---|---|
| VIN | 3V3 | |
| GND | GND | |
| SDA | GPIO 21 | I2C data |
| SCL | GPIO 22 | I2C clock |
| XSHUT | GPIO 19 | sensor reset (recovery from hangs) |

| Other | ESP32 pin | Notes |
|---|---|---|
| Tactile button to GND | GPIO 18 | held while a physical fault was present |
| On-board LED | GPIO 2 | lit while a fault was injected |

Step-by-step wiring and bring-up are in [`docs/BRING_UP.md`](docs/BRING_UP.md).

---

## Method

I kept my simulation paper's method so the two studies could be compared directly:

* **Fault model:** a proximity-coupled Gilbert–Elliott chain. Faults became more frequent and lasted longer as the bob approached a virtual source on the sensor axis, as they did when the simulated drone approached the radiation source.
* **Five corruption conditions:** inflated variance, frozen output, bias-dominated, random per-burst bias and systematic proximity-scaled bias, plus a continuous sweep between inflated variance and bias to locate the crossover.
* **Regimes:** every sample was labelled nominal, burst or recovery, using the paper's 2.1 s recovery window.
* **Tuning:** measurement noise fixed from static recordings and one shared process-noise value; gate confidence, coasting timeout and IMM inflation re-tuned per condition on validation sessions only. Classical filters were reported both carried over and re-tuned; the GRU was trained afresh on each condition.
* **Statistics:** median and 95th-percentile error per regime, with paired bootstrap comparisons, on test sessions that were never used for tuning or training.

The paper ran at 10 Hz and the sensor runs at about 30 Hz, so I converted every timing so that the behaviour in seconds stayed the same. Every conversion and every deviation from the paper is in [`docs/methods.md`](docs/methods.md).

For reference, the paper's simulation results (2D drone, 10 Hz, burst-regime 95th-percentile error) were:

| Condition | Paper result |
|---|---|
| Inflated variance (κ = 100) | IMM 0.63 m vs genie-aided bound 0.60 m |
| Frozen output | GRU 0.77 m vs re-tuned Robust KF 1.11 m |
| Crossover | λ ≈ 0.81, between inflated variance and systematic bias |

Because the setups differ in scale, I compare rankings rather than magnitudes.

---

## Limitations

* **My ground truth was a fitted model, not an independent instrument.** I fitted a damped sine to the clean readings in 10 s windows, so errors smaller than the fit residual could not be resolved.
* **Most faults were injected in software on real data.** Only the physical-fault sessions and the guard-circuit sessions used faults that happened on their own.
* **The setup was narrower than the paper's.** I used one sensor type, one motion and one dimension at about 30 Hz, where the paper simulated a 2D drone at 10 Hz.

---

## Repository layout

```
firmware/src/main.cpp     the ESP32 program (two builds: esp32dev, esp32dev_guard)
firmware/include/         estimator, fault-injector and sensor-guard logic, unit-tested on a PC
lib/sel_guard/            guard library from sel-sefi-sensor-guard (see ORIGIN.md)
analysis/                 logging, noise analysis, ground truth, estimators, tuning, evaluation, figures
tests/                    C++ and Python tests, including firmware/Python equivalence checks
data/                     recordings, session splits and data dictionary
results/                  figures and tables
docs/                     bring-up guide, methods, hardware setup, test card
```

---

## Reproducing the results

```bash
python3.11 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt platformio
make test          # all tests, no hardware needed
pio run            # build the firmware
```

With the recordings in `data/raw/`:

```bash
python analysis/ground_truth.py
python analysis/make_splits.py
python analysis/calibrate_proximity.py
python analysis/tune.py
python analysis/evaluate.py --crossover
python analysis/figures.py
```

The full protocol can take many hours on a laptop. `python analysis/tune.py --estimate-time` estimates it, and `--quick` gives a faster first look.

---

## Citation

> Agarwal, V. (2026). *estimator-crossover-hardware: Sim-to-Silicon test of the estimator crossover on real hardware* (version 0.1.0) [Software].

Original simulation paper:

> Agarwal, V. *Robust, Fault-Tolerant State Estimation for Autonomous Drones During Safety-Critical Operations.*

Code: MIT ([`LICENSE`](LICENSE)). Data, figures and documentation: CC BY 4.0 ([`LICENSE-data`](LICENSE-data)).
