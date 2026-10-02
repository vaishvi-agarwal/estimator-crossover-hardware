# Bring-up: testing the hardware step by step

This is the plan for the day the parts arrive. It follows the milestones in
SPEC.md section 10. **Do one milestone at a time** and only move on when its
check passes. Each step says what to wire, what the multimeter should read,
what the serial monitor should print, and what to do if it does not.

All the code is already written and compiles. Nothing here has been tried on
real hardware yet, so if reality disagrees with this page, believe reality,
write down what you saw, and fix the page.

> **The one rule:** unplug the USB cable → change the wiring → check it with
> the multimeter → plug the USB cable back in. Never move a wire while the
> board is powered.

---

## 0. Before you start

### What you need

| Part | Notes |
|---|---|
| ESP32 DevKit V1 (30-pin) | the board with the blue LED on GPIO 2 |
| VL53L0X breakout board | any breakout with VIN, GND, SDA, SCL, XSHUT pins |
| Tactile push button | 2 or 4 legs |
| Breadboard and jumper wires | short wires (under 15 cm) for SDA and SCL |
| USB cable | must be a **data** cable, not charge-only |
| Multimeter | DC volts (20 V range) and continuity (beep) mode |
| Pendulum parts (Milestone 4) | string, small weight, stiff white card about 10 × 10 cm, rigid support, heavy base for the sensor |
| Ruler or tape measure, thermometer | for the session metadata |

### Software, once

```bash
python3.11 -m venv .venv
```

```bash
source .venv/bin/activate
```

```bash
pip install -r requirements.txt platformio
```

Find the board's serial port: plug it in and run `pio device list`. On a
Mac it looks like `/dev/cu.usbserial-0001`, on Windows `COM3`.

### Safety, read once

* **Power the VL53L0X from 3V3, not from VIN/5V.** Many breakouts accept
  5 V, but then their I2C pins may pull up to 5 V, and the ESP32's pins are
  **not** 5 V tolerant: 5 V on GPIO 21/22 can damage the ESP32.
* **The button goes between GPIO 18 and GND, never to 3V3.** The firmware
  turns on the ESP32's internal pull-up resistor; a button to 3V3 does
  nothing useful, and a wrong pin could short 3V3 to GND when pressed.
* **3V3 must never touch GND.** Before the first power-up of every new
  wiring, use continuity mode: 3V3 to GND must **not** beep.
* The VL53L0X has an infrared laser (Class 1, eye-safe in normal use). Do
  not look into it from close up through a magnifier.
* Make the pendulum support solid: a falling weight can break the sensor.
  Keep the bob's swing well clear of the sensor (it should never touch it).
* Block E (strong lamp): halogen lamps get hot. Keep the lamp away from
  the breadboard and wires, and switch it off between sessions.

---

## Milestone 1: read the VL53L0X at about 30 Hz

**Goal:** distances arrive over USB about 30 times per second and change
smoothly when a hand moves in front of the sensor.

### Wiring (USB unplugged)

| VL53L0X pin | ESP32 pin | What it is |
|---|---|---|
| VIN | 3V3 | power (3.3 V) |
| GND | GND | ground |
| SDA | GPIO 21 | I2C data |
| SCL | GPIO 22 | I2C clock |
| XSHUT | GPIO 19 | sensor reset (used from Milestone 2 on) |

Wire XSHUT now too, so you do not need to touch the wiring again later.

### Continuity check (USB still unplugged)

| Probes on | Expected |
|---|---|
| ESP32 3V3 and GND | **no beep** (a beep = short circuit: find it before powering!) |
| ESP32 GPIO 21 and VL53L0X SDA | beep |
| ESP32 GPIO 22 and VL53L0X SCL | beep |
| ESP32 GPIO 21 and GPIO 22 | no beep |

### Flash the firmware

```bash
pio run -e esp32dev -t upload
```

If the upload stalls at `Connecting....`, hold the board's **BOOT** button
until the upload starts.

### Multimeter (USB plugged in)

Black probe on GND.

| Red probe on | Expected |
|---|---|
| ESP32 3V3 | 3.2–3.4 V |
| VL53L0X VIN | the same as 3V3 |
| SDA (GPIO 21) | about 2.8–3.3 V (I2C lines rest HIGH) |
| SCL (GPIO 22) | about 2.8–3.3 V |
| XSHUT (GPIO 19) | about 2.8 V on most breakouts (their own pull-up); anything from 2.6 to 3.3 V is fine |

### Serial output

```bash
pio device monitor
```

Press the board's **EN** (reset) button. You should see something like
(numbers will differ):

```
# fw_commit=1ca1a96e5f build=standalone
# params fw_commit=1ca1a96e5f build=standalone inject=0 mode=variance lambda=0.5 seed=1 timing_budget_us=33000
# params entry_far=0.002 entry_near=0.12 persist_far=0.5 persist_near=0.90000000000000002 r_ref_mm=40 ...
# params sigma_nom_mm=2 kappa=100 bias_c_sigmas=4 randbias_sigmas=10
# params kf_q=1 kf_r=3.9999999999999998e-06 kf_p0_vel=1 ... gated_gate=6.6348966010212144 gated_coast_s=1
# params imm_q=1 imm_r=3.9999999999999998e-06 ... imm_r_bad_factor=100 ...
# sizeof kf_bytes=48 gated_bytes=72 imm_bytes=192 injector_bytes=136
# reset
t_us,seq,raw_mm,clean_mm,faulted_mm,ge_state,mode,button,recovery_event,kf_pos,...
1534211,0,312,312,312.000,0,off,0,0,0.3120000,0.0000000,0.3120000,0.0000000,0.3120000,0.0000000,0.100000,4,5,31
1567543,1,311,311,311.000,0,off,0,0,0.3115123,-0.0146000,...
```

The full firmware is already on the board, so there are more columns than
this milestone needs. For now look only at `raw_mm` (third column).

### Check (SPEC Milestone 1)

1. Move a hand slowly towards the sensor and away: `raw_mm` changes
   smoothly, from about 50 mm up to 1000 mm or more.
2. `t_us` grows by about 33 000 each row (33 ms): roughly 30 rows per second.
3. Record 60 s with the logger and check that no rows were lost:

```bash
python analysis/logger.py --port /dev/cu.usbserial-0001 --block B --duration 60 --no-prompt --session-id test_m1 --data /tmp/m1
```

   It should end with `Saved ... (about 1800 rows, 0 lost)`. Delete `/tmp/m1`
   afterwards: this was only a test.

### If it fails

| What you see | Likely cause |
|---|---|
| nothing at all in the monitor | wrong port, or the monitor is not at 921600 (platformio.ini sets it; use `pio device monitor`) |
| garbage characters | wrong baud rate |
| `# warning sensor_init_failed` | SDA and SCL swapped; VIN not connected; XSHUT held low (check its voltage) |
| `raw_mm` always 8190 or 8191 | nothing in front of the sensor within range (that is "out of range"); point it at a wall |
| `raw_mm` always 65535 | the sensor is hung: unplug USB for 10 s and try again |
| rows lost | a bad USB cable or hub; try a short cable directly into the computer |

---

## Milestone 2: sensor guard and button

**Goal:** when the sensor stops answering, the firmware notices, resets it
through XSHUT, and logs the recovery.

### Wiring (USB unplugged)

Add the button:

| Button leg | ESP32 pin |
|---|---|
| one leg | GPIO 18 |
| the opposite leg | GND |

(With a 4-leg button, use two legs that are diagonally opposite; those are
always on different sides of the switch.)

### Multimeter (USB plugged in)

| Red probe on | Button up | Button pressed |
|---|---|---|
| GPIO 18 | about 3.3 V | 0 V |

### Serial output

In `pio device monitor`, the `button` column (8th) is `0`, and `1` while you
hold the button.

### Check (SPEC Milestone 2): unplug and replug SDA

> This is safe: SDA only carries tiny signal currents. Pull the **SDA wire
> at the VL53L0X end**, wait 2 s, push it back. Do not pull VIN or GND while
> powered.

Expected lines:

```
# event t_us=81234567 type=3 reason=timeout ok=0 outage_us=0      <- fault detected
# event t_us=81245678 type=4 reason=none ok=0 outage_us=0         <- reset tried while SDA is out: fails
...  (more type=4 lines every 0.5 s while SDA is unplugged)
# event t_us=83301234 type=4 reason=none ok=1 outage_us=0         <- SDA back: reset works
# event t_us=83340000 type=5 reason=none ok=0 outage_us=2210000   <- first good reading: recovered
```

`reason` may also be `bus` (I2C errors) instead of `timeout`; both are fine.
During the outage the rows show `-1` in `raw_mm` and `clean_mm`, and the
`recovery_event` column shows `3` once at the start and `5` once at the end.

Repeat 10 times. Every attempt must end with a `type=5` line (see
`docs/HARDWARE.md`, `BOOT_MS`).

### If it fails

| What you see | Likely cause |
|---|---|
| `button` never changes | button legs on the same side of the switch; try the diagonal pair |
| no `type=3` event | the sensor kept answering: pull SDA at the sensor end, not at the ESP32 |
| `type=4 ok=0` forever after SDA is back | XSHUT not connected to GPIO 19, so the reset does nothing: unplug USB for 10 s |
| the board resets (boot text appears) | a loose GND or VIN wire: check every wire is fully pushed in |

---

## Milestone 3: static noise (Block A)

**Goal:** measure how noisy the sensor really is.

No wiring change. Fix the sensor to its heavy base. Tape the white card to a
book or box so that it **cannot move**. Measure the distance from the front
of the sensor to the card with a ruler.

Record **5 distances × 60 s**, for example 150, 200, 250, 300, 350 mm:

```bash
python analysis/logger.py --port /dev/cu.usbserial-0001 --block A --target-distance-mm 250
```

The logger asks for the temperature and the other conditions. Then:

```bash
python analysis/noise_characterisation.py
```

**Check:** it prints one row per distance, with `sigma_mm` around 1–5 mm and
`longest_equal_run` well below 30 (see `docs/HARDWARE.md`). Figure 1 appears
in `results/figures/`. Then set `r` (`docs/HARDWARE.md`).

If `sigma_mm` is much larger than 5 mm, check that the card is matte (not
glossy), square to the sensor, and larger than the sensor's field of view.

---

## Milestone 4: pendulum and ground truth (Block B)

**Goal:** a swinging pendulum whose motion is well described by a damped sine.

### Build

* String about 0.5 m long (period about 1.4 s), tied to a rigid support.
* Bob: small weight with the white card glued flat, facing the sensor.
* Sensor on its heavy base, pointing **horizontally** at the card **along
  the swing direction**, 25–35 cm from the card at rest.
* Pull the bob back 5–10 cm towards or away from the sensor and let go. It
  should swing straight towards and away from the sensor, not in a circle.

Measure and write down: string length, rest distance, card size,
temperature (the logger asks for them).

### Record Block B (at least 20 × 60 s, injection off)

```bash
python analysis/logger.py --port /dev/cu.usbserial-0001 --block B
```

Vary the release amplitude a little between sessions.

```bash
python analysis/ground_truth.py
```

**Check (SPEC Milestone 4):** for each session, `resid_over_static` is close
to 1 (fit residual close to the static noise). A value above 2 prints a
warning: the damped sine does not describe that swing well.

### If it fails

| What you see | Likely cause |
|---|---|
| residual much bigger than the static noise | the bob twists (the card turns away), swings in an ellipse, or the card leaves the sensor's view at the ends of the swing; use a stiffer card and a gentler release |
| some windows "could not be fitted" | readings missing for a long stretch; check for `type=3` events |
| `clean_mm` jumps at the ends of the swing | the sensor sees the support or the wall behind the card: move it closer or use a bigger card |

---

## Milestone 5: fault injector

**Goal:** the injected bursts have the statistics you asked for.

The `ge` command takes the paper's four probabilities **per 0.1 s**
(entry far, entry near, persistence far, persistence near); the firmware
converts them to its own sample period (`docs/methods.md`, section 4.1).
For this check, switch the proximity effect off by making far = near.

No wiring change. In `pio device monitor` type:

```
ge 0.06 0.06 0.7 0.7
mode variance
inject on
```

The **blue LED** flashes on during each burst, and `ge_state` (6th column)
is `1` during bursts. At 30 Hz these 10 Hz values become, per sample,
entry = 1 − 0.94^(1/3) ≈ 0.0204 and exit = 1 − 0.7^(1/3) ≈ 0.112, so the
expected fraction of BAD readings is 0.0204 / (0.0204 + 0.112) ≈ 15 % and
bursts last about 9 readings (0.3 s) on average.

**Check (SPEC Milestone 5):** record 60 s with injection on and compare:

```bash
python analysis/logger.py --port /dev/cu.usbserial-0001 --block C --cmd "ge 0.06 0.06 0.7 0.7" --no-prompt --session-id test_m5 --data /tmp/m5
```

```bash
python -c "import pandas as p; d=p.read_csv('/tmp/m5/raw/test_m5.csv'); b=d.ge_state; print('BAD fraction', b.mean(), ' bursts', ((b.diff()==1).sum()))"
```

The BAD fraction should be between about 0.08 and 0.25 (60 s is short), and
the number of bursts about 1800 × 0.0204 × 0.85 ≈ 30. Then type
`mode frozen` and check that during bursts `faulted_mm` repeats the last
good value. Try the secondary conditions too: `mode bias` (each BAD reading
is offset away from the sensor by 4 × sigma_nom × rho, so the offset grows as
the bob nears the virtual source), `mode randbias` (one random offset per
burst, held for the whole burst, plus extra noise) and `mode sysbias`.

Then calibrate the proximity geometry (`docs/HARDWARE.md`: record Block B
first, run `analysis/calibrate_proximity.py`, set `prox <d_src> <r_ref>`) and
restore the paper's chain with `ge 0.002 0.12 0.5 0.9`: bursts should now
start mostly when the bob is near the virtual source. Delete `/tmp/m5`
when you are done.

## Milestone 6: estimators on the ESP32

**Goal:** the KF, gated KF and IMM run live, and the PC replays them exactly.

First send the shared process noise chosen by `tune.py` (it is printed as
`shared q = ...`; before any tuning the default is a starting value):
`q <value>`, and the reading noise R from Block A to each filter
(`kf <q> <R>`, `gated <q> <R> <gate> <coast_s>`, `imm <q> <R> <kappa_hat> <p_nb> <p_bn>`). Also set the noise settings from Block A
(`noise <sigma_nom_mm> 100`, `docs/HARDWARE.md`).

No wiring change. With the pendulum swinging, look at `kf_pos`, `gated_pos`,
`imm_pos` (metres): they follow `clean_mm / 1000`. `kf_us`, `gated_us`,
`imm_us` (last three columns) show the compute time per update in
microseconds: expect a few µs for the KF and gated KF and a few tens of µs
for the IMM (the ESP32 does double-precision arithmetic in software).

**Check:** record a Block C session and replay it in Python:

```bash
python -c "import sys; sys.path.insert(0,'analysis'); from common import *; from estimators import check_against_firmware as c; p=Paths(); s=list_sessions(p,'C')[-1]; d,m=load_session(p,s); print(s, c(d,m))"
```

All three numbers should be below 1e-6 (metres). Larger numbers mean the C++
and Python code no longer agree: run `make test` on the PC.

---

## Milestone 7: data collection, Blocks C, D and E

No wiring change. The logger sends the right settings for each block
(`mode variance` for C, `mode frozen` for D, injection off for E) and a
different random seed for every session.

```bash
python analysis/logger.py --port /dev/cu.usbserial-0001 --block C
```

| Block | Sessions | What you do |
|---|---|---|
| C | at least 15 × 60 s | swing the pendulum; injection is automatic |
| D | at least 15 × 60 s | swing the pendulum; injection is automatic |
| E | at least 10 × 60 s | swing the pendulum and cause real faults: a hand between sensor and card, a strong lamp or sunlight on the sensor, tapping the I2C wires. **Hold the button down for as long as each fault lasts.** |

Block E safety: tap the wires gently with a finger or a plastic pen. Do not
touch the pins with anything metal (a screwdriver can short 3V3 to GND).

After recording (and again whenever you add sessions):

```bash
python analysis/make_splits.py
```

It never moves a session that already has a split.

---

## Milestone 8: analysis

No hardware. In this order:

```bash
python analysis/ground_truth.py
```

```bash
python analysis/tune.py
```

```bash
python analysis/evaluate.py --crossover --fail-threshold-mm 20
```

```bash
python analysis/figures.py
```

`tune.py` and the crossover sweep take a while (tens of minutes). The
failure threshold is your choice; write it in `docs/test_card.md`.

---

## Milestone 9: integration with Spec 2 (Blocks F and G)

### Block F: measured fault parameters

No wiring change (standard firmware). Copy `results/ge_parameters.json` from
the Spec 2 repository (**not** the demo file) into this repository's
`results/`, check it, then record:

```bash
python analysis/ge_params.py results/ge_parameters.json
```

```bash
python analysis/logger.py --port /dev/cu.usbserial-0001 --block F --ge-params results/ge_parameters.json
```

At least 10 × 60 s. If the Spec 2 confidence intervals are wide,
`ge_params.py` also prints commands for the interval ends; record a few
sessions at each with `--cmd`.

### Block G: integrated live run

> **Follow Spec 2's `docs/safety.md` and `docs/BRING_UP.md` for the guard
> circuit.** In this run the VL53L0X is powered through the Spec 2
> protection circuit (P-MOSFET switch, INA219), not straight from 3V3.

Wiring: build the Spec 2 circuit exactly as in its bring-up guide. The
VL53L0X keeps SDA → GPIO 21, SCL → GPIO 22, XSHUT → GPIO 19, but its VIN
comes from the protected rail. The INA219 uses GPIO 16/17, the rail switch
GPIO 25, the latch-up trigger GPIO 26. The button stays on GPIO 18.

Flash the guard build:

```bash
pio run -e esp32dev_guard -t upload
```

At boot you should see `build=guard`, and no `# warning INA219 not found`.
In this build the guard controls the blue LED (lit while the sensor rail is on).

**Check:** pull SDA as in Milestone 2. You should see `# guard_event` lines
with `type=3` and later `type=5`, and the same codes in the `recovery_event`
column. Then trigger simulated latch-ups as Spec 2 describes: `type=1`
events, rail off for `off_ms`, then recovery.

Record at least 10 × 60 s with the pendulum swinging, causing lock-ups and
latch-ups during the swing:

```bash
python analysis/logger.py --port /dev/cu.usbserial-0001 --block G
```

The logger warns if a Block G session was recorded with the standard build
(or another block with the guard build).

---

## Milestone 10: write-up

Run Milestone 8 again with all sessions, then fill in the README results
section and `docs/test_card.md` from the **test** sessions only
(`results/tables/test_card_values.json` has most of the numbers).
