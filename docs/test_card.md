# Test card

Test conditions reported with the seven fields of Vaishvi's Minimum
Reporting Standard, adapted from radiation testing to fault injection.

> **Status: not filled in. No hardware test has been run yet.** Fill this in
> from the **test** sessions only, after the final run of `evaluate.py`.
> Most values are computed for you in `results/tables/test_card_values.json`;
> the rest come from `docs/hardware_setup.md` and the session sidecars.

One column per condition. `_..._` = to be filled in.

| Field | Variance bursts (C) | Frozen bursts (D) | Physical faults (E) | Measured GE (F) | Integrated run (G) |
|---|---|---|---|---|---|
| **1. Fault type and source** | injected GE, inflated variance (software, on real readings) | injected GE, frozen output (software) | physical: hand occlusion, strong light, I2C wire tapping | injected GE with parameters measured in Spec 2 | real sensor lock-ups and simulated latch-ups, recovered by the Spec 2 guard |
| **2. Fault rate** (entry far/near and persistence far/near per 0.1 s; r_ref, d_src in mm; sample period) | paper chain 0.002/0.12, 0.5/0.9; r_ref, d_src: _..._ | same | not controlled (report bursts per minute: _..._) | from Spec 2: _..._ (with CI) | not controlled (report outages per minute: _..._) |
| **3. Total exposure** (number of bursts; total BAD time in s; total test time in s) | _..._ | _..._ | _..._ | _..._ | _..._ |
| **4. Operating state** (sensor mode, timing budget, supply voltage) | continuous ranging, 33 ms, _... V_ | same | same | same | same, sensor powered through the guard rail: _... V_ |
| **5. Ambient temperature** (°C, min-max over sessions) | _..._ | _..._ | _..._ | _..._ | _..._ |
| **6. Failure criterion** (session burst-regime p95 error above _... mm_): share of test sessions failing, per estimator | _..._ | _..._ | _..._ | _..._ | _..._ |
| **7. Device identity** (see below) | | | | | |

## Field 7: device identity

| Item | Value |
|---|---|
| ESP32 board (part number, revision) | _..._ |
| VL53L0X breakout (part number, version) | _..._ |
| Spec 2 guard hardware (Block G) | _..._ |
| PlatformIO platform | espressif32 @ 6.9.0 (Arduino core 2.0.x) |
| VL53L0X library | pololu/VL53L0X @ 1.3.1 |
| Guard library (Block G) | sel-sefi-sensor-guard, copied at commit c7fe1a916e (`lib/sel_guard/ORIGIN.md`) |
| Firmware commit(s) | _..._ (`firmware_commits` in `test_card_values.json`) |
| Analysis commit | _..._ |
| Python and package versions | Python 3.11, `requirements.txt` |

## Where each number comes from

| Field | Source |
|---|---|
| 2 | `fault_rate` in `results/tables/test_card_values.json` (from each session's `# params` line) |
| 3 | `exposure` in the same file |
| 4 | `operating_state` (mode, timing budget; supply voltage typed in by Vaishvi at each session) |
| 5 | `ambient_temperature_c` |
| 6 | `failure_criterion`: run `python analysis/evaluate.py --fail-threshold-mm <value>` |
| 7 | `device_identity`, plus `docs/hardware_setup.md` |
