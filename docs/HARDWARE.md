# Values that can only be found on real hardware

Every value below is a **sensible default** in the code, marked
`// TODO(hardware)` (or `# TODO(hardware)`) where it is defined. None of them
has been measured yet. When you measure one, update the code, delete the
TODO comment, tick the box, and note the measured value and the session ID.

Find them all with:

```bash
grep -rn "TODO(hardware)" firmware analysis
```

The milestone numbers refer to SPEC.md section 10 and to `docs/BRING_UP.md`.

## Serial link and sensor timing

- [ ] **No rows lost at 921600 baud.** Not a code value: check that
  `logger.py` reports `0 lost` for a 60 s recording. If rows are lost, try a
  shorter USB cable or another USB port. Milestone 1.
- [ ] **`Vl53Reader::BOOT_MS`** (default 5 ms): wait after releasing XSHUT
  before talking to the sensor. `firmware/include/vl53_reader.h`.
  How: in Milestone 2, unplug and replug SDA 10 times; every recovery event
  should end with `ok=1` on the first `type=4` line. Milestone 2.

## Sensor guard (hang detection)

- [ ] **`SensorGuardConfig::timeout_us`** (default 200 ms = 6 missed
  readings). `firmware/include/sensor_guard.h`. How: a 10-minute recording
  of a swinging pendulum must contain no `reason=timeout` events.
  Milestone 2.
- [ ] **`SensorGuardConfig::stuck_count`** (default 30 identical readings,
  about 1 s). `firmware/include/sensor_guard.h`. How: after Block A, look at
  the `longest_equal_run` column of `results/tables/noise_summary.csv`. The
  setting must be well above the largest value at every distance.
  Milestone 3.

## Pendulum and injector

- [ ] **Proximity calibration: `GeConfig::d_src_mm` and `GeConfig::r_ref_mm`**
  (placeholders 220 mm and 40 mm). `firmware/include/ge_injector.h`
  (`TODO(hardware)`), set on the board with `prox <d_src_mm> <r_ref_mm>`.
  The paper chose r_ref (8 m) by calibration: widened until a typical pass
  gave enough BAD samples, with the proximity factor rho between 0.65 and
  0.99 at closest approach. Do the same on the pendulum. How: record Block B,
  then run `python analysis/calibrate_proximity.py --d-src <mm> --r-ref 20,30,40,50,60`.
  Put d_src a little in front of the bob's typical closest approach, and pick
  the r_ref whose `rho_median` / `rho_min` fall in 0.65-0.99 and whose
  `bursts_per_min` gives enough bursts per 60 s session. Record the chosen
  values and the session IDs here. Milestone 4/5.
- [ ] **`GeConfig::sample_period_s`** (default 1/30 s): the period used to
  convert the paper's 10 Hz probabilities. `firmware/include/ge_injector.h`,
  set with `period <s>`. How: the mean of the differences of `t_us` in the
  Block B recordings (in seconds). Milestone 4.
- [ ] **`GeConfig::sigma_nom_mm`** (default 2 mm): nominal reading noise R =
  sigma², used for the inflated variance kappa · R and to scale the biases
  (c = 4 sigma, sigma_b = 10 sigma). `firmware/include/ge_injector.h`,
  set with `noise <sigma_nom_mm> 100`. How: sigma at the pendulum's rest
  distance from `results/tables/noise_model.json` (Block A). Milestone 3.

## Estimator noise settings

- [ ] **Reading-noise variance `r`** in `KfParams`, `GatedParams` and
  `ImmParams` (default 4e-6 m², which is (2 mm)²).
  `firmware/include/kalman1d.h`, `gated_kf.h`, `imm1d.h`.
  How: after Block A, take sigma at the pendulum's rest distance from
  `results/tables/noise_model.json` and set r = (sigma / 1000)².
  As in the paper, R is not tuned: `tune.py` reads it from Block A
  automatically, so this default only matters for the live estimates
  printed by the firmware.
  Milestone 3/6.

## Physical faults (Block E)

- [ ] **`BUTTON_MARGIN_S`** (default 0.5 s): readings this close to a
  button press are left out of the truth fit.
  `analysis/ground_truth.py`. How: plot a few Block E sessions
  (`clean_mm` and `button` against time) and check that the margin covers
  the delay between the fault starting/ending and your press/release.
  Milestone 7.

## Integrated run (Block G)

- [ ] **Latch-up operating point** for the guard (`threshold_mA`,
  `debounce`, `blank_us`, `off_us`). `firmware/src/main.cpp`, in `setup()`
  under `USE_SEL_GUARD`. How: copy the values from Spec 2's
  `results/tables/operating_point.json` once Spec 2 has measured them.
  Milestone 9.
- [ ] **Guard library values.** `lib/sel_guard/src/` is a copy of the Spec 2
  guard and contains its own `TODO(hardware)` markers (shunt resistance,
  INA219 noise, hang timeouts, ladder settings). They are measured in the
  Spec 2 repository and listed in its `docs/HARDWARE.md`. When they change
  there, copy the library here again (see `lib/sel_guard/ORIGIN.md`).
  Milestone 9.
