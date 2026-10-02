# Where this folder came from

This is a **copy** of the latch-up / lock-up guard library from the sibling
repository `sel-sefi-sensor-guard` (Spec 2, also by Vaishvi Agarwal).

| | |
|---|---|
| Source repository | `sel-sefi-sensor-guard` (side by side with this repository in `claude-code-kit/`) |
| Source folder | `firmware/include/` (every `.h` file) and `LICENSE` |
| Source commit | `c7fe1a916e53ba24a90c5cb66111816f5f6fe17a` |
| Copied on | 2026-10-02 |
| Licence | MIT (see `LICENSE` in this folder) |

The files in `src/` are **unchanged**. Only `library.json` and this note were
added, so PlatformIO can use the folder as a library.

## Why a copy and not a link?

A copy ("vendoring") means this repository builds on its own, on any computer
and in GitHub Actions, without the other repository next to it. The price is
that fixes made in Spec 2 do not arrive here by themselves.

## How to update it

1. In `sel-sefi-sensor-guard`, commit your change and note the new commit hash.
2. Copy `firmware/include/*.h` into `lib/sel_guard/src/` again (overwrite).
3. Update the commit hash and date in the table above.
4. Run `pio run -e esp32dev_guard` and `make test` here.

## How it is used here

Only the `esp32dev_guard` build (Block G, the integrated live run) uses this
library. In that build the guard owns the VL53L0X and this repository's own
`sensor_guard.h` is not used. See `docs/methods.md` (Block G) and
`docs/BRING_UP.md` (milestone 9).
