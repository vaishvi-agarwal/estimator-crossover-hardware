# Hardware setup

> **Photos and measured dimensions: to be added by Vaishvi once the rig is
> built.** Nothing on this page has been measured yet; the numbers are the
> targets from SPEC.md section 3.

## Parts

| Part | Notes |
|---|---|
| ESP32 DevKit V1 (ESP32-WROOM-32, 30 pins) | Arduino framework via PlatformIO |
| VL53L0X time-of-flight breakout | read with the Pololu `VL53L0X` library 1.3.1 |
| Tactile push button | marks physical faults (Block E) |
| White matte card, about 10 × 10 cm | the target, glued flat to the bob |
| String about 0.5 m, small weight | the pendulum |
| Rigid support | doorframe hook, retort stand or shelf |
| Heavy base | holds the sensor still |
| Block G only: the Spec 2 protection circuit | P-MOSFET switch, INA219 current sensor, see the Spec 2 repository |

Record the exact part numbers and board versions here (test card field 7):

| Item | Part number / version |
|---|---|
| ESP32 board | _to be added_ |
| VL53L0X breakout | _to be added_ |
| USB cable / computer | _to be added_ |

## Wiring (ESP32 DevKit V1)

| VL53L0X pin | ESP32 pin | Notes |
|---|---|---|
| VIN | 3V3 | 3.3 V (not 5 V: keeps the I2C lines at 3.3 V) |
| GND | GND | |
| SDA | GPIO 21 | I2C data |
| SCL | GPIO 22 | I2C clock |
| XSHUT | GPIO 19 | sensor reset, used to recover from hangs |

| Other | ESP32 pin | Notes |
|---|---|---|
| Tactile button to GND | GPIO 18 (INPUT_PULLUP) | held down while a physical fault is present |
| On-board LED | GPIO 2 | lit while a fault is being injected (in the guard build: lit while the sensor rail is on) |

Block G additionally uses the Spec 2 guard's pins: GPIO 16/17 (INA219 on
its own I2C bus), GPIO 25 (rail enable), GPIO 26 (simulated latch-up
trigger). See `lib/sel_guard/src/pins.h` and the Spec 2 repository.

_Wiring diagram: to be added (photo or drawing)._

## Pendulum geometry

```
     support
       |
       |  string, length L (about 0.5 m)
       |
       |
     [card]  <- bob with the white card facing the sensor
       ^
       |  swings towards / away from the sensor
       v
                 rest distance about 25-35 cm
     [card] ............................ [VL53L0X] on a heavy base
```

The sensor points horizontally along the swing direction, so the distance
it measures goes up and down like a damped sine wave.

| Quantity | Target | Measured (per session, in the JSON sidecar) |
|---|---|---|
| String length L | about 0.5 m (period about 1.4 s) | `user.string_length_m` |
| Rest distance | 25-35 cm | `user.rest_distance_mm` |
| Swing amplitude | 5-10 cm | `user.amplitude_mm` |
| Card size | about 10 × 10 cm | `user.card_size_cm` |
| Room temperature | as it is, recorded every session | `user.temp_c` |

_Photo of the rig: to be added._
