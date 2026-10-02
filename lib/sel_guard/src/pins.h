// pins.h — which ESP32 pin is wired to what.
//
// Keep this file in sync with docs/circuit_explained.md and docs/BRING_UP.md.
//
// One change from the pin plan in SPEC.md section 3.2: the INA219 current
// sensor is on its OWN I2C bus (GPIO 16/17) instead of sharing GPIO 21/22 with
// the VL53L0X. Reason: when the VL53L0X is switched off, its I2C pins can pull
// the shared bus low (see "I2C back-powering" in docs/circuit_explained.md).
// If the INA219 shared that bus, the guard would go blind exactly when it
// matters. On a separate bus, the current sensor keeps working no matter what
// the VL53L0X does.
#pragma once
#include <stdint.h>

namespace pins {

// I2C bus 0 ("Wire"): VL53L0X only. Its power is switched by the guard.
constexpr uint8_t SENSOR_SDA = 21;
constexpr uint8_t SENSOR_SCL = 22;

// I2C bus 1 ("Wire1"): INA219 only. Always powered from the ESP32 3.3 V pin.
constexpr uint8_t INA_SDA = 16;
constexpr uint8_t INA_SCL = 17;

// HIGH = protected rail ON. Drives the NPN base through 1 kOhm; the NPN pulls
// the P-MOSFET gate low, which switches the MOSFET on.
// LOW (and the default at boot) = rail OFF.
constexpr uint8_t RAIL_ENABLE = 25;

// HIGH = simulated latch-up ON (N-MOSFET connects the 47 Ohm load across the rail).
// A 10 kOhm pull-down keeps it OFF while the ESP32 boots.
constexpr uint8_t LATCH_TRIGGER = 26;

// VL53L0X XSHUT (shutdown) pin. Driven LOW = sensor held in reset.
// Released (set to INPUT) = the module's own pull-up lets the sensor run.
// We never drive it HIGH, because the sensor runs at 2.8 V internally.
constexpr uint8_t SENSOR_XSHUT = 19;

// The blue LED on the DevKit board. Lit while the rail is ON.
constexpr uint8_t STATUS_LED = 2;

}  // namespace pins

// I2C addresses (7-bit).
constexpr uint8_t INA219_ADDR = 0x40;
constexpr uint8_t VL53L0X_ADDR = 0x29;
