// board_pins.h — which ESP32 pin is wired to what (SPEC.md section 3.2).
//
// Keep this file in sync with docs/hardware_setup.md and docs/BRING_UP.md.
#pragma once
#include <stdint.h>

namespace board {

// VL53L0X distance sensor, on I2C bus 0 ("Wire").
constexpr uint8_t SENSOR_SDA = 21;  // I2C data
constexpr uint8_t SENSOR_SCL = 22;  // I2C clock

// VL53L0X XSHUT (shutdown) pin. Driven LOW = sensor held in reset.
// Released (pin set to INPUT) = the breakout board's own pull-up lets the
// sensor run. We never drive it HIGH (same rule as the Spec 2 guard).
constexpr uint8_t SENSOR_XSHUT = 19;

// Tactile button between this pin and GND. INPUT_PULLUP means the pin reads
// HIGH when the button is up and LOW while it is pressed.
// Vaishvi holds it down while a physical fault is present (Block E).
constexpr uint8_t BUTTON = 18;

// The blue LED on the DevKit board: lit while a fault is being injected.
constexpr uint8_t LED = 2;

}  // namespace board
