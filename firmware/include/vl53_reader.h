// vl53_reader.h — start the VL53L0X and read it WITHOUT blocking.
// ESP32/Arduino only (the decision logic is in sensor_guard.h, which is not).
//
// Why "without blocking"? The Pololu library's readRangeContinuousMillimeters()
// waits until a reading is ready. If the sensor hangs, the whole program
// waits with it, and we could not log the hang. Instead, poll() asks the
// sensor "is a reading ready?" and returns at once if not.
#pragma once
#ifdef ARDUINO
#include <Arduino.h>
#include <VL53L0X.h>
#include <Wire.h>

#include "board_pins.h"
#include "sensor_guard.h"  // for ReadResult

class Vl53Reader {
 public:
  // Timing budget = how long the sensor spends on each measurement.
  // 33 ms gives about 30 readings per second (SPEC.md section 3.3).
  static const uint32_t TIMING_BUDGET_US = 33000;
  // How long XSHUT is held low for a hardware reset (SPEC.md section 3.3).
  static const uint32_t XSHUT_LOW_MS = 10;
  // Time from releasing XSHUT until the sensor answers on I2C. The datasheet
  // boot time is about 1.2 ms; we wait longer to be safe.
  // TODO(hardware): confirm that re-initialisation succeeds every time with 5 ms.
  static const uint32_t BOOT_MS = 5;

  // Set up the pins and the I2C bus, then start the sensor.
  bool begin() {
    pinMode(board::SENSOR_XSHUT, INPUT);  // released: sensor runs
    Wire.begin(board::SENSOR_SDA, board::SENSOR_SCL, 400000);
    // If the sensor holds the bus, give up after 20 ms instead of freezing.
    Wire.setTimeOut(20);
    delay(BOOT_MS);
    return start();
  }

  // Initialise the sensor and start continuous ranging.
  bool start() {
    sensor_.setTimeout(50);  // ms, only used inside init()
    if (!sensor_.init()) return false;
    if (!sensor_.setMeasurementTimingBudget(TIMING_BUDGET_US)) return false;
    sensor_.startContinuous();  // back-to-back measurements
    return sensor_.last_status == 0;
  }

  // Hardware reset: XSHUT low for 10 ms, then initialise again from scratch.
  // Blocks for about 20 ms; only called while recovering from a hang.
  bool xshutReset() {
    pinMode(board::SENSOR_XSHUT, OUTPUT);
    digitalWrite(board::SENSOR_XSHUT, LOW);
    delay(XSHUT_LOW_MS);
    pinMode(board::SENSOR_XSHUT, INPUT);
    delay(BOOT_MS);
    return start();
  }

  // Ask for a new reading. Never waits.
  ReadResult poll(uint16_t& mm) {
    // Bits 0-2 of this register are non-zero when a new result is ready.
    const uint8_t status = sensor_.readReg(VL53L0X::RESULT_INTERRUPT_STATUS);
    if (sensor_.last_status != 0) return ReadResult::BUS_ERROR;
    if ((status & 0x07) == 0) return ReadResult::NOT_READY;
    // The distance in mm is 10 bytes after RESULT_RANGE_STATUS
    // (the same register the Pololu library reads).
    mm = sensor_.readReg16Bit(VL53L0X::RESULT_RANGE_STATUS + 10);
    if (sensor_.last_status != 0) return ReadResult::BUS_ERROR;
    // Tell the sensor we have taken the result, so it can flag the next one.
    sensor_.writeReg(VL53L0X::SYSTEM_INTERRUPT_CLEAR, 0x01);
    if (sensor_.last_status != 0) return ReadResult::BUS_ERROR;
    return ReadResult::READY;
  }

 private:
  VL53L0X sensor_;
};

#endif  // ARDUINO
