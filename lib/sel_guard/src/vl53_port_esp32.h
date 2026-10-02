// vl53_port_esp32.h — switch the VL53L0X on and off safely, and read it
// without blocking. ESP32/Arduino only.
//
// THE I2C BACK-POWERING PROBLEM (read docs/circuit_explained.md for the long
// version): every I2C chip has small protection diodes from its SDA/SCL pins
// to its own supply pin. If we cut the sensor's power but the ESP32 keeps the
// I2C lines HIGH, current flows from the lines, through those diodes, into the
// "unpowered" sensor. Two bad things follow:
//   1. the sensor is half-powered, so a "power cycle" may not really reset it;
//   2. the sensor drags the I2C lines down, so nothing else on that bus works.
//
// What this file does about it:
//   * before power-off: hold XSHUT low, stop the I2C peripheral, and turn the
//     ESP32's SDA/SCL pins into plain inputs (no pull-ups, not driven), so no
//     current can flow into the sensor;
//   * after power-on: wait for the sensor to boot, then restart I2C and
//     initialise the sensor from scratch.
// And in pins.h the INA219 is on a different I2C bus, so the guard can always
// measure current, whatever state the sensor is in.
#pragma once
#ifdef ARDUINO
#include <Arduino.h>
#include <Wire.h>
#include <VL53L0X.h>

#include "guard_types.h"

class Vl53Port {
 public:
  // Time from XSHUT release to the sensor accepting I2C. The datasheet boot
  // time is about 1.2 ms; we wait longer to be safe.
  // TODO(hardware): confirm on the logic analyzer that init succeeds every time.
  static const uint32_t BOOT_MS = 5;
  // How long XSHUT is held low for a hardware reset (ladder step b).
  // TODO(hardware): 10 ms follows Spec 1; check in Block D that it is long enough.
  static const uint32_t XSHUT_LOW_MS = 10;

  void configure(TwoWire& bus, uint8_t sda, uint8_t scl, uint8_t xshut, uint32_t i2c_hz,
                 uint32_t timing_budget_us) {
    bus_ = &bus;
    sda_ = sda;
    scl_ = scl;
    xshut_ = xshut;
    hz_ = i2c_hz;
    budget_us_ = timing_budget_us;
    holdInReset();
    floatPins();
  }

  // Call this BEFORE switching the protected rail off.
  void releaseBus() {
    holdInReset();
    if (attached_) {
      bus_->end();
      attached_ = false;
    }
    floatPins();
    running_ = false;
  }

  // Call this AFTER switching the rail on. Full start-up: release XSHUT,
  // wait for boot, start I2C, initialise, begin continuous ranging.
  bool start() {
    releaseReset();
    delay(BOOT_MS);
    return initSensor();
  }

  // Ladder step (a): re-initialise over I2C without any reset.
  bool reinit() { return initSensor(); }

  // Ladder step (b): XSHUT hardware reset, then a full start.
  bool xshutReset() {
    if (attached_) {
      bus_->end();
      attached_ = false;
    }
    holdInReset();
    delay(XSHUT_LOW_MS);
    return start();
  }

  // Non-blocking read. The Pololu library's readRangeContinuousMillimeters()
  // WAITS for data (up to 33 ms), which would stall everything else, so we
  // read the "data ready" flag ourselves and only fetch the distance when it
  // is there. Register addresses are from the Pololu library / ST API.
  PollResult poll(uint16_t& mm) {
    if (!running_) return PollResult::BUS_ERROR;
    const uint8_t status = dev_.readReg(VL53L0X::RESULT_INTERRUPT_STATUS);
    if (dev_.last_status != 0) return PollResult::BUS_ERROR;
    if ((status & 0x07) == 0) return PollResult::NOT_READY;
    mm = dev_.readReg16Bit(VL53L0X::RESULT_RANGE_STATUS + 10);
    if (dev_.last_status != 0) return PollResult::BUS_ERROR;
    dev_.writeReg(VL53L0X::SYSTEM_INTERRUPT_CLEAR, 0x01);
    if (dev_.last_status != 0) return PollResult::BUS_ERROR;
    return PollResult::READY;
  }

  // Let the sensor boot (XSHUT released) without starting I2C. Used by
  // inrush_test so the sensor's own start-up current is part of the inrush.
  void bootOnly() { releaseReset(); }

  // Lock-up injection for sefi_test method "sdalow": take SDA away from the
  // I2C peripheral, hold it LOW for `ms` while clocking SCL a few times (a
  // garbage bus transaction, like an electrical glitch), then hand it back.
  // Whether this actually hangs the sensor is something Block D measures.
  void injectSdaLow(uint32_t ms) {
    if (attached_) {
      bus_->end();
      attached_ = false;
    }
    pinMode(sda_, OUTPUT);
    digitalWrite(sda_, LOW);
    pinMode(scl_, OUTPUT);
    const uint32_t t0 = millis();
    while (millis() - t0 < ms) {
      digitalWrite(scl_, HIGH);
      delayMicroseconds(50);
      digitalWrite(scl_, LOW);
      delayMicroseconds(50);
    }
    floatPins();
    attachBus();
  }

  // Drive XSHUT low (sensor held in reset). Public so the latch-up task can
  // call it right after cutting power. A GPIO write is safe from either core.
  void holdInReset() {
    pinMode(xshut_, OUTPUT);
    digitalWrite(xshut_, LOW);
  }

  bool running() const { return running_; }
  VL53L0X& device() { return dev_; }  // used by the lock-up injection methods

 private:
  bool initSensor() {
    attachBus();
    dev_.setBus(bus_);
    dev_.setTimeout(100);  // ms; only used inside init()
    if (!dev_.init()) {
      running_ = false;
      return false;
    }
    dev_.setMeasurementTimingBudget(budget_us_);
    dev_.startContinuous(0);  // back-to-back measurements
    running_ = (dev_.last_status == 0);
    return running_;
  }

  void attachBus() {
    if (!attached_) {
      bus_->begin(sda_, scl_, hz_);
      attached_ = true;
    }
  }

  // Let go of XSHUT; the module's own pull-up (to its 2.8 V supply) takes it
  // high. We never drive it HIGH from the 3.3 V ESP32.
  void releaseReset() { pinMode(xshut_, INPUT); }

  // SDA/SCL as plain inputs: not driven, no internal pull-ups.
  void floatPins() {
    pinMode(sda_, INPUT);
    pinMode(scl_, INPUT);
  }

  TwoWire* bus_ = nullptr;
  VL53L0X dev_;
  uint8_t sda_ = 21, scl_ = 22, xshut_ = 19;
  uint32_t hz_ = 400000;
  uint32_t budget_us_ = 33000;
  bool attached_ = false;
  bool running_ = false;
};

#endif  // ARDUINO
