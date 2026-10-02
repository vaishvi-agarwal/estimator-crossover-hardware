// ina219_fast.h — read the INA219 current sensor as fast as possible.
//
// The usual INA219 libraries read bus voltage, shunt voltage, current and power
// one after another, which takes milliseconds. To catch a latch-up quickly we
// only need the SHUNT voltage (the tiny voltage across the 0.1 Ohm resistor,
// which is proportional to current). So this driver:
//   1. puts the chip in "shunt voltage only, continuous" mode,
//   2. uses the fastest 9-bit conversion (about 84 microseconds each),
//   3. points the chip's register pointer at the shunt register ONCE,
//   4. then each reading is a single 2-byte I2C read (no address write needed).
//
// The top half of this file is plain C++ (no Arduino), so the maths can be
// unit-tested on a PC. The Arduino driver class is at the bottom, inside
// `#ifdef ARDUINO`.
#pragma once
#include <stdint.h>

namespace ina219 {

// Register addresses (INA219 datasheet, "Register Maps").
constexpr uint8_t REG_CONFIG = 0x00;
constexpr uint8_t REG_SHUNT = 0x01;

// Programmable gain (PGA) = the shunt-voltage range.
// With a 0.1 Ohm shunt: DIV1 = +/-400 mA, DIV8 = +/-3.2 A.
// We default to DIV8 so the capacitor inrush (which can briefly reach amps)
// is not clipped. TODO(hardware): check with `scope` whether DIV8 noise is acceptable.
enum class Pga : uint8_t { DIV1_40mV = 0, DIV2_80mV = 1, DIV4_160mV = 2, DIV8_320mV = 3 };

// Shunt ADC resolution. Fewer bits = faster conversion, more noise.
enum class Adc : uint8_t { BITS9 = 0x0, BITS10 = 0x1, BITS11 = 0x2, BITS12 = 0x3 };

// MODE bits 2..0 = 0b101: shunt voltage, continuous.
constexpr uint16_t MODE_SHUNT_CONTINUOUS = 0x5;

// Build the 16-bit configuration register value.
// Bit layout: [13] bus range, [12:11] PGA, [10:7] bus ADC, [6:3] shunt ADC, [2:0] mode.
// The bus range and bus ADC fields are left at 0 because we never read bus voltage.
inline uint16_t configWord(Pga pga, Adc shunt_adc) {
  return (uint16_t)(((uint16_t)pga << 11) | ((uint16_t)shunt_adc << 3) | MODE_SHUNT_CONTINUOUS);
}

// One count of the shunt register is always 10 microvolts (= 0.01 mV).
// Ohm's law: current (mA) = voltage (mV) / resistance (Ohm).
// Example: raw = 700 -> 7.00 mV -> 7.00 / 0.1 = 70 mA.
inline float rawToMilliamps(int16_t raw, float shunt_ohm) {
  return (float)raw * 0.01f / shunt_ohm;
}

// The largest current each PGA setting can measure, in mA.
inline float fullScaleMilliamps(Pga pga, float shunt_ohm) {
  const float range_mV[4] = {40.0f, 80.0f, 160.0f, 320.0f};
  return range_mV[(uint8_t)pga] / shunt_ohm;
}

// Datasheet conversion times in microseconds (single sample, no averaging).
inline uint16_t conversionTimeUs(Adc adc) {
  const uint16_t t[4] = {84, 148, 276, 532};
  return t[(uint8_t)adc];
}

// The chip sends the most significant byte first.
inline int16_t bytesToRaw(uint8_t msb, uint8_t lsb) {
  return (int16_t)(((uint16_t)msb << 8) | lsb);
}

}  // namespace ina219

#ifdef ARDUINO
#include <Wire.h>

class Ina219Fast {
 public:
  // shunt_ohm: the resistor on your INA219 board. Most boards use "R100" = 0.1 Ohm.
  // TODO(hardware): measure the real shunt (or calibrate against a multimeter
  // in series with a known load) and update DEFAULT_SHUNT_OHM in main.cpp.
  bool begin(TwoWire& bus, uint8_t addr, float shunt_ohm,
             ina219::Pga pga = ina219::Pga::DIV8_320mV,
             ina219::Adc adc = ina219::Adc::BITS9) {
    bus_ = &bus;
    addr_ = addr;
    shunt_ohm_ = shunt_ohm;
    pga_ = pga;
    // Step 1: write the configuration register (pointer byte + 2 data bytes).
    const uint16_t cfg = ina219::configWord(pga, adc);
    bus_->beginTransmission(addr_);
    bus_->write(ina219::REG_CONFIG);
    bus_->write((uint8_t)(cfg >> 8));
    bus_->write((uint8_t)(cfg & 0xFF));
    if (bus_->endTransmission() != 0) return false;  // no answer: check wiring
    // Step 2: point the chip at the shunt register. After this, every read
    // returns the shunt voltage without sending the register address again.
    bus_->beginTransmission(addr_);
    bus_->write(ina219::REG_SHUNT);
    ok_ = (bus_->endTransmission() == 0);
    return ok_;
  }

  // One fast reading. Returns false if the I2C read failed.
  bool readRaw(int16_t& raw) {
    if (bus_->requestFrom(addr_, (uint8_t)2) != 2) return false;
    const uint8_t msb = bus_->read();
    const uint8_t lsb = bus_->read();
    raw = ina219::bytesToRaw(msb, lsb);
    return true;
  }

  // Convenience: read and convert to milliamps in one call.
  bool readMilliamps(float& mA) {
    int16_t raw;
    if (!readRaw(raw)) return false;
    mA = ina219::rawToMilliamps(raw, shunt_ohm_);
    return true;
  }

  float shuntOhm() const { return shunt_ohm_; }
  ina219::Pga pga() const { return pga_; }
  bool ok() const { return ok_; }

 private:
  TwoWire* bus_ = nullptr;
  uint8_t addr_ = 0x40;
  float shunt_ohm_ = 0.1f;
  ina219::Pga pga_ = ina219::Pga::DIV8_320mV;
  bool ok_ = false;
};
#endif  // ARDUINO
