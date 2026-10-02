// guard_types.h — small shared types used by several guard headers.
// Plain C++ (no Arduino), so the host tests can use them too.
#pragma once
#include <stdint.h>

// Result of asking the VL53L0X "do you have a new distance for me?"
enum class PollResult : uint8_t {
  NOT_READY = 0,  // no new measurement yet (normal; try again later)
  READY = 1,      // a new distance was read
  BUS_ERROR = 2,  // the I2C transfer failed (sensor not answering)
};

// Why the sensor output stopped being trustworthy.
enum class SensorFault : uint8_t {
  NONE = 0,
  TIMEOUT = 1,      // no new reading for too long
  INVALID = 2,      // the reading was 65535 (the VL53L0X "hung" value)
  STUCK = 3,        // N identical readings in a row
  BUS_ERROR = 4,    // I2C transfer failed
  INIT_FAILED = 5,  // the sensor did not accept initialisation
  POWER_OFF = 6,    // the latch-up guard switched the power off
};

// Steps of the lock-up recovery ladder, in escalation order.
enum class LadderStep : uint8_t {
  NONE = 0,         // no ladder step was needed (e.g. power came back after a latch-up trip)
  REINIT = 1,       // (a) re-initialise over I2C
  XSHUT = 2,        // (b) hardware reset through the XSHUT pin
  POWER_CYCLE = 3,  // (c) full power cycle through the protection switch
  FAILED = 4,       // every step failed: the sensor is marked FAILED
};

inline const char* faultName(SensorFault f) {
  switch (f) {
    case SensorFault::NONE: return "none";
    case SensorFault::TIMEOUT: return "timeout";
    case SensorFault::INVALID: return "invalid";
    case SensorFault::STUCK: return "stuck";
    case SensorFault::BUS_ERROR: return "bus";
    case SensorFault::INIT_FAILED: return "init";
    case SensorFault::POWER_OFF: return "power_off";
  }
  return "?";
}

inline const char* stepName(LadderStep s) {
  switch (s) {
    case LadderStep::NONE: return "none";
    case LadderStep::REINIT: return "reinit";
    case LadderStep::XSHUT: return "xshut";
    case LadderStep::POWER_CYCLE: return "power";
    case LadderStep::FAILED: return "failed";
  }
  return "?";
}

// Everything the guard reports. Spec 1 logs these next to its estimator output.
enum class EventType : uint8_t {
  NONE = 0,
  SEL_TRIP = 1,          // latch-up detected, power removed (peak_mA is set)
  SEL_LOCKOUT = 2,       // too many trips in a row: power stays off until reset
  SENSOR_FAULT = 3,      // the sensor output went bad (fault is set): an outage starts
  LADDER_STEP = 4,       // one recovery step finished (step, ok, duration_us are set)
  SENSOR_RECOVERED = 5,  // good readings again: the outage ended (step = what fixed it)
  SENSOR_FAILED = 6,     // the ladder gave up
  CURRENT_SENSOR_ERROR = 7,  // the INA219 stopped answering: the guard is blind
};

struct GuardEvent {
  uint32_t seq = 0;          // increases by one for every event; lets callers spot new ones
  EventType type = EventType::NONE;
  uint64_t t_us = 0;         // when it happened (microseconds since boot)
  SensorFault fault = SensorFault::NONE;
  LadderStep step = LadderStep::NONE;
  bool ok = false;           // LADDER_STEP: did this step work?
  uint32_t duration_us = 0;  // LADDER_STEP: step time; SENSOR_RECOVERED: outage length
  uint64_t outage_start_us = 0;  // SENSOR_RECOVERED: time of the last good reading before the outage
  float peak_mA = 0.0f;      // SEL_TRIP: highest current in the tripping run
};
