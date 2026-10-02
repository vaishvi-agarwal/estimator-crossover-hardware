// sensor_guard.h — decides when the VL53L0X has stopped working and when to
// reset it. Plain C++ (no Arduino), so tests/test_host.cpp can check it on a PC.
//
// The firmware calls onPoll() after every attempt to read the sensor. The
// guard answers three questions:
//   * Can this reading be trusted?           (GuardStep::reading_ok)
//   * Should main.cpp reset the sensor now?   (GuardStep::request_recovery)
//   * Did something worth logging happen?     (GuardStep::event)
// main.cpp does the actual reset (XSHUT low for 10 ms, re-initialise) and
// reports back with onRecoveryAttempt().
//
// What counts as "stopped working" (SPEC.md sections 3.3 and 4.1):
//   TIMEOUT    no new reading for timeout_us
//   INVALID    the reading is 65535 (the value a hung VL53L0X returns)
//   STUCK      stuck_count identical readings in a row
//   BUS_ERROR  bus_error_count I2C failures in a row
// Out-of-range readings (8190 / 8191 mm: "nothing in front of me") are NOT a
// hang: they are simply not used (reading_ok = false, no recovery).
#pragma once
#include <stdint.h>

// Result of one attempt to read the sensor.
enum class ReadResult : uint8_t {
  NOT_READY = 0,  // no new measurement yet (normal; try again later)
  READY = 1,      // a new distance was read
  BUS_ERROR = 2,  // the I2C transfer failed (sensor not answering)
};

// Codes written to the recovery_event column of the CSV.
// They are the SAME numbers as the Spec 2 guard's EventType
// (lib/sel_guard/src/guard_types.h), so analysis/evaluate.py can label real
// outages with one rule for both firmware builds. Codes 1, 2 and 7 (latch-up
// events) only occur in the integrated build (Block G).
enum class RecoveryEvent : uint8_t {
  NONE = 0,
  SENSOR_FAULT = 3,      // the sensor stopped working: an outage starts
  LADDER_STEP = 4,       // one XSHUT reset + re-initialisation was tried
  SENSOR_RECOVERED = 5,  // good readings again: the outage is over
  SENSOR_FAILED = 6,     // max_attempts resets in a row failed (we keep trying)
};

enum class HangReason : uint8_t { NONE = 0, TIMEOUT = 1, INVALID = 2, STUCK = 3, BUS_ERROR = 4 };

inline const char* hangReasonName(HangReason r) {
  switch (r) {
    case HangReason::NONE: return "none";
    case HangReason::TIMEOUT: return "timeout";
    case HangReason::INVALID: return "invalid";
    case HangReason::STUCK: return "stuck";
    case HangReason::BUS_ERROR: return "bus";
  }
  return "?";
}

struct SensorGuardConfig {
  // No reading for this long = TIMEOUT. A reading normally arrives every 33 ms,
  // so 200 ms is six missed readings.
  // TODO(hardware): check in Milestone 2 that normal operation never gets near this.
  uint32_t timeout_us = 200000;
  // This many identical readings in a row = STUCK (30 = about 1 s, the same as
  // the Spec 2 guard). With a moving pendulum the distance changes every
  // reading. With a STILL card (Block A) the noise of about 1-2 mm still
  // changes most readings, but runs of equal values are normal, so this must
  // be large enough never to fire on a still card.
  // TODO(hardware): check with the Block A recordings (noise_summary.csv,
  // column longest_equal_run, at each distance) and raise it if needed.
  uint16_t stuck_count = 30;
  // The value a hung VL53L0X returns.
  uint16_t hung_mm = 65535;
  // Readings at or above this are "out of range" (not used, but not a hang).
  uint16_t out_of_range_mm = 8190;
  // I2C failures in a row before we call it a fault (one can be a glitch).
  uint8_t bus_error_count = 3;
  // Wait this long between failed reset attempts.
  uint32_t retry_us = 500000;
  // After this many failed attempts in a row, log SENSOR_FAILED once (we keep trying).
  uint8_t max_attempts = 5;
};

struct GuardStep {
  bool reading_ok = false;        // this reading can be used
  bool request_recovery = false;  // main.cpp should reset the sensor now
  RecoveryEvent event = RecoveryEvent::NONE;
  HangReason reason = HangReason::NONE;  // set with SENSOR_FAULT
  uint32_t outage_us = 0;                // set with SENSOR_RECOVERED
};

class SensorGuardLogic {
 public:
  enum class State : uint8_t { OK = 0, OUTAGE = 1, VERIFYING = 2 };

  void setConfig(const SensorGuardConfig& c) { cfg_ = c; }
  const SensorGuardConfig& config() const { return cfg_; }

  void reset(uint64_t t_us) {
    state_ = State::OK;
    last_good_us_ = t_us;
    have_last_ = false;
    same_ = 0;
    bus_errors_ = 0;
    attempts_ = 0;
    failed_logged_ = false;
  }

  State state() const { return state_; }
  bool inOutage() const { return state_ != State::OK; }
  uint32_t recoveries() const { return recoveries_; }

  // Call after every read attempt. mm is only looked at when r == READY.
  GuardStep onPoll(uint64_t t_us, ReadResult r, uint16_t mm) {
    GuardStep s;
    if (state_ == State::OUTAGE) {
      // Waiting to (re)try a reset. Readings are not trusted meanwhile.
      if (t_us >= next_retry_us_) s.request_recovery = true;
      return s;
    }

    if (r == ReadResult::BUS_ERROR) {
      if (++bus_errors_ >= cfg_.bus_error_count) return fault(t_us, HangReason::BUS_ERROR);
      return s;
    }
    bus_errors_ = 0;

    if (r == ReadResult::NOT_READY) {
      if (t_us - last_good_us_ > cfg_.timeout_us) return fault(t_us, HangReason::TIMEOUT);
      return s;
    }

    // r == READY
    if (mm == cfg_.hung_mm) return fault(t_us, HangReason::INVALID);
    if (have_last_ && mm == last_mm_) {
      same_++;
    } else {
      same_ = 1;  // this reading is the first of a (possible) run
    }
    last_mm_ = mm;
    have_last_ = true;
    if (same_ >= cfg_.stuck_count) return fault(t_us, HangReason::STUCK);
    if (mm >= cfg_.out_of_range_mm) {
      // Nothing in range. Not a hang, but the value is useless. It also
      // counts as "no reading" for the timeout.
      if (t_us - last_good_us_ > cfg_.timeout_us) return fault(t_us, HangReason::TIMEOUT);
      return s;
    }

    last_good_us_ = t_us;
    s.reading_ok = true;
    if (state_ == State::VERIFYING) {
      // First good reading after a successful reset: the outage is over.
      state_ = State::OK;
      attempts_ = 0;
      failed_logged_ = false;
      recoveries_++;
      s.event = RecoveryEvent::SENSOR_RECOVERED;
      s.outage_us = (uint32_t)(t_us - outage_start_us_);
    }
    return s;
  }

  // main.cpp calls this after it tried a reset. ok = the sensor initialised.
  GuardStep onRecoveryAttempt(uint64_t t_us, bool ok) {
    GuardStep s;
    s.event = RecoveryEvent::LADDER_STEP;
    if (ok) {
      // Re-initialised. Wait for a good reading before trusting it again.
      state_ = State::VERIFYING;
      last_good_us_ = t_us;  // restart the timeout clock
      have_last_ = false;
      same_ = 0;
      bus_errors_ = 0;
    } else {
      state_ = State::OUTAGE;
      next_retry_us_ = t_us + cfg_.retry_us;
      if (++attempts_ >= cfg_.max_attempts && !failed_logged_) {
        failed_logged_ = true;
        s.event = RecoveryEvent::SENSOR_FAILED;
      }
    }
    return s;
  }

 private:
  GuardStep fault(uint64_t t_us, HangReason why) {
    GuardStep s;
    if (state_ == State::OK) outage_start_us_ = last_good_us_;  // outage began after the last good reading
    state_ = State::OUTAGE;
    next_retry_us_ = t_us;  // try a reset straight away
    s.event = RecoveryEvent::SENSOR_FAULT;
    s.reason = why;
    s.request_recovery = true;
    return s;
  }

  SensorGuardConfig cfg_;
  State state_ = State::OK;
  uint64_t last_good_us_ = 0;
  uint64_t next_retry_us_ = 0;
  uint64_t outage_start_us_ = 0;
  uint16_t last_mm_ = 0;
  bool have_last_ = false;
  uint16_t same_ = 0;
  uint8_t bus_errors_ = 0;
  uint8_t attempts_ = 0;
  bool failed_logged_ = false;
  uint32_t recoveries_ = 0;
};
