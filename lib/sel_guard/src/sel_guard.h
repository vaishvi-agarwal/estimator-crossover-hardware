// sel_guard.h — the latch-up (SEL) protection state machine.
//
// Plain C++ with no Arduino code: it never touches a pin. You feed it current
// samples with timestamps, and it tells you when to switch the rail on or off.
// That is what makes it testable on a PC (tests/test_host.cpp).
//
//            powerOn()
//   OFF ─────────────────> BLANKING ──(blank time over)──> MONITOR
//                             ^                               │
//                             │                 current > threshold for
//                    (off time over)            `debounce` samples in a row
//                             │                               v
//                             └─────────────────────────── TRIPPED  (rail OFF)
//
//   * BLANKING: right after power-on the capacitors charge and the current
//     spikes (inrush). Samples in this window are ignored so inrush does not
//     look like a latch-up.
//   * DEBOUNCE: one noisy sample above the threshold is not enough; it takes
//     `debounce` samples in a row. More debounce = fewer false trips, but a
//     slower reaction. Measuring that trade-off is research question RQ2.
//   * CYCLING: a power cycle asked for by the lock-up (SEFI) ladder. It works
//     like TRIPPED but is not counted as a latch-up.
//   * LOCKOUT: optional. If the device trips again and again (a real fault that
//     power-cycling does not clear), stop re-powering it and wait for a human.
//
// Times are 64-bit microseconds, so they never wrap around during a long test.
#pragma once
#include <stdint.h>

enum class SelState : uint8_t { OFF = 0, BLANKING = 1, MONITOR = 2, TRIPPED = 3, CYCLING = 4, LOCKOUT = 5 };

// What the caller must do with the rail after a call.
enum class RailCmd : uint8_t { NONE = 0, ON = 1, OFF = 2 };

struct SelConfig {
  float threshold_mA = 60.0f;      // trip when current is ABOVE this...
  uint16_t debounce = 4;           // ...for this many samples in a row (0 is treated as 1)
  uint32_t blank_us = 2000;        // ignore samples this long after power-on
  uint32_t off_us = 500000;        // keep power off this long after a trip
  uint8_t max_consecutive_trips = 0;  // 0 = never lock out (used by the experiments)
  uint32_t clean_reset_us = 5000000;  // this long in MONITOR without a trip resets the count
};

// Facts about the most recent trip, for logging.
struct SelTrip {
  uint64_t t_first_above_us = 0;  // first sample of the run that caused the trip
  uint64_t t_trip_us = 0;         // sample that completed the debounce count
  float peak_mA = 0.0f;           // highest current seen in that run
  uint16_t samples_above = 0;     // length of the run (= debounce)
};

class SelGuard {
 public:
  explicit SelGuard(const SelConfig& cfg = SelConfig()) : cfg_(cfg) {}

  void setConfig(const SelConfig& cfg) { cfg_ = cfg; }
  const SelConfig& config() const { return cfg_; }
  SelState state() const { return state_; }
  bool railShouldBeOn() const { return state_ == SelState::BLANKING || state_ == SelState::MONITOR; }

  uint32_t tripCount() const { return trip_count_; }
  uint8_t consecutiveTrips() const { return consecutive_; }
  const SelTrip& lastTrip() const { return last_trip_; }
  uint16_t aboveCount() const { return above_; }

  // Switch the rail on and start the blanking window.
  RailCmd powerOn(uint64_t now) {
    if (state_ == SelState::LOCKOUT) return RailCmd::NONE;
    state_ = SelState::BLANKING;
    on_since_ = now;
    above_ = 0;
    return RailCmd::ON;
  }

  // Switch the rail off on purpose (not a trip).
  RailCmd powerOff() {
    if (state_ == SelState::LOCKOUT) return RailCmd::NONE;
    state_ = SelState::OFF;
    above_ = 0;
    return RailCmd::OFF;
  }

  // Power cycle requested by the lock-up ladder: off now, on again after off_us.
  RailCmd requestPowerCycle(uint64_t now) {
    if (state_ == SelState::LOCKOUT) return RailCmd::NONE;
    state_ = SelState::CYCLING;
    off_since_ = now;
    above_ = 0;
    return RailCmd::OFF;
  }

  // Leave LOCKOUT (a human decided it is safe). The rail stays off until powerOn().
  void reset() {
    state_ = SelState::OFF;
    above_ = 0;
    consecutive_ = 0;
  }

  // Feed one current sample taken at time `now`. Call this for every sample,
  // also while the rail is off, so the off-timer can run.
  RailCmd update(uint64_t now, float current_mA) {
    switch (state_) {
      case SelState::OFF:
      case SelState::LOCKOUT:
        return RailCmd::NONE;

      case SelState::TRIPPED:
      case SelState::CYCLING:
        if (now - off_since_ >= cfg_.off_us) return powerOn(now);
        return RailCmd::NONE;

      case SelState::BLANKING:
        if (now - on_since_ < cfg_.blank_us) {
          above_ = 0;  // inrush: ignore, and do not let it count towards debounce
          return RailCmd::NONE;
        }
        state_ = SelState::MONITOR;
        monitor_since_ = now;
        // fall through: this sample is the first one we monitor
        [[fallthrough]];

      case SelState::MONITOR:
        return monitor(now, current_mA);
    }
    return RailCmd::NONE;
  }

 private:
  RailCmd monitor(uint64_t now, float mA) {
    // A long clean stretch means earlier trips were isolated events.
    if (consecutive_ > 0 && now - monitor_since_ >= cfg_.clean_reset_us) consecutive_ = 0;

    if (mA > cfg_.threshold_mA) {
      if (above_ == 0) {
        run_start_ = now;
        run_peak_ = mA;
      } else if (mA > run_peak_) {
        run_peak_ = mA;
      }
      if (above_ < 0xFFFF) above_++;
      const uint16_t needed = cfg_.debounce == 0 ? 1 : cfg_.debounce;
      if (above_ >= needed) return trip(now);
    } else {
      above_ = 0;  // the run is broken: start counting again
    }
    return RailCmd::NONE;
  }

  RailCmd trip(uint64_t now) {
    last_trip_.t_first_above_us = run_start_;
    last_trip_.t_trip_us = now;
    last_trip_.peak_mA = run_peak_;
    last_trip_.samples_above = above_;
    trip_count_++;
    if (consecutive_ < 0xFF) consecutive_++;
    above_ = 0;
    off_since_ = now;
    if (cfg_.max_consecutive_trips > 0 && consecutive_ >= cfg_.max_consecutive_trips) {
      state_ = SelState::LOCKOUT;
    } else {
      state_ = SelState::TRIPPED;
    }
    return RailCmd::OFF;
  }

  SelConfig cfg_;
  SelState state_ = SelState::OFF;
  uint64_t on_since_ = 0;
  uint64_t off_since_ = 0;
  uint64_t monitor_since_ = 0;
  uint64_t run_start_ = 0;
  float run_peak_ = 0.0f;
  uint16_t above_ = 0;
  uint32_t trip_count_ = 0;
  uint8_t consecutive_ = 0;
  SelTrip last_trip_;
};
