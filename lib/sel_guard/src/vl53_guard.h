// vl53_guard.h — the VL53L0X lock-up watchdog (plain C++, no Arduino).
//
// Two parts:
//   HangDetector      looks at each reading and decides "is the sensor hung?"
//   SensorSupervisor  runs the sensor: starts it after power-on, polls it,
//                     notices faults, runs the SEFI ladder, and reports
//                     outages as GuardEvents.
//
// The supervisor talks to the hardware only through the small SensorPort
// interface below. On the ESP32 that is the real VL53L0X (see
// sel_sefi_guard.h); in tests/test_host.cpp it is a fake sensor whose
// failures we script. That is how the ladder is tested without hardware.
#pragma once
#include <stdint.h>

#include "guard_types.h"
#include "sefi_ladder.h"

// ---------------------------------------------------------------------------
// HangDetector
// ---------------------------------------------------------------------------
struct HangConfig {
  // No new reading for this long = TIMEOUT. Normal period is ~33 ms.
  // TODO(hardware): check in the soak test that normal running never comes close.
  uint32_t timeout_us = 200000;
  // This many IDENTICAL readings in a row = STUCK. Real readings jitter by a
  // few mm, so 30 identical ones (about 1 s) means the sensor is frozen.
  // TODO(hardware): check the soak log for false STUCK detections on a still target.
  uint16_t stuck_count = 30;
  // Readings at or above this (8190/8191) mean "no target in range". That is
  // normal, not a fault, and those values are excluded from the STUCK check.
  uint16_t out_of_range_min = 8190;
};

class HangDetector {
 public:
  static const uint16_t HUNG_VALUE = 65535;

  explicit HangDetector(const HangConfig& cfg = HangConfig()) : cfg_(cfg) {}
  void setConfig(const HangConfig& cfg) { cfg_ = cfg; }
  const HangConfig& config() const { return cfg_; }

  // Call after (re)starting the sensor: the timeout clock starts now.
  void restart(uint64_t now) {
    last_reading_us_ = now;
    same_ = 0;
    have_last_ = false;
  }

  // A new reading arrived.
  SensorFault onReading(uint64_t now, uint16_t mm) {
    last_reading_us_ = now;
    if (mm == HUNG_VALUE) return SensorFault::INVALID;
    if (mm >= cfg_.out_of_range_min) {  // "no target": fine, but breaks a run of identical values
      have_last_ = false;
      same_ = 0;
      return SensorFault::NONE;
    }
    if (have_last_ && mm == last_mm_) {
      same_++;
      if (same_ + 1 >= cfg_.stuck_count) return SensorFault::STUCK;
    } else {
      same_ = 0;
    }
    last_mm_ = mm;
    have_last_ = true;
    return SensorFault::NONE;
  }

  // No reading was ready this time.
  SensorFault onNoReading(uint64_t now) const {
    return (now - last_reading_us_ >= cfg_.timeout_us) ? SensorFault::TIMEOUT : SensorFault::NONE;
  }

 private:
  HangConfig cfg_;
  uint64_t last_reading_us_ = 0;
  uint16_t last_mm_ = 0;
  uint16_t same_ = 0;
  bool have_last_ = false;
};

// ---------------------------------------------------------------------------
// SensorPort: what the supervisor needs from the sensor hardware.
// ---------------------------------------------------------------------------
class SensorPort {
 public:
  virtual ~SensorPort() {}
  virtual bool start() = 0;       // after power-on: boot, init, start ranging
  virtual bool reinit() = 0;      // ladder step (a)
  virtual bool xshutReset() = 0;  // ladder step (b) (includes a fresh start)
  virtual PollResult poll(uint16_t& mm) = 0;
  virtual void releaseBus() = 0;  // make the I2C pins safe before power-off
  virtual uint64_t nowUs() = 0;   // clock (the actions above take time)
};

// ---------------------------------------------------------------------------
// SensorSupervisor
// ---------------------------------------------------------------------------
struct SupervisorConfig {
  HangConfig hang;
  LadderConfig ladder;
  // After a recovery step, this many good readings in a row prove it worked...
  uint8_t verify_reads = 3;
  // ...and they must arrive within this time, or the step failed.
  // TODO(hardware): check in Block D that a working sensor always passes in time
  // (a too-short timeout would wrongly escalate to the next step).
  uint32_t verify_timeout_us = 500000;
};

enum class SupervisorState : uint8_t {
  POWERED_OFF = 0,  // rail is off (or still in blanking)
  RUNNING = 1,      // normal: readings flowing
  VERIFYING = 2,    // a ladder step was done; waiting for good readings
  WAIT_POWER = 3,   // the ladder asked for a power cycle; waiting for the rail
  FAILED = 4,       // the ladder gave up
};

class SensorSupervisor {
 public:
  static const int EVENT_QUEUE = 16;

  SensorSupervisor(SensorPort& port, const SupervisorConfig& cfg = SupervisorConfig())
      : port_(port), cfg_(cfg), detector_(cfg.hang), ladder_(cfg.ladder) {}

  // Change settings (only while the supervisor is not running a ladder).
  void setConfig(const SupervisorConfig& cfg) {
    cfg_ = cfg;
    detector_.setConfig(cfg.hang);
    ladder_.setConfig(cfg.ladder);
  }
  const SupervisorConfig& config() const { return cfg_; }

  SupervisorState state() const { return state_; }
  bool outageOpen() const { return outage_open_; }
  uint32_t goodReadings() const { return good_; }
  uint32_t badReadings() const { return bad_; }
  uint16_t lastRange() const { return last_mm_; }
  uint32_t readingSeq() const { return reading_seq_; }  // increases with every good reading

  // The ladder wants a power cycle. The caller (who owns the rail) must take
  // this request and switch the rail off and on again.
  bool takePowerCycleRequest() {
    const bool r = pc_request_;
    pc_request_ = false;
    return r;
  }

  // Leave FAILED and try again (a human decided to).
  void resetFailed() {
    ladder_.reset();
    if (state_ == SupervisorState::FAILED) state_ = SupervisorState::POWERED_OFF;
    seen_epoch_ = 0xFFFFFFFF;  // force a fresh start at the next update
  }

  // Events are queued; read them with popEvent() after update().
  bool popEvent(GuardEvent& ev) {
    if (q_count_ == 0) return false;
    ev = q_[q_head_];
    q_head_ = (q_head_ + 1) % EVENT_QUEUE;
    q_count_--;
    return true;
  }

  // Queue an event that happened outside the supervisor (e.g. an SEL trip),
  // so callers see one ordered stream.
  void pushEvent(GuardEvent ev) {
    ev.seq = ++seq_;
    if (q_count_ == EVENT_QUEUE) {  // full: drop the oldest
      q_head_ = (q_head_ + 1) % EVENT_QUEUE;
      q_count_--;
    }
    q_[(q_head_ + q_count_) % EVENT_QUEUE] = ev;
    q_count_++;
  }

  // Call often (every loop). `rail_ready`: the rail is on AND past blanking.
  // `rail_epoch`: a counter that goes up every time the rail is switched on.
  void update(bool rail_ready, uint32_t rail_epoch) {
    const uint64_t now = port_.nowUs();

    if (!rail_ready) {
      if (state_ == SupervisorState::RUNNING || state_ == SupervisorState::VERIFYING) {
        // Power went away while the sensor was in use: a latch-up trip.
        port_.releaseBus();
        openOutage(now, SensorFault::POWER_OFF);
        // If a ladder step was in progress, abandon it: the latch-up guard's
        // power cycle has overtaken it. The outage stays open and closes at
        // the first good reading after power returns (fixed_by = none).
        if (ladder_.active()) ladder_.reset();
        state_ = SupervisorState::POWERED_OFF;
      } else if (state_ == SupervisorState::WAIT_POWER && !bus_released_) {
        port_.releaseBus();
        bus_released_ = true;
      }
      return;
    }

    // Rail is on and past blanking. Was it just (re)powered?
    if (rail_epoch != seen_epoch_) {
      seen_epoch_ = rail_epoch;
      if (state_ == SupervisorState::FAILED) return;
      const bool wasWaiting = (state_ == SupervisorState::WAIT_POWER) || ladder_.active();
      const bool ok = port_.start();
      detector_.restart(port_.nowUs());
      // Outside an outage, a fresh start counts as the reference point, so a
      // fault right after power-up is measured from here (not from boot).
      if (!outage_open_) last_good_ = port_.nowUs();
      if (wasWaiting) {
        beginVerify(port_.nowUs());
        if (!ok) verifyFailed(port_.nowUs());
      } else if (ok) {
        state_ = SupervisorState::RUNNING;
      } else {
        state_ = SupervisorState::RUNNING;
        fault(port_.nowUs(), SensorFault::INIT_FAILED);
      }
      return;
    }

    if (state_ != SupervisorState::RUNNING && state_ != SupervisorState::VERIFYING) return;

    uint16_t mm = 0;
    const PollResult r = port_.poll(mm);
    const uint64_t t = port_.nowUs();
    if (r == PollResult::READY) {
      const SensorFault f = detector_.onReading(t, mm);
      if (f == SensorFault::NONE) goodReading(t, mm);
      else { bad_++; fault(t, f); }
    } else if (r == PollResult::BUS_ERROR) {
      fault(t, SensorFault::BUS_ERROR);
    } else if (state_ == SupervisorState::VERIFYING) {
      if (t - verify_start_ >= cfg_.verify_timeout_us) verifyFailed(t);
    } else {
      const SensorFault f = detector_.onNoReading(t);
      if (f != SensorFault::NONE) fault(t, f);
    }
  }

 private:
  void emit(EventType type, uint64_t t, SensorFault f = SensorFault::NONE,
            LadderStep step = LadderStep::NONE, bool ok = false, uint32_t dur = 0) {
    GuardEvent ev;
    ev.type = type;
    ev.t_us = t;
    ev.fault = f;
    ev.step = step;
    ev.ok = ok;
    ev.duration_us = dur;
    ev.outage_start_us = outage_start_;
    pushEvent(ev);
  }

  void openOutage(uint64_t t, SensorFault f) {
    if (outage_open_) return;
    outage_open_ = true;
    outage_start_ = last_good_;  // the outage began after the last good reading
    outage_fault_ = f;
    emit(EventType::SENSOR_FAULT, t, f);
  }

  void closeOutage(uint64_t t_first_good, LadderStep fixed_by) {
    if (!outage_open_) return;
    outage_open_ = false;
    emit(EventType::SENSOR_RECOVERED, t_first_good, outage_fault_, fixed_by, true,
         (uint32_t)(t_first_good - outage_start_));
  }

  void goodReading(uint64_t t, uint16_t mm) {
    good_++;
    last_mm_ = mm;
    reading_seq_++;
    if (state_ == SupervisorState::VERIFYING) {
      if (verify_good_ == 0) verify_first_good_ = t;
      if (++verify_good_ >= cfg_.verify_reads) {
        const LadderStep step = ladder_.currentStep();
        emit(EventType::LADDER_STEP, t, outage_fault_, step, true, (uint32_t)(t - step_start_));
        ladder_.onVerify(true, t);
        state_ = SupervisorState::RUNNING;
        last_good_ = t;
        closeOutage(verify_first_good_, step);
      }
      return;
    }
    last_good_ = t;
    if (outage_open_) closeOutage(t, LadderStep::NONE);  // e.g. back after a latch-up trip
  }

  void fault(uint64_t t, SensorFault f) {
    if (state_ == SupervisorState::VERIFYING) {
      verifyFailed(t);
      return;
    }
    openOutage(t, f);
    execute(ladder_.onFault(t), t);
  }

  void beginVerify(uint64_t t) {
    state_ = SupervisorState::VERIFYING;
    verify_start_ = t;
    verify_good_ = 0;
  }

  void verifyFailed(uint64_t t) {
    emit(EventType::LADDER_STEP, t, outage_fault_, ladder_.currentStep(), false, (uint32_t)(t - step_start_));
    execute(ladder_.onVerify(false, t), t);
  }

  // Carry out what the ladder decided. Each action is followed by verification.
  void execute(LadderAction a, uint64_t t) {
    step_start_ = t;
    switch (a) {
      case LadderAction::DO_REINIT:
      case LadderAction::DO_XSHUT_RESET: {
        const bool ok = (a == LadderAction::DO_REINIT) ? port_.reinit() : port_.xshutReset();
        detector_.restart(port_.nowUs());
        beginVerify(port_.nowUs());
        if (!ok) verifyFailed(port_.nowUs());  // init itself failed: escalate now
        break;
      }
      case LadderAction::DO_POWER_CYCLE:
        pc_request_ = true;
        bus_released_ = false;
        state_ = SupervisorState::WAIT_POWER;
        break;
      case LadderAction::REPORT_FAILED:
        state_ = SupervisorState::FAILED;
        emit(EventType::SENSOR_FAILED, t, outage_fault_, LadderStep::FAILED);
        break;
      default:
        break;
    }
  }

  SensorPort& port_;
  SupervisorConfig cfg_;
  HangDetector detector_;
  SefiLadder ladder_;
  SupervisorState state_ = SupervisorState::POWERED_OFF;
  uint32_t seen_epoch_ = 0xFFFFFFFF;
  bool pc_request_ = false;
  bool bus_released_ = false;

  bool outage_open_ = false;
  uint64_t outage_start_ = 0;
  SensorFault outage_fault_ = SensorFault::NONE;
  uint64_t last_good_ = 0;
  uint64_t step_start_ = 0;
  uint64_t verify_start_ = 0;
  uint64_t verify_first_good_ = 0;
  uint8_t verify_good_ = 0;

  uint32_t good_ = 0, bad_ = 0, reading_seq_ = 0;
  uint16_t last_mm_ = 0;

  GuardEvent q_[EVENT_QUEUE];
  int q_head_ = 0, q_count_ = 0;
  uint32_t seq_ = 0;
};
