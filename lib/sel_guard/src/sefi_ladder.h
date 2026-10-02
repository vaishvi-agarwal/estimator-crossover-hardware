// sefi_ladder.h — the lock-up (SEFI) recovery escalation ladder.
//
// A single-event functional interrupt leaves a chip stuck in a wrong mode
// until something resets it. We try the gentlest fix first and only escalate
// if it does not work:
//
//   fault ──> (a) REINIT ──fail──> (b) XSHUT ──fail──> (c) POWER_CYCLE ──fail──> ... (K times) ──> FAILED
//                 │ ok                 │ ok                   │ ok
//                 v                    v                      v
//              RECOVERED            RECOVERED              RECOVERED
//
// This class only DECIDES what to do next. It does not touch hardware: the
// caller performs the returned action, checks whether the sensor works again,
// and reports back with onVerify(). That keeps it testable on a PC.
#pragma once
#include <stdint.h>

#include "guard_types.h"

enum class LadderAction : uint8_t {
  NONE = 0,
  DO_REINIT = 1,         // perform step (a), then call onVerify()
  DO_XSHUT_RESET = 2,    // perform step (b), then call onVerify()
  DO_POWER_CYCLE = 3,    // perform step (c), then call onVerify()
  REPORT_RECOVERED = 4,  // done: the sensor works (see fixedBy())
  REPORT_FAILED = 5,     // done: give up, the sensor is FAILED
};

struct LadderConfig {
  // K in SPEC.md: how many full power cycles to try before declaring FAILED.
  // TODO(hardware): check in Block D whether a 2nd or 3rd power cycle ever helps.
  uint8_t max_power_cycles = 3;
};

class SefiLadder {
 public:
  explicit SefiLadder(const LadderConfig& cfg = LadderConfig()) : cfg_(cfg) {
    if (cfg_.max_power_cycles == 0) cfg_.max_power_cycles = 1;
  }

  void setConfig(const LadderConfig& cfg) {
    cfg_ = cfg;
    if (cfg_.max_power_cycles == 0) cfg_.max_power_cycles = 1;
  }

  bool active() const { return active_; }
  bool failed() const { return failed_; }
  LadderStep currentStep() const { return step_; }
  uint8_t powerCycleAttempt() const { return pc_attempt_; }
  uint8_t stepsTried() const { return steps_tried_; }
  LadderStep fixedBy() const { return fixed_by_; }
  uint64_t startedUs() const { return t_start_; }

  // The sensor just went bad. Returns the first action to take.
  LadderAction onFault(uint64_t now) {
    if (failed_) return LadderAction::NONE;
    if (active_) return onVerify(false, now);  // a new fault mid-ladder = this step failed
    active_ = true;
    t_start_ = now;
    steps_tried_ = 1;
    pc_attempt_ = 0;
    fixed_by_ = LadderStep::NONE;
    step_ = LadderStep::REINIT;
    return LadderAction::DO_REINIT;
  }

  // Report whether the sensor works after the last action.
  LadderAction onVerify(bool ok, uint64_t /*now*/) {
    if (!active_) return LadderAction::NONE;
    if (ok) {
      fixed_by_ = step_;
      active_ = false;
      step_ = LadderStep::NONE;
      return LadderAction::REPORT_RECOVERED;
    }
    // Escalate to the next step.
    switch (step_) {
      case LadderStep::REINIT:
        step_ = LadderStep::XSHUT;
        steps_tried_++;
        return LadderAction::DO_XSHUT_RESET;
      case LadderStep::XSHUT:
        step_ = LadderStep::POWER_CYCLE;
        pc_attempt_ = 1;
        steps_tried_++;
        return LadderAction::DO_POWER_CYCLE;
      case LadderStep::POWER_CYCLE:
        if (pc_attempt_ < cfg_.max_power_cycles) {
          pc_attempt_++;
          steps_tried_++;
          return LadderAction::DO_POWER_CYCLE;
        }
        break;
      default:
        break;
    }
    active_ = false;
    failed_ = true;
    fixed_by_ = LadderStep::FAILED;
    step_ = LadderStep::FAILED;
    return LadderAction::REPORT_FAILED;
  }

  // A human (or the `reset` command) clears FAILED so the ladder can run again.
  void reset() {
    active_ = false;
    failed_ = false;
    step_ = LadderStep::NONE;
    pc_attempt_ = 0;
    steps_tried_ = 0;
  }

 private:
  LadderConfig cfg_;
  bool active_ = false;
  bool failed_ = false;
  LadderStep step_ = LadderStep::NONE;
  LadderStep fixed_by_ = LadderStep::NONE;
  uint8_t pc_attempt_ = 0;
  uint8_t steps_tried_ = 0;
  uint64_t t_start_ = 0;
};
