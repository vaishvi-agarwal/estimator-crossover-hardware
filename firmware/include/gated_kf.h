// gated_kf.h — innovation-gated ("robust") Kalman filter (SPEC.md 4.1.3), with
// the paper's settings. Plain C++ (no Arduino); Python copy in analysis/estimators.py.
//
// Same as the KF, with one extra rule: before using a reading, check whether
// it is believable. The "innovation" y = reading - predicted position should
// be about as big as its predicted spread sqrt(S). The normalised squared
// innovation  d2 = y^2 / S  follows a chi-square distribution with 1 degree
// of freedom when the reading is healthy. If d2 is larger than the gate (a
// chi-square threshold for a chosen confidence: tuned from 95, 99 or 99.9 %),
// the reading is ignored and the filter just predicts ("coasts").
//
// Coasting timeout (default: the paper's 10 steps at 10 Hz = 1.0 s; tuned from
// 3, 5, 10, 20, 50 steps = 0.3-5 s): if the filter has not accepted a reading
// for coast_s seconds, the next reading is accepted whatever the gate says,
// with a normal update (as in the paper). Without this, a filter that has drifted away
// from the readings rejects every later reading and never comes back.
#pragma once
#include "kalman1d.h"

struct GatedParams {
  double q = 1.0;       // as KfParams (one shared value for all filters, chosen by tune.py)
  double r = 4e-6;      // as KfParams. TODO(hardware): from Block A, like KfParams::r.
  double p0_vel = 1.0;
  // Chi-square threshold, 1 DOF. 6.635 = 99 % confidence. The analysis tunes the
  // confidence on validation from {95, 99, 99.9 %} (3.841, 6.635, 10.83), as in the
  // paper, where 95 % was clearly worse; send the result with `gated`.
  double gate = 6.634896601021214;
  double coast_s = 1.0;  // accept the next reading after this long without an accepted one
};

class GatedKfEstimator {
 public:
  GatedParams params;
  Kalman1D kf;
  bool last_rejected = false;  // was the last reading thrown away?
  unsigned long rejected = 0;  // how many readings were thrown away in total
  unsigned long forced = 0;    // how many readings were accepted by the coasting timeout
  double coasting = 0.0;       // seconds since the last accepted reading

  void reset() {
    kf.reset();
    last_rejected = false;
    rejected = 0;
    forced = 0;
    coasting = 0.0;
  }

  void step(double dt, bool has_z, double z) {
    last_rejected = false;
    if (!kf.initialised) {
      if (has_z) {
        kf.init(z, params.r, params.p0_vel);
        coasting = 0.0;
      }
      return;
    }
    if (dt > 0.0) {
      kf.predict(dt, params.q);
      coasting = coasting + dt;
    }
    if (!has_z) return;
    const double y = kf.innovation(z);
    const double S = kf.innovationVar(params.r);
    const double d2 = y * y / S;
    if (d2 > params.gate) {
      // (1e-9 s tolerance: 30 steps of 1/30 s add up to 0.99999... s)
      if (coasting < params.coast_s - 1e-9) {
        last_rejected = true;
        rejected++;
        return;
      }
      forced++;  // coasted too long: take this reading anyway
    }
    kf.update(z, params.r);
    coasting = 0.0;
  }

  bool ready() const { return kf.initialised; }
  double pos() const { return kf.x0; }
  double vel() const { return kf.x1; }
  double posVar() const { return kf.P00; }
};
