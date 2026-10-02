// kalman1d.h — Kalman filter for a 1D "double integrator" (SPEC.md 4.1.3).
// Plain C++ (no Arduino). analysis/estimators.py has a line-by-line Python
// copy; tests/test_estimators.py checks both give the same numbers.
//
// The model, in plain words:
//   state x = [position (m), velocity (m/s)]
//   between readings, position changes by velocity * dt, and velocity is
//   nudged by random accelerations of strength q ("process noise");
//   each reading measures the position, with noise variance r (m^2).
//
// Written out for 2x2 matrices (no matrix library), with every operation in a
// fixed order: that is what makes the Python copy bit-for-bit identical.
// Do not "simplify" a formula here without changing estimators.py the same way.
#pragma once
#include <math.h>

struct KfParams {
  // Process noise strength (m^2/s^3). Bigger = trusts the motion model less.
  // As in the paper, ONE value is shared by every Kalman-based filter. It is
  // chosen on validation sessions by analysis/tune.py and then fixed; send it
  // to the firmware with the `q` command. 1.0 is only a starting point.
  double q = 1.0;
  // Measurement noise variance (m^2). (2 mm)^2 = 4e-6.
  // TODO(hardware): set from the Block A noise sigma (results/tables/noise_model.json).
  double r = 4e-6;
  // Starting velocity variance ((m/s)^2) when the first reading arrives.
  double p0_vel = 1.0;
};

class Kalman1D {
 public:
  // State and covariance (P is symmetric, so P10 = P01 is not stored).
  double x0 = 0.0, x1 = 0.0;
  double P00 = 0.0, P01 = 0.0, P11 = 0.0;
  bool initialised = false;

  void reset() {
    x0 = x1 = 0.0;
    P00 = P01 = P11 = 0.0;
    initialised = false;
  }

  // Start at the first reading: position = reading, velocity = 0.
  void init(double z, double r, double p0_vel) {
    x0 = z;
    x1 = 0.0;
    P00 = r;
    P01 = 0.0;
    P11 = p0_vel;
    initialised = true;
  }

  // Move the state forward by dt seconds.
  void predict(double dt, double q) {
    const double dt2 = dt * dt;
    const double dt3 = dt2 * dt;
    x0 = x0 + dt * x1;
    const double p00 = P00 + 2.0 * dt * P01 + dt2 * P11 + q * dt3 / 3.0;
    const double p01 = P01 + dt * P11 + q * dt2 / 2.0;
    const double p11 = P11 + q * dt;
    P00 = p00;
    P01 = p01;
    P11 = p11;
  }

  // Innovation (reading minus predicted position) and its variance.
  double innovation(double z) const { return z - x0; }
  double innovationVar(double r) const { return P00 + r; }

  // Correct the state with reading z (variance r).
  void update(double z, double r) {
    const double y = z - x0;
    const double S = P00 + r;
    const double k0 = P00 / S;
    const double k1 = P01 / S;
    x0 = x0 + k0 * y;
    x1 = x1 + k1 * y;
    const double p00 = (1.0 - k0) * P00;
    const double p01 = (1.0 - k0) * P01;
    const double p11 = P11 - k1 * P01;
    P00 = p00;
    P01 = p01;
    P11 = p11;
  }
};

// The standard KF estimator: predict every sample, update when there is a reading.
class KfEstimator {
 public:
  KfParams params;
  Kalman1D kf;

  void reset() { kf.reset(); }

  // dt = seconds since the previous sample; has_z = a reading is available.
  void step(double dt, bool has_z, double z) {
    if (!kf.initialised) {
      if (has_z) kf.init(z, params.r, params.p0_vel);
      return;
    }
    if (dt > 0.0) kf.predict(dt, params.q);
    if (has_z) kf.update(z, params.r);
  }

  bool ready() const { return kf.initialised; }
  double pos() const { return kf.x0; }
  double vel() const { return kf.x1; }
  double posVar() const { return kf.P00; }
};
