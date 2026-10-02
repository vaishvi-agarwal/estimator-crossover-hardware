// imm1d.h — 2-mode Interacting Multiple Model (IMM) filter (SPEC.md 4.1.3).
// Plain C++ (no Arduino); Python copy in analysis/estimators.py.
//
// The idea in plain words: run two Kalman filters side by side.
//   mode 0 "nominal": readings have the normal noise r
//   mode 1 "bad":     readings have inflated noise r * r_bad_factor
// Each filter says how likely the latest reading was under its assumption.
// The IMM keeps a probability mu for each mode, updates it with those
// likelihoods, and blends the two filters' answers using mu. During a burst
// of noisy readings the "bad" mode explains them better, mu[1] rises, and the
// estimate stops chasing the noise.
//
// The modes switch like a Markov chain, the same shape as the Gilbert-Elliott
// fault model:
//   P(nominal -> bad) = p_nb,  P(bad -> nominal) = p_bn   (settable over serial)
//
// Each cycle: (1) mix the two filters' states according to how likely a mode
// switch was, (2) predict both, (3) update both with the reading, (4) update
// mu from the likelihoods, (5) combine into one estimate.
// Likelihoods are handled as logarithms, so a huge outlier cannot make both
// of them round to 0.
#pragma once
#include <math.h>

#include "kalman1d.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

struct ImmParams {
  double q = 1.0;           // as KfParams (shared)
  double r = 4e-6;          // nominal noise (m^2). TODO(hardware): from Block A, like KfParams::r.
  double p0_vel = 1.0;
  double r_bad_factor = 100.0;  // kappa_hat: bad-mode noise = r * this (the paper's value)
  // Mode switch probabilities per sample. Defaults: the paper's estimated
  // matrix [[0.99, 0.01], [0.33, 0.67]] at 10 Hz, converted to 30 Hz
  // (docs/methods.md). The analysis tunes them (IMM) or counts them from
  // training sessions (IMM-est); set them over serial with `imm`.
  double p_nb = 0.003344506587403595;  // P(nominal -> bad) = 1 - 0.99^(1/3)
  double p_bn = 0.12496598771667256;   // P(bad -> nominal) = 1 - 0.67^(1/3)
  double mu_bad0 = 0.1;     // starting probability of the bad mode
};

class ImmEstimator {
 public:
  ImmParams params;
  Kalman1D f[2];      // the two mode-matched filters
  double mu[2] = {1.0, 0.0};
  // Combined estimate
  double x0 = 0.0, x1 = 0.0, P00 = 0.0, P01 = 0.0, P11 = 0.0;
  bool initialised = false;

  void reset() {
    f[0].reset();
    f[1].reset();
    mu[0] = 1.0;
    mu[1] = 0.0;
    x0 = x1 = P00 = P01 = P11 = 0.0;
    initialised = false;
  }

  double rOf(int j) const { return j == 0 ? params.r : params.r * params.r_bad_factor; }

  // One update with the fixed switch probabilities in params.
  void step(double dt, bool has_z, double z) { stepWith(dt, has_z, z, params.p_nb, params.p_bn); }

  // One update with this step's own switch probabilities. IMM-oracle uses
  // this with the injector's TRUE probabilities, which change with distance.
  void stepWith(double dt, bool has_z, double z, double p_nb, double p_bn) {
    if (!initialised) {
      if (!has_z) return;
      f[0].init(z, params.r, params.p0_vel);
      f[1].init(z, params.r, params.p0_vel);
      mu[0] = 1.0 - params.mu_bad0;
      mu[1] = params.mu_bad0;
      combine();
      initialised = true;
      return;
    }

    // Transition matrix T[i][j] = P(mode j now | mode i before).
    const double T[2][2] = {{1.0 - p_nb, p_nb}, {p_bn, 1.0 - p_bn}};

    // (1) Mixing. c[j] = predicted probability of mode j.
    double c[2];
    c[0] = T[0][0] * mu[0] + T[1][0] * mu[1];
    c[1] = T[0][1] * mu[0] + T[1][1] * mu[1];
    Kalman1D mixed[2];
    for (int j = 0; j < 2; j++) {
      // w[i] = P(was in mode i | now in mode j)
      double w0, w1;
      if (c[j] > 0.0) {
        w0 = T[0][j] * mu[0] / c[j];
        w1 = T[1][j] * mu[1] / c[j];
      } else {
        w0 = (j == 0) ? 1.0 : 0.0;
        w1 = (j == 1) ? 1.0 : 0.0;
      }
      const double m0 = w0 * f[0].x0 + w1 * f[1].x0;
      const double m1 = w0 * f[0].x1 + w1 * f[1].x1;
      const double a0 = f[0].x0 - m0, a1 = f[0].x1 - m1;  // spread of filter 0 around the mix
      const double b0 = f[1].x0 - m0, b1 = f[1].x1 - m1;  // spread of filter 1 around the mix
      mixed[j].x0 = m0;
      mixed[j].x1 = m1;
      mixed[j].P00 = w0 * (f[0].P00 + a0 * a0) + w1 * (f[1].P00 + b0 * b0);
      mixed[j].P01 = w0 * (f[0].P01 + a0 * a1) + w1 * (f[1].P01 + b0 * b1);
      mixed[j].P11 = w0 * (f[0].P11 + a1 * a1) + w1 * (f[1].P11 + b1 * b1);
      mixed[j].initialised = true;
    }

    // (2) + (3) Predict and update each mode; keep the log-likelihoods.
    double ll[2] = {0.0, 0.0};
    for (int j = 0; j < 2; j++) {
      f[j] = mixed[j];
      if (dt > 0.0) f[j].predict(dt, params.q);
      if (has_z) {
        const double r = rOf(j);
        const double y = f[j].innovation(z);
        const double S = f[j].innovationVar(r);
        ll[j] = -0.5 * (y * y / S + log(2.0 * M_PI * S));
        f[j].update(z, r);
      }
    }

    // (4) Mode probabilities: mu[j] proportional to c[j] * likelihood[j].
    if (has_z) {
      const double mx = ll[0] > ll[1] ? ll[0] : ll[1];
      const double u0 = c[0] * exp(ll[0] - mx);
      const double u1 = c[1] * exp(ll[1] - mx);
      const double s = u0 + u1;
      if (s > 0.0) {
        mu[0] = u0 / s;
        mu[1] = u1 / s;
      } else {
        mu[0] = c[0];
        mu[1] = c[1];
      }
    } else {
      mu[0] = c[0];  // no reading: only the Markov chain moves the probabilities
      mu[1] = c[1];
    }

    // (5) One combined estimate.
    combine();
  }

  bool ready() const { return initialised; }
  double pos() const { return x0; }
  double vel() const { return x1; }
  double posVar() const { return P00; }
  double pBad() const { return mu[1]; }

 private:
  void combine() {
    x0 = mu[0] * f[0].x0 + mu[1] * f[1].x0;
    x1 = mu[0] * f[0].x1 + mu[1] * f[1].x1;
    const double a0 = f[0].x0 - x0, a1 = f[0].x1 - x1;
    const double b0 = f[1].x0 - x0, b1 = f[1].x1 - x1;
    P00 = mu[0] * (f[0].P00 + a0 * a0) + mu[1] * (f[1].P00 + b0 * b0);
    P01 = mu[0] * (f[0].P01 + a0 * a1) + mu[1] * (f[1].P01 + b0 * b1);
    P11 = mu[0] * (f[0].P11 + a1 * a1) + mu[1] * (f[1].P11 + b1 * b1);
  }
};
