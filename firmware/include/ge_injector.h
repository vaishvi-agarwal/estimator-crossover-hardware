// ge_injector.h — proximity-coupled Gilbert–Elliott burst-fault injector
// (SPEC.md section 4.1.2), with the published paper's parameters.
// Plain C++ (no Arduino). analysis/injector.py is a line-by-line copy in
// Python that produces the same numbers (tests/test_injector.py checks this).
//
// The idea in plain words: the sensor is either GOOD or BAD. At each new
// reading it may switch:
//     GOOD -> BAD   with the "entry" probability   (a burst of faults starts)
//     BAD  -> BAD   with the "persistence" probability (the burst goes on)
// so bursts come in clumps, like the single-event effects that motivated the paper.
//
// Proximity coupling (as in the paper): a proximity factor
//     rho(r) = r_ref^2 / (r^2 + r_ref^2)          (1 at the source, -> 0 far away)
// where r = |d - d_src| is the bob's distance from a virtual "source" placed
// on the sensor axis at d_src. Both probabilities interpolate LINEARLY in rho
// between a far value (rho = 0) and a near value (rho = 1):
//     entry:       0.002 -> 0.12      persistence: 0.5 -> 0.9
// These are the paper's values PER STEP AT 10 Hz. The hardware runs at about
// 30 Hz, so each probability is converted to keep the same behaviour in
// SECONDS (docs/methods.md, "Converting 10 Hz values"):
//     persist_step = persist_10Hz ^ (T / 0.1)
//     entry_step   = 1 - (1 - entry_10Hz) ^ (T / 0.1)
// with T = the sample period in seconds.
//
// While BAD, the reading handed to the estimators is corrupted. The paper's
// five conditions, plus the blend used for the crossover sweep
// (R = sigma_nom^2 is the nominal reading variance; the real reading already
// carries R, so "total variance kappa * R" means (kappa - 1) * R is ADDED):
//     VARIANCE  zero-mean noise, total variance kappa * R             (primary)
//     FROZEN    the last GOOD reading, repeated                        (primary)
//     BIAS      nominal variance + b_k, b_k = c * rho(r_k) * (+1),
//               c = 4 * sigma_nom: an offset away from the sensor that grows
//               as the bob nears the virtual source          (bias-dominated)
//     RANDBIAS  total variance kappa * R + b_j, b_j ~ N(0, sigma_b^2) drawn
//               once when the burst starts and held, sigma_b = 10 * sigma_nom
//     SYSBIAS   total variance kappa * R + b_k = 4 * sigma_nom * rho(r_k)
//     BLEND     lambda in [0, 1]: added variance (1 - lambda)(kappa - 1) R and
//               offset lambda * b_k (b_k as in BIAS). lambda = 0 is VARIANCE,
//               lambda = 1 is BIAS. The endpoints match the paper; the linear
//               path between them is assumed (docs/methods.md).
//
// The injector works on a COPY: the clean reading is logged unchanged.
#pragma once
#include <math.h>
#include <stdint.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// A small, fast random number generator (SplitMix64). Same seed = same
// sequence, on the ESP32, on a PC, and in Python.
struct SplitMix64 {
  uint64_t state = 0;
  void seed(uint64_t s) { state = s; }
  uint64_t next() {
    state += 0x9E3779B97F4A7C15ULL;
    uint64_t z = state;
    z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL;
    z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL;
    return z ^ (z >> 31);
  }
  // Uniform number in [0, 1): the top 53 bits, scaled. Exact in a double.
  double uniform() { return (double)(next() >> 11) * (1.0 / 9007199254740992.0); }
  // Standard Gaussian (mean 0, sd 1) by the Box-Muller method.
  // Always uses exactly two uniforms, so the sequence is easy to reproduce.
  double gaussian() {
    const double u1 = 1.0 - uniform();  // in (0, 1]: log(u1) is finite
    const double u2 = uniform();
    return sqrt(-2.0 * log(u1)) * cos(2.0 * M_PI * u2);
  }
};

enum class CorruptMode : uint8_t { VARIANCE = 0, FROZEN = 1, BIAS = 2, BLEND = 3, RANDBIAS = 4, SYSBIAS = 5 };

inline const char* corruptModeName(CorruptMode m) {
  switch (m) {
    case CorruptMode::VARIANCE: return "variance";
    case CorruptMode::FROZEN: return "frozen";
    case CorruptMode::BIAS: return "bias";
    case CorruptMode::BLEND: return "blend";
    case CorruptMode::RANDBIAS: return "randbias";
    case CorruptMode::SYSBIAS: return "sysbias";
  }
  return "?";
}

// The paper's step: probabilities below are "per 0.1 s".
static const double PAPER_STEP_S = 0.1;

struct GeConfig {
  // --- Fault chain, the paper's values at 10 Hz ---
  double entry_far = 0.002;   // P(GOOD -> BAD) per 0.1 s at rho = 0
  double entry_near = 0.12;   // ... at rho = 1
  double persist_far = 0.5;   // P(BAD -> BAD) per 0.1 s at rho = 0
  double persist_near = 0.9;  // ... at rho = 1
  // --- Proximity geometry (mm) ---
  // TODO(hardware): calibrate r_ref and d_src on Block B recordings so that
  // rho at the bob's closest approach is about 0.65-0.99 and a typical 60 s
  // session has enough BAD samples (analysis/calibrate_proximity.py).
  double r_ref_mm = 40.0;   // the paper's r_ref (8 m) scaled to the pendulum
  double d_src_mm = 220.0;  // virtual source position on the sensor axis
  // --- Sample period used to convert the 10 Hz probabilities ---
  // TODO(hardware): set to the measured mean sample period (mean of the
  // differences of t_us in Block B), about 1/30 s.
  double sample_period_s = 1.0 / 30.0;
  // --- Corruption ---
  // TODO(hardware): nominal reading noise sigma (mm) from Block A
  // (results/tables/noise_model.json at the rest distance).
  double sigma_nom_mm = 2.0;
  double kappa = 100.0;          // BAD variance = kappa * R (the paper's value)
  double bias_c_sigmas = 4.0;    // proximity bias scale c, in units of sigma_nom (paper: c = 4 sigma)
  double randbias_sigmas = 10.0; // per-burst bias sd sigma_b, in units of sigma_nom (paper: 10 sigma)
  double lambda = 0.5;           // BLEND position: 0 = variance, 1 = bias
  CorruptMode mode = CorruptMode::VARIANCE;
};

struct GeOutput {
  double faulted_mm;  // what the estimators receive
  bool bad;           // GE state for this reading
};

class GeInjector {
 public:
  void setConfig(const GeConfig& c) { cfg_ = c; }
  const GeConfig& config() const { return cfg_; }
  GeConfig& config() { return cfg_; }

  // Start again: state GOOD, random sequence from the beginning of `seed`.
  void reset(uint64_t seed) {
    seed_ = seed;
    rng_.seed(seed);
    bad_ = false;
    have_last_good_ = false;
    last_good_mm_ = 0.0;
    burst_bias_mm_ = 0.0;
  }
  uint64_t seed() const { return seed_; }
  bool bad() const { return bad_; }

  // Proximity factor at distance d (mm).
  double rho(double d_mm) const {
    const double r = fabs(d_mm - cfg_.d_src_mm);
    const double rr = cfg_.r_ref_mm * cfg_.r_ref_mm;
    return rr / (r * r + rr);
  }
  // Per-step (at sample_period_s) probabilities at distance d.
  double pEntry(double d_mm) const {
    const double p10 = cfg_.entry_far + (cfg_.entry_near - cfg_.entry_far) * rho(d_mm);
    return 1.0 - pow(1.0 - p10, cfg_.sample_period_s / PAPER_STEP_S);
  }
  double pPersist(double d_mm) const {
    const double s10 = cfg_.persist_far + (cfg_.persist_near - cfg_.persist_far) * rho(d_mm);
    return pow(s10, cfg_.sample_period_s / PAPER_STEP_S);
  }

  // Process one clean reading (mm). Call once per new reading, in order.
  GeOutput apply(double clean_mm) {
    // 1. Move between GOOD and BAD (one uniform number per reading).
    const double u = rng_.uniform();
    const bool wasBad = bad_;
    if (!bad_) {
      if (u < pEntry(clean_mm)) bad_ = true;
    } else {
      if (u < 1.0 - pPersist(clean_mm)) bad_ = false;  // exit = 1 - persistence
    }

    // 2. GOOD: pass the reading through and remember it.
    if (!bad_) {
      last_good_mm_ = clean_mm;
      have_last_good_ = true;
      return {quantise(clean_mm), false};
    }

    // 3. BAD: corrupt it. Two Gaussian numbers are drawn for EVERY BAD reading
    //    in every mode (n for noise, nb for a new burst's random bias), so the
    //    GOOD/BAD sequence for a given seed is the same whatever the mode:
    //    sessions with different modes can be compared burst for burst.
    const double n = rng_.gaussian();
    const double nb = rng_.gaussian();
    if (!wasBad) burst_bias_mm_ = cfg_.randbias_sigmas * cfg_.sigma_nom_mm * nb;  // RANDBIAS: drawn at burst start, held
    const double addSd = sqrt(cfg_.kappa - 1.0) * cfg_.sigma_nom_mm;  // extra sd for total variance kappa*R
    const double proxBias = cfg_.bias_c_sigmas * cfg_.sigma_nom_mm * rho(clean_mm);  // b_k = c rho(r_k) (+1)
    double out;
    switch (cfg_.mode) {
      case CorruptMode::VARIANCE: out = clean_mm + addSd * n; break;
      case CorruptMode::FROZEN: out = have_last_good_ ? last_good_mm_ : clean_mm; break;
      case CorruptMode::BIAS: out = clean_mm + proxBias; break;
      case CorruptMode::RANDBIAS: out = clean_mm + burst_bias_mm_ + addSd * n; break;
      case CorruptMode::SYSBIAS: out = clean_mm + proxBias + addSd * n; break;
      default:  // BLEND (assumed linear between the paper's two endpoints)
        out = clean_mm + sqrt(1.0 - cfg_.lambda) * addSd * n + cfg_.lambda * proxBias;
        break;
    }
    return {quantise(out), true};
  }

  // Round to 0.001 mm. The CSV prints 3 decimals, so after this rounding the
  // logged number is EXACTLY the number the estimators used. That lets the
  // Python estimators replay a session and get identical results.
  static double quantise(double mm) { return floor(mm * 1000.0 + 0.5) / 1000.0; }

 private:
  GeConfig cfg_;
  SplitMix64 rng_;
  uint64_t seed_ = 0;
  bool bad_ = false;
  bool have_last_good_ = false;
  double last_good_mm_ = 0.0;
  double burst_bias_mm_ = 0.0;  // RANDBIAS: the current burst's offset
};
