// test_host.cpp — unit tests for the firmware logic, run on a PC (no ESP32).
//
//   make test-host          build and run every test
//   ./build/test_host --reference tests/reference_io/reference.csv
//                           also write the C++/Python equivalence reference file
//
// Only the Arduino-free headers in firmware/include/ are tested here.
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <string>
#include <vector>

#include "gated_kf.h"
#include "ge_injector.h"
#include "imm1d.h"
#include "kalman1d.h"
#include "sensor_guard.h"
#include "serial_commands.h"

// ---- A tiny test framework: CHECK(condition) records a failure and goes on. ----
static int g_failures = 0;
static int g_checks = 0;
#define CHECK(cond)                                                       \
  do {                                                                    \
    g_checks++;                                                           \
    if (!(cond)) {                                                        \
      g_failures++;                                                       \
      std::printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);      \
    }                                                                     \
  } while (0)

struct TestCase {
  const char* name;
  std::function<void()> fn;
};
static std::vector<TestCase>& tests() {
  static std::vector<TestCase> t;
  return t;
}
struct Register {
  Register(const char* n, std::function<void()> f) { tests().push_back({n, f}); }
};
#define TEST(name)                                 \
  static void name();                              \
  static Register reg_##name(#name, name);         \
  static void name()

// ===========================================================================
// Sensor guard (sensor_guard.h)
// ===========================================================================
TEST(guard_accepts_normal_readings) {
  SensorGuardLogic g;
  g.reset(0);
  for (int i = 1; i <= 100; i++) {
    GuardStep s = g.onPoll(i * 33000ULL, ReadResult::READY, (uint16_t)(250 + (i % 5)));
    CHECK(s.reading_ok);
    CHECK(s.event == RecoveryEvent::NONE);
  }
}

TEST(guard_detects_timeout_and_recovers) {
  SensorGuardLogic g;
  g.reset(0);
  CHECK(g.onPoll(33000, ReadResult::READY, 250).reading_ok);
  // No reading for 150 ms: still fine (timeout is 200 ms).
  CHECK(g.onPoll(183000, ReadResult::NOT_READY, 0).event == RecoveryEvent::NONE);
  // 250 ms without a reading: timeout fault, reset requested at once.
  GuardStep s = g.onPoll(283000, ReadResult::NOT_READY, 0);
  CHECK(s.event == RecoveryEvent::SENSOR_FAULT);
  CHECK(s.reason == HangReason::TIMEOUT);
  CHECK(s.request_recovery);
  // The reset works; the next good reading ends the outage.
  CHECK(g.onRecoveryAttempt(300000, true).event == RecoveryEvent::LADDER_STEP);
  s = g.onPoll(340000, ReadResult::READY, 251);
  CHECK(s.reading_ok);
  CHECK(s.event == RecoveryEvent::SENSOR_RECOVERED);
  CHECK(s.outage_us == 340000 - 33000);  // from the last good reading
  CHECK(g.recoveries() == 1);
}

TEST(guard_detects_65535_as_hang) {
  SensorGuardLogic g;
  g.reset(0);
  GuardStep s = g.onPoll(33000, ReadResult::READY, 65535);
  CHECK(!s.reading_ok);
  CHECK(s.event == RecoveryEvent::SENSOR_FAULT && s.reason == HangReason::INVALID);
}

TEST(guard_detects_stuck_readings) {
  SensorGuardLogic g;
  g.reset(0);
  SensorGuardConfig c;
  c.stuck_count = 5;
  g.setConfig(c);
  for (int i = 1; i <= 4; i++) CHECK(g.onPoll(i * 33000ULL, ReadResult::READY, 300).reading_ok);
  GuardStep s = g.onPoll(5 * 33000ULL, ReadResult::READY, 300);
  CHECK(s.event == RecoveryEvent::SENSOR_FAULT && s.reason == HangReason::STUCK);
}

TEST(guard_out_of_range_is_not_a_hang) {
  SensorGuardLogic g;
  g.reset(0);
  GuardStep s = g.onPoll(33000, ReadResult::READY, 8190);
  CHECK(!s.reading_ok);
  CHECK(s.event == RecoveryEvent::NONE);
}

TEST(guard_bus_errors_need_several_in_a_row) {
  SensorGuardLogic g;
  g.reset(0);
  CHECK(g.onPoll(1000, ReadResult::BUS_ERROR, 0).event == RecoveryEvent::NONE);
  CHECK(g.onPoll(2000, ReadResult::READY, 250).reading_ok);  // resets the count
  CHECK(g.onPoll(3000, ReadResult::BUS_ERROR, 0).event == RecoveryEvent::NONE);
  CHECK(g.onPoll(4000, ReadResult::BUS_ERROR, 0).event == RecoveryEvent::NONE);
  GuardStep s = g.onPoll(5000, ReadResult::BUS_ERROR, 0);
  CHECK(s.event == RecoveryEvent::SENSOR_FAULT && s.reason == HangReason::BUS_ERROR);
}

TEST(guard_retries_and_reports_failure_once) {
  SensorGuardLogic g;
  g.reset(0);
  GuardStep s = g.onPoll(300000, ReadResult::NOT_READY, 0);
  CHECK(s.request_recovery);
  uint64_t t = 300000;
  int failedEvents = 0;
  for (int attempt = 0; attempt < 8; attempt++) {
    if (g.onRecoveryAttempt(t, false).event == RecoveryEvent::SENSOR_FAILED) failedEvents++;
    // Before retry_us has passed, no new attempt is requested.
    CHECK(!g.onPoll(t + 1000, ReadResult::NOT_READY, 0).request_recovery);
    t += 500000;
    CHECK(g.onPoll(t, ReadResult::NOT_READY, 0).request_recovery);
  }
  CHECK(failedEvents == 1);
}

// ===========================================================================
// Gilbert–Elliott injector (ge_injector.h)
// ===========================================================================
// Constant probabilities: entry_far = entry_near and persist_far = persist_near.
static GeConfig constantChain(double entry10, double persist10) {
  GeConfig c;
  c.entry_far = c.entry_near = entry10;
  c.persist_far = c.persist_near = persist10;
  return c;
}

TEST(ge_state_frequencies_match_theory) {
  // The paper's 10 Hz values converted to 30 Hz must give, per step,
  //   entry = 1 - (1 - 0.06)^(1/3),  exit = 1 - 0.7^(1/3)
  // so the long-run BAD fraction is entry / (entry + exit) and bursts last
  // 1 / exit readings on average: the same behaviour in SECONDS as at 10 Hz.
  GeInjector inj;
  inj.setConfig(constantChain(0.06, 0.7));
  inj.reset(12345);
  const double entry = 1.0 - std::pow(0.94, 1.0 / 3.0), exitP = 1.0 - std::pow(0.7, 1.0 / 3.0);
  CHECK(std::fabs(inj.pEntry(300) - entry) < 1e-15);
  CHECK(std::fabs(1.0 - inj.pPersist(300) - exitP) < 1e-15);
  const int N = 600000;
  int nBad = 0, bursts = 0;
  bool prev = false;
  for (int i = 0; i < N; i++) {
    const bool b = inj.apply(300.0).bad;
    nBad += b;
    if (b && !prev) bursts++;
    prev = b;
  }
  const double frac = (double)nBad / N, theory = entry / (entry + exitP);
  const double meanBurst = (double)nBad / bursts;
  std::printf("    BAD fraction %.4f (theory %.4f), mean burst %.2f (theory %.2f)\n", frac, theory, meanBurst,
              1.0 / exitP);
  CHECK(std::fabs(frac - theory) < 0.01);
  CHECK(std::fabs(meanBurst - 1.0 / exitP) < 0.3);
  // In seconds, a burst lasts as long as at 10 Hz: 1/(1-0.7) steps of 0.1 s ~ 0.33 s (geometric
  // in continuous time this is -0.1/ln(0.7) = 0.28 s; both rates give the same survival curve).
  CHECK(std::fabs(std::pow(inj.pPersist(300), 30) - std::pow(0.7, 10)) < 1e-12);  // survive 1 s
}

TEST(ge_rate_conversion_keeps_seconds) {
  // Halving the sample period must keep the probability of staying BAD for 1 s.
  GeInjector a, b;
  GeConfig c = constantChain(0.12, 0.9);
  c.sample_period_s = 0.1;
  a.setConfig(c);
  c.sample_period_s = 0.05;
  b.setConfig(c);
  CHECK(std::fabs(a.pPersist(0) - 0.9) < 1e-15);         // at 10 Hz the value is unchanged
  CHECK(std::fabs(a.pEntry(0) - 0.12) < 1e-15);
  CHECK(std::fabs(std::pow(b.pPersist(0), 20) - std::pow(0.9, 10)) < 1e-12);
  CHECK(std::fabs(std::pow(1 - b.pEntry(0), 20) - std::pow(0.88, 10)) < 1e-12);
}

TEST(ge_proximity_coupling) {
  GeInjector inj;
  GeConfig c;  // the paper's chain: entry 0.002 -> 0.12, persistence 0.5 -> 0.9
  c.d_src_mm = 220;
  c.r_ref_mm = 40;
  inj.setConfig(c);
  CHECK(std::fabs(inj.rho(220) - 1.0) < 1e-15);            // at the source
  CHECK(std::fabs(inj.rho(260) - 0.5) < 1e-15);            // r = r_ref
  CHECK(std::fabs(inj.rho(180) - 0.5) < 1e-15);            // symmetric
  CHECK(inj.rho(1000) < 0.01);
  // Linear interpolation in rho, at 10 Hz, then converted:
  const double p10 = 0.002 + (0.12 - 0.002) * 0.5;
  CHECK(std::fabs(inj.pEntry(260) - (1.0 - std::pow(1.0 - p10, 1.0 / 3.0))) < 1e-15);
  const double s10 = 0.5 + (0.9 - 0.5) * 0.5;
  CHECK(std::fabs(inj.pPersist(260) - std::pow(s10, 1.0 / 3.0)) < 1e-15);
  // Empirically, bursts start more often near the source.
  inj.reset(7);
  int startsNear = 0, startsFar = 0;
  for (int i = 0; i < 400000; i++) {
    const double d = (i % 2 == 0) ? 220.0 : 400.0;
    const bool wasBad = inj.bad();
    const bool b = inj.apply(d).bad;
    if (b && !wasBad) (d < 300 ? startsNear : startsFar)++;
  }
  CHECK(startsNear > 4 * startsFar);
}

TEST(ge_frozen_repeats_last_good_value) {
  GeInjector inj;
  GeConfig c = constantChain(0.4, 0.6);
  c.mode = CorruptMode::FROZEN;
  inj.setConfig(c);
  inj.reset(99);
  double lastGood = -1;
  int badSeen = 0;
  for (int i = 0; i < 5000; i++) {
    const double clean = 300.0 + 50.0 * std::sin(i * 0.15);
    const GeOutput o = inj.apply(clean);
    if (!o.bad) {
      CHECK(o.faulted_mm == GeInjector::quantise(clean));
      lastGood = clean;
    } else if (lastGood >= 0) {
      CHECK(o.faulted_mm == GeInjector::quantise(lastGood));
      badSeen++;
    }
  }
  CHECK(badSeen > 1000);
}

TEST(ge_variance_gives_total_variance_kappa_R) {
  // The reading already has variance R; the injector adds (kappa - 1) R, so the
  // total is kappa R (sd 10 x nominal for kappa = 100). Here the clean value is
  // exact, so only the added part is seen: sd = sqrt(99) * sigma_nom.
  GeInjector inj;
  GeConfig c = constantChain(1.0, 1.0);  // always BAD
  c.mode = CorruptMode::VARIANCE;
  c.sigma_nom_mm = 2.0;
  inj.setConfig(c);
  inj.reset(5);
  double s = 0, s2 = 0;
  const int N = 100000;
  for (int i = 0; i < N; i++) {
    const double e = inj.apply(300.0).faulted_mm - 300.0;
    s += e;
    s2 += e * e;
  }
  const double mean = s / N, sd = std::sqrt(s2 / N - mean * mean);
  CHECK(std::fabs(mean) < 0.1);
  CHECK(std::fabs(sd - std::sqrt(99.0) * 2.0) < 0.15);
}

TEST(ge_bias_modes) {
  // The paper's conditions, with sigma_nom = 2 mm:
  //   bias      clean + 4 sigma rho(r)              (no added noise)
  //   sysbias   clean + 4 sigma rho(r) + noise      (same noise as variance)
  //   randbias  clean + b_j + noise, b_j held for the whole burst
  //   blend(0) = variance, blend(1) = bias; the GOOD/BAD sequence is the same in every mode.
  GeConfig c = constantChain(0.2, 0.7);
  c.sigma_nom_mm = 2.0;
  GeInjector v, bi, sy, rb, b0, b1;
  c.mode = CorruptMode::VARIANCE; v.setConfig(c);
  c.mode = CorruptMode::BIAS;     bi.setConfig(c);
  c.mode = CorruptMode::SYSBIAS;  sy.setConfig(c);
  c.mode = CorruptMode::RANDBIAS; rb.setConfig(c);
  c.mode = CorruptMode::BLEND; c.lambda = 0.0; b0.setConfig(c);
  c.lambda = 1.0;                 b1.setConfig(c);
  for (GeInjector* g : {&v, &bi, &sy, &rb, &b0, &b1}) g->reset(2024);
  int bad = 0, bursts = 0;
  double burstBias = 0, sb = 0, sb2 = 0;
  bool prevBad = false;
  for (int i = 0; i < 60000; i++) {
    const double clean = 230.0 + 60.0 * std::sin(i * 0.2);
    const GeOutput ov = v.apply(clean), ob = bi.apply(clean), os = sy.apply(clean), orb = rb.apply(clean),
                   o0 = b0.apply(clean), o1 = b1.apply(clean);
    CHECK(ov.bad == ob.bad && ov.bad == os.bad && ov.bad == orb.bad && ov.bad == o0.bad && ov.bad == o1.bad);
    CHECK(ov.faulted_mm == o0.faulted_mm);
    CHECK(ob.faulted_mm == o1.faulted_mm);
    if (ob.bad) {
      bad++;
      const double prox = 4.0 * 2.0 * bi.rho(clean);
      CHECK(std::fabs(ob.faulted_mm - GeInjector::quantise(clean + prox)) < 1e-9);
      const double noise = ov.faulted_mm - clean;
      CHECK(std::fabs(os.faulted_mm - (clean + prox + noise)) < 0.002);
      const double off = orb.faulted_mm - clean - noise;  // this burst's random bias
      if (!prevBad) {
        bursts++;
        burstBias = off;
        sb += off;
        sb2 += off * off;
      } else {
        CHECK(std::fabs(off - burstBias) < 0.002);  // held for the whole burst
      }
    }
    prevBad = ob.bad;
  }
  CHECK(bad > 3000);
  const double mean = sb / bursts, sd = std::sqrt(sb2 / bursts - mean * mean);
  std::printf("    random burst bias over %d bursts: mean %.2f mm, sd %.2f mm (expected 0 and 20)\n", bursts, mean, sd);
  CHECK(std::fabs(sd - 20.0) < 1.5);
  CHECK(std::fabs(mean) < 1.5);
}

// ===========================================================================
// Estimators (kalman1d.h, gated_kf.h, imm1d.h)
// ===========================================================================
TEST(kf_converges_on_a_constant) {
  // A still target at 0.300 m read with +-2 mm noise: the estimate must settle
  // on 0.300 m, the velocity on 0, and the variance must shrink.
  KfEstimator kf;
  kf.params.q = 1e-6;  // the target does not move: almost no process noise
  SplitMix64 rng;
  rng.seed(1);
  for (int i = 0; i < 600; i++) kf.step(1.0 / 30, true, 0.300 + 0.002 * rng.gaussian());
  CHECK(std::fabs(kf.pos() - 0.300) < 0.0005);
  CHECK(std::fabs(kf.vel()) < 0.005);
  CHECK(kf.posVar() < kf.params.r / 5);
}

TEST(kf_follows_constant_velocity) {
  KfEstimator kf;
  for (int i = 0; i < 300; i++) kf.step(1.0 / 30, true, 0.25 + 0.1 * i / 30.0);
  CHECK(std::fabs(kf.vel() - 0.1) < 1e-3);
}

TEST(gate_rejects_a_large_outlier) {
  GatedKfEstimator g;
  KfEstimator k;
  g.params.q = k.params.q = 1e-3;
  for (int i = 0; i < 200; i++) {
    g.step(1.0 / 30, true, 0.300);
    k.step(1.0 / 30, true, 0.300);
  }
  g.step(1.0 / 30, true, 0.500);  // 200 mm jump: impossible for a 2 mm-noise sensor
  k.step(1.0 / 30, true, 0.500);
  CHECK(g.last_rejected);
  CHECK(g.rejected == 1);
  CHECK(std::fabs(g.pos() - 0.300) < 1e-4);  // the gated filter ignored it...
  CHECK(k.pos() > 0.301);                    // ...the plain KF was pulled towards it
  // A normal reading is accepted.
  g.step(1.0 / 30, true, 0.301);
  CHECK(!g.last_rejected);
}

TEST(gated_coasting_timeout_reacquires) {
  // The readings jump by 100 mm and stay there (as if the target moved
  // suddenly). Without the timeout the gate would reject them for ever; with
  // the paper's 1.0 s timeout the filter takes a reading after 1 s of
  // coasting and then follows the new position.
  GatedKfEstimator g;
  g.params.q = 1e-3;
  for (int i = 0; i < 200; i++) g.step(1.0 / 30, true, 0.300);
  int rejectedInARow = 0;
  for (int i = 0; i < 29; i++) {
    g.step(1.0 / 30, true, 0.400);
    rejectedInARow += g.last_rejected;
  }
  CHECK(rejectedInARow == 29);  // 29 steps of 1/30 s: still under 1.0 s
  CHECK(g.forced == 0);
  g.step(1.0 / 30, true, 0.400);  // 30th step: 1.0 s of coasting -> accepted
  CHECK(!g.last_rejected && g.forced == 1);
  for (int i = 0; i < 60; i++) g.step(1.0 / 30, true, 0.400);
  CHECK(std::fabs(g.pos() - 0.400) < 0.002);
  // Without the timeout it stays locked out much longer: still rejecting after
  // 2 s (it only comes back once its own uncertainty has grown past 100 mm).
  GatedKfEstimator stuck;
  stuck.params.q = 1e-4;
  stuck.params.coast_s = 1e9;
  for (int i = 0; i < 200; i++) stuck.step(1.0 / 30, true, 0.300);
  for (int i = 0; i < 60; i++) stuck.step(1.0 / 30, true, 0.400);
  CHECK(stuck.last_rejected && stuck.rejected == 60);
}

TEST(gated_coasting_counts_outage_time) {
  // An outage (no readings) also counts as coasting.
  GatedKfEstimator g;
  g.params.q = 1e-3;
  for (int i = 0; i < 100; i++) g.step(1.0 / 30, true, 0.300);
  for (int i = 0; i < 40; i++) g.step(1.0 / 30, false, 0.0);
  g.step(1.0 / 30, true, 0.900);  // absurd, but after > 1 s of coasting it is taken
  CHECK(!g.last_rejected && g.forced == 1);
}

TEST(imm_mode_probability_rises_during_a_burst) {
  ImmEstimator imm;
  SplitMix64 rng;
  rng.seed(3);
  double before = 0, during = 0, after = 0;
  for (int i = 0; i < 600; i++) {
    const bool burst = (i >= 300 && i < 360);
    const double sd = burst ? 0.05 : 0.002;  // 50 mm noise during the burst
    imm.step(1.0 / 30, true, 0.300 + sd * rng.gaussian());
    if (i >= 250 && i < 300) before += imm.pBad() / 50;
    if (i >= 310 && i < 360) during += imm.pBad() / 50;
    if (i >= 500 && i < 550) after += imm.pBad() / 50;
  }
  std::printf("    mean P(bad mode): before %.3f, during %.3f, after %.3f\n", before, during, after);
  CHECK(during > 0.8);
  CHECK(before < 0.3);
  CHECK(after < 0.3);
}

TEST(imm_step_with_matches_fixed_probabilities) {
  // stepWith() with the same probabilities every step = step() with them in params.
  ImmEstimator a, b;
  a.params.p_nb = b.params.p_nb = 0.02;
  a.params.p_bn = b.params.p_bn = 0.2;
  SplitMix64 rng;
  rng.seed(11);
  for (int i = 0; i < 500; i++) {
    const double z = 0.3 + 0.002 * rng.gaussian() + (i % 50 < 5 ? 0.05 * rng.gaussian() : 0.0);
    a.step(1.0 / 30, true, z);
    b.stepWith(1.0 / 30, true, z, 0.02, 0.2);
  }
  CHECK(a.pos() == b.pos() && a.pBad() == b.pBad());
}

TEST(imm_survives_huge_outlier_and_gaps) {
  ImmEstimator imm;
  for (int i = 0; i < 100; i++) imm.step(1.0 / 30, true, 0.3);
  imm.step(1.0 / 30, true, 1e6);  // absurd reading: likelihoods underflow without log handling
  CHECK(std::isfinite(imm.pos()) && std::isfinite(imm.pBad()));
  CHECK(std::fabs(imm.mu[0] + imm.mu[1] - 1.0) < 1e-12);
  for (int i = 0; i < 30; i++) imm.step(0.05, false, 0.0);  // an outage: predict only
  CHECK(std::isfinite(imm.pos()) && imm.posVar() > 0);
}

// ===========================================================================
// Serial command parser (serial_commands.h)
// ===========================================================================
TEST(parse_commands) {
  Command c;
  CHECK(parseCommand("ge 0.002 0.12 0.5 0.9", c));
  CHECK(!std::strcmp(c.name, "ge") && c.count == 4 && c.numbers[3] == 0.9);
  CHECK(parseCommand("MODE Blend 0.81\r\n", c));
  CHECK(!std::strcmp(c.name, "mode") && !std::strcmp(c.word, "blend") && c.count == 1 && c.numbers[0] == 0.81);
  CHECK(parseCommand("inject on", c) && !std::strcmp(c.word, "on") && c.count == 0);
  CHECK(parseCommand("ge 1e-4 0.088 1e+09", c) && c.count == 3 && c.numbers[2] == 1e9);
  CHECK(parseCommand("seed 12x", c) && c.bad_number);
  CHECK(!parseCommand("   ", c));
  LineBuffer lb;
  const char* text = "params\r\nhelp\n";
  int lines = 0;
  for (const char* p = text; *p; p++) lines += lb.feed(*p);
  CHECK(lines == 2);
}

// ---------------------------------------------------------------------------
// Reference files for the C++/Python equivalence tests (tests/test_injector.py,
// tests/test_estimators.py). The input is a made-up but deterministic swing.
// ---------------------------------------------------------------------------
static double referenceClean(int i) {
  // A damped swing quantised to whole mm, like the VL53L0X.
  const double t = i / 30.0;
  return std::floor(300.0 + 60.0 * std::exp(-0.02 * t) * std::sin(4.43 * t + 0.3) + 0.5);
}

static void writeInjectorReference(const std::string& dir) {
  const std::string path = dir + "/injector_reference.csv";
  FILE* f = std::fopen(path.c_str(), "w");
  if (!f) { std::printf("cannot write %s\n", path.c_str()); std::exit(2); }
  std::fprintf(f, "case,i,mode,lambda,entry_far,entry_near,persist_far,persist_near,r_ref_mm,d_src_mm,"
                  "sample_period_s,sigma_nom_mm,kappa,bias_c_sigmas,randbias_sigmas,seed,clean_mm,faulted_mm,bad\n");
  struct Case { CorruptMode mode; double lambda, ef, en, pf, pn, rref, dsrc, T, sig, kappa, bc, rbs; uint64_t seed; };
  const Case cases[] = {
      // The paper's chain with the default geometry, in every mode
      {CorruptMode::VARIANCE, 0.0, 0.002, 0.12, 0.5, 0.9, 40, 220, 1.0 / 30, 2.0, 100, 4, 10, 1},
      {CorruptMode::FROZEN, 0.0, 0.002, 0.12, 0.5, 0.9, 40, 220, 1.0 / 30, 2.0, 100, 4, 10, 2},
      {CorruptMode::BIAS, 0.0, 0.002, 0.12, 0.5, 0.9, 40, 220, 1.0 / 30, 2.0, 100, 4, 10, 3},
      {CorruptMode::RANDBIAS, 0.0, 0.002, 0.12, 0.5, 0.9, 40, 220, 1.0 / 30, 2.0, 100, 4, 10, 4},
      {CorruptMode::SYSBIAS, 0.0, 0.002, 0.12, 0.5, 0.9, 40, 220, 1.0 / 30, 2.0, 100, 4, 10, 5},
      {CorruptMode::BLEND, 0.81, 0.002, 0.12, 0.5, 0.9, 40, 220, 1.0 / 30, 2.0, 100, 4, 10, 6},
      // other geometry, period, noise and scales, and an extreme seed
      {CorruptMode::BLEND, 0.3, 0.01, 0.2, 0.6, 0.95, 25, 260, 0.0334, 1.5, 50, -3, 7, 18446744073709551615ULL},
  };
  int k = 0;
  for (const Case& c : cases) {
    GeInjector inj;
    GeConfig g;
    g.mode = c.mode; g.lambda = c.lambda; g.entry_far = c.ef; g.entry_near = c.en; g.persist_far = c.pf;
    g.persist_near = c.pn; g.r_ref_mm = c.rref; g.d_src_mm = c.dsrc; g.sample_period_s = c.T;
    g.sigma_nom_mm = c.sig; g.kappa = c.kappa; g.bias_c_sigmas = c.bc; g.randbias_sigmas = c.rbs;
    inj.setConfig(g);
    inj.reset(c.seed);
    for (int i = 0; i < 3000; i++) {
      const double clean = referenceClean(i);
      const GeOutput o = inj.apply(clean);
      std::fprintf(f, "%d,%d,%s,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%llu,%.17g,%.17g,%d\n",
                   k, i, corruptModeName(c.mode), c.lambda, c.ef, c.en, c.pf, c.pn, c.rref, c.dsrc, c.T, c.sig,
                   c.kappa, c.bc, c.rbs, (unsigned long long)c.seed, clean, o.faulted_mm, o.bad ? 1 : 0);
    }
    k++;
  }
  std::fclose(f);
  std::printf("wrote %s\n", path.c_str());
}

// The estimators on a faulted stream with gaps and uneven time steps.
// Non-default parameters, so a mismatch in any of them would show.
static void writeEstimatorReference(const std::string& dir) {
  const std::string path = dir + "/estimators_reference.csv";
  FILE* f = std::fopen(path.c_str(), "w");
  if (!f) { std::printf("cannot write %s\n", path.c_str()); std::exit(2); }
  std::fprintf(f, "# kf q=%.17g r=%.17g p0_vel=%.17g\n", 0.05, 9e-6, 0.5);
  std::fprintf(f, "# gated q=%.17g r=%.17g p0_vel=%.17g gate=%.17g coast_s=%.17g\n", 0.05, 9e-6, 0.5, 3.841, 0.5);
  std::fprintf(f, "# imm q=%.17g r=%.17g p0_vel=%.17g r_bad_factor=%.17g p_nb=%.17g p_bn=%.17g mu_bad0=%.17g\n",
               0.05, 9e-6, 0.5, 300.0, 0.03, 0.12, 0.2);
  std::fprintf(f, "t_us,faulted_mm,bad,kf_pos,kf_vel,kf_var,gated_pos,gated_vel,gated_var,gated_rejected,"
                  "gated_forced,imm_pos,imm_vel,imm_var,imm_p_bad,p_nb_true,p_bn_true,imm_oracle_pos,imm_oracle_vel,"
                  "imm_oracle_var,imm_oracle_p_bad\n");
  KfEstimator kf;
  kf.params.q = 0.05; kf.params.r = 9e-6; kf.params.p0_vel = 0.5;
  GatedKfEstimator g;
  g.params.q = 0.05; g.params.r = 9e-6; g.params.p0_vel = 0.5; g.params.gate = 3.841; g.params.coast_s = 0.5;
  ImmEstimator imm;
  imm.params.q = 0.05; imm.params.r = 9e-6; imm.params.p0_vel = 0.5; imm.params.r_bad_factor = 300.0;
  imm.params.p_nb = 0.03; imm.params.p_bn = 0.12; imm.params.mu_bad0 = 0.2;
  ImmEstimator oracle = imm;  // same settings, but the injector's true switch probabilities at each step

  GeInjector inj;
  GeConfig gc;  // the paper's chain; blend near the paper's crossover, so the gate and its timeout both act
  gc.mode = CorruptMode::BLEND; gc.lambda = 0.8; gc.d_src_mm = 250;
  inj.setConfig(gc);
  inj.reset(77);
  uint64_t t = 1000000, prevT = 0;
  for (int i = 0; i < 3000; i++) {
    t += 33000 + (uint64_t)((i * 7919) % 900);        // uneven sample times, like the real sensor
    const bool gap = (i % 400) >= 390 || i == 5;       // short outages ("no reading")
    double faulted = -1.0;
    bool bad = inj.bad();
    double pnb = 0.0, pbn = 0.0;  // no reading: the injector does not move, no switch possible
    if (!gap) {
      pnb = inj.pEntry(referenceClean(i));
      pbn = 1.0 - inj.pPersist(referenceClean(i));
      const GeOutput o = inj.apply(referenceClean(i));
      faulted = o.faulted_mm;
      bad = o.bad;
      // A sustained 60 mm offset for 100 readings: the gate rejects it until
      // the coasting timeout forces it to accept.
      if (i >= 1500 && i < 1600) faulted = GeInjector::quantise(faulted + 60.0);
    }
    // Exactly what main.cpp does for each row:
    const double dt = (prevT == 0) ? 0.0 : (double)(t - prevT) / 1e6;
    prevT = t;
    const bool has = faulted >= 0.0;
    const double z = has ? faulted / 1000.0 : 0.0;
    kf.step(dt, has, z);
    g.step(dt, has, z);
    imm.step(dt, has, z);
    oracle.stepWith(dt, has, z, pnb, pbn);
    std::fprintf(f, "%llu,%.3f,%d,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%d,%lu,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,"
                    "%.17g,%.17g,%.17g,%.17g\n",
                 (unsigned long long)t, faulted, bad ? 1 : 0, kf.pos(), kf.vel(), kf.posVar(), g.pos(), g.vel(),
                 g.posVar(), g.last_rejected ? 1 : 0, g.forced, imm.pos(), imm.vel(), imm.posVar(), imm.pBad(), pnb,
                 pbn, oracle.pos(), oracle.vel(), oracle.posVar(), oracle.pBad());
  }
  std::fclose(f);
  std::printf("wrote %s\n", path.c_str());
}

// ===========================================================================
int main(int argc, char** argv) {
  if (argc == 3 && std::strcmp(argv[1], "--reference") == 0) {
    writeInjectorReference(argv[2]);
    writeEstimatorReference(argv[2]);
    return 0;
  }
  for (auto& t : tests()) {
    const int before = g_failures;
    t.fn();
    std::printf("%s %s\n", g_failures == before ? "ok  " : "FAIL", t.name);
  }
  std::printf("\n%d checks, %d failures, %zu tests\n", g_checks, g_failures, tests().size());
  return g_failures == 0 ? 0 : 1;
}
