// main.cpp — estimator-crossover-hardware firmware.
//
// Two builds (see platformio.ini):
//   esp32dev        Blocks A-F. This repository's sensor_guard.h watches the
//                   VL53L0X for timeouts and hangs and resets it through XSHUT.
//   esp32dev_guard  Block G. The Spec 2 latch-up / lock-up guard (lib/sel_guard)
//                   owns the sensor, and its recovery events go into the
//                   recovery_event column. No software injection is needed:
//                   the real outages are the bursts.
//
// Every ~33 ms:
//   1. read the VL53L0X (sensor guard watches for timeouts and hangs),
//   2. pass a COPY of the clean reading through the Gilbert–Elliott injector,
//   3. feed the faulted reading to the KF, gated KF and IMM, timing each one,
//   4. print one CSV row over USB serial (921600 baud).
//
// Lines that start with '#' are metadata (settings, events, messages), not
// data. analysis/logger.py stores them in the session's JSON sidecar.
// Type `help` in the serial monitor for the list of commands.
#include <Arduino.h>

#include "board_pins.h"
#include "gated_kf.h"
#include "ge_injector.h"
#include "imm1d.h"
#include "kalman1d.h"
#include "sensor_guard.h"
#include "serial_commands.h"

#ifdef USE_SEL_GUARD
#include <sel_sefi_guard.h>
// The recovery_event column uses the guard's EventType numbers in BOTH builds
// (analysis/evaluate.py relies on it). Check they still agree:
static_assert((uint8_t)EventType::SENSOR_FAULT == (uint8_t)RecoveryEvent::SENSOR_FAULT, "event codes");
static_assert((uint8_t)EventType::LADDER_STEP == (uint8_t)RecoveryEvent::LADDER_STEP, "event codes");
static_assert((uint8_t)EventType::SENSOR_RECOVERED == (uint8_t)RecoveryEvent::SENSOR_RECOVERED, "event codes");
static_assert((uint8_t)EventType::SENSOR_FAILED == (uint8_t)RecoveryEvent::SENSOR_FAILED, "event codes");
#define BUILD_NAME "guard"
#else
#include "vl53_reader.h"
#define BUILD_NAME "standalone"
#endif

#ifndef FW_GIT_COMMIT
#define FW_GIT_COMMIT "unknown"
#endif

// If no reading has arrived for this long, print a row anyway with the
// distance columns set to -1 ("no measurement"), so the data shows the gap.
static const uint32_t MISSING_ROW_US = 50000;

#ifdef USE_SEL_GUARD
static SelSefiGuard selGuard;
#else
static Vl53Reader reader;
static SensorGuardLogic guard;
#endif
static GeInjector injector;
static bool injectOn = false;
static uint64_t seedValue = 1;
static LineBuffer lineBuf;
static KfEstimator kf;
static GatedKfEstimator gated;
static ImmEstimator imm;
static uint64_t prevSampleUs = 0;  // time of the previous row (for dt); 0 = first row after reset

static uint32_t seq = 0;          // row counter: a gap in seq means a row was lost
static uint64_t lastRowUs = 0;    // time of the last printed row
static uint8_t pendingEvent = 0;  // event code for the next row (0 = none)

static uint64_t nowUs() { return (uint64_t)esp_timer_get_time(); }

static void printHeader() {
  Serial.println("t_us,seq,raw_mm,clean_mm,faulted_mm,ge_state,mode,button,recovery_event,"
                 "kf_pos,kf_vel,gated_pos,gated_vel,imm_pos,imm_vel,imm_p_bad,kf_us,gated_us,imm_us");
}

// VL53L0X timing budget of the build (33 ms in both).
static uint32_t timingBudgetUs() {
#ifdef USE_SEL_GUARD
  return selGuard.settings().timing_budget_us;
#else
  return Vl53Reader::TIMING_BUDGET_US;
#endif
}

static void printParams() {
  const GeConfig& g = injector.config();
  // 17 significant digits: the exact doubles, so Python can replay a session exactly.
  Serial.printf("# params fw_commit=%s build=%s inject=%d mode=%s lambda=%.17g seed=%llu timing_budget_us=%lu\n",
                FW_GIT_COMMIT, BUILD_NAME, injectOn ? 1 : 0, corruptModeName(g.mode), g.lambda,
                (unsigned long long)seedValue, (unsigned long)timingBudgetUs());
  Serial.printf("# params entry_far=%.17g entry_near=%.17g persist_far=%.17g persist_near=%.17g r_ref_mm=%.17g "
                "d_src_mm=%.17g sample_period_s=%.17g\n",
                g.entry_far, g.entry_near, g.persist_far, g.persist_near, g.r_ref_mm, g.d_src_mm, g.sample_period_s);
  Serial.printf("# params sigma_nom_mm=%.17g kappa=%.17g bias_c_sigmas=%.17g randbias_sigmas=%.17g\n",
                g.sigma_nom_mm, g.kappa, g.bias_c_sigmas, g.randbias_sigmas);
  Serial.printf("# params kf_q=%.17g kf_r=%.17g kf_p0_vel=%.17g gated_q=%.17g gated_r=%.17g gated_p0_vel=%.17g "
                "gated_gate=%.17g gated_coast_s=%.17g\n",
                kf.params.q, kf.params.r, kf.params.p0_vel, gated.params.q, gated.params.r, gated.params.p0_vel,
                gated.params.gate, gated.params.coast_s);
  Serial.printf("# params imm_q=%.17g imm_r=%.17g imm_p0_vel=%.17g imm_r_bad_factor=%.17g imm_p_nb=%.17g "
                "imm_p_bn=%.17g imm_mu_bad0=%.17g\n",
                imm.params.q, imm.params.r, imm.params.p0_vel, imm.params.r_bad_factor, imm.params.p_nb,
                imm.params.p_bn, imm.params.mu_bad0);
  // Memory each estimator needs (RQ6), in bytes.
  Serial.printf("# sizeof kf_bytes=%u gated_bytes=%u imm_bytes=%u injector_bytes=%u\n", (unsigned)sizeof(kf),
                (unsigned)sizeof(gated), (unsigned)sizeof(imm), (unsigned)sizeof(injector));
}

// Start every estimator, the injector and the row counter afresh, and print
// the header. logger.py sends this last, so a recording starts from a known
// state and the Python estimators can replay it exactly.
static void resetAll() {
  kf.reset();
  gated.reset();
  imm.reset();
  injector.reset(seedValue);
  seq = 0;
  prevSampleUs = 0;
  Serial.println("# reset");
  printHeader();
}

static void printHelp() {
  Serial.println("# commands (fault-chain probabilities are the paper's values per 0.1 s; they are");
  Serial.println("# converted to the sample period, see docs/methods.md):");
  Serial.println("#   mode <variance|frozen|bias|randbias|sysbias|blend> [lambda]   how BAD readings are corrupted");
  Serial.println("#   ge <entry_far> <entry_near> <persist_far> <persist_near>   fault chain at 10 Hz");
  Serial.println("#   prox <d_src_mm> <r_ref_mm>           proximity: rho = r_ref^2 / (r^2 + r_ref^2), r = |d - d_src|");
  Serial.println("#   period <seconds>                     sample period used for the conversion (about 0.0333)");
  Serial.println("#   noise <sigma_nom_mm> <kappa>         nominal noise sd; BAD variance = kappa * R");
  Serial.println("#   bias <c> <sigma_b>                   bias scales in units of sigma_nom (paper: 4 and 10)");
  Serial.println("#   seed <n>                             restart the random sequence");
  Serial.println("#   inject <on|off>                      start / stop fault injection");
  Serial.println("#   q <q>                                shared process noise (m^2/s^3) for all three filters");
  Serial.println("#   kf <q> <r>                           KF process noise (m^2/s^3) and reading noise (m^2)");
  Serial.println("#   gated <q> <r> <gate> <coast_s>       gated KF (gate: chi-square, 1 DOF; coasting timeout in s)");
  Serial.println("#   imm <q> <r> <kappa_hat> <p_nb> <p_bn>   IMM; p_nb, p_bn = mode switch probabilities per sample");
  Serial.println("#   reset                                restart estimators, injector and seq; print header");
  Serial.println("#   params                               print all settings as # lines");
  Serial.println("#   header                               print the CSV header again");
  Serial.println("#   help                                 this list");
}

static bool isProb(double p) { return p >= 0.0 && p <= 1.0; }

// Apply one command line. Every change is confirmed with fresh `# params` lines.
static void handleCommand(const char* line) {
  Command c;
  if (!parseCommand(line, c)) return;
  GeConfig& g = injector.config();
  bool changed = true;
  if (!strcmp(c.name, "mode") && c.word[0]) {
    if (!strcmp(c.word, "variance")) g.mode = CorruptMode::VARIANCE;
    else if (!strcmp(c.word, "frozen")) g.mode = CorruptMode::FROZEN;
    else if (!strcmp(c.word, "bias")) g.mode = CorruptMode::BIAS;
    else if (!strcmp(c.word, "blend")) g.mode = CorruptMode::BLEND;
    else if (!strcmp(c.word, "randbias")) g.mode = CorruptMode::RANDBIAS;
    else if (!strcmp(c.word, "sysbias")) g.mode = CorruptMode::SYSBIAS;
    else { Serial.printf("# error unknown mode %s\n", c.word); return; }
    if (c.count >= 1) {
      if (!isProb(c.numbers[0])) { Serial.println("# error lambda must be 0..1"); return; }
      g.lambda = c.numbers[0];
    }
  } else if (!strcmp(c.name, "ge") && c.count == 4) {
    if (!isProb(c.numbers[0]) || !isProb(c.numbers[1]) || !isProb(c.numbers[2]) || !isProb(c.numbers[3])) {
      Serial.println("# error probabilities must be 0..1");
      return;
    }
    g.entry_far = c.numbers[0];
    g.entry_near = c.numbers[1];
    g.persist_far = c.numbers[2];
    g.persist_near = c.numbers[3];
  } else if (!strcmp(c.name, "ge")) {
    Serial.println("# error ge needs 4 numbers (10 Hz values). For Spec 2's ge_parameters.json use "
                   "analysis/ge_params.py, which prints the right command");
    return;
  } else if (!strcmp(c.name, "prox") && c.count == 2 && c.numbers[1] > 0) {
    g.d_src_mm = c.numbers[0];
    g.r_ref_mm = c.numbers[1];
  } else if (!strcmp(c.name, "period") && c.count == 1 && c.numbers[0] > 0) {
    g.sample_period_s = c.numbers[0];
  } else if (!strcmp(c.name, "noise") && c.count == 2 && c.numbers[0] >= 0 && c.numbers[1] >= 1) {
    g.sigma_nom_mm = c.numbers[0];
    g.kappa = c.numbers[1];
  } else if (!strcmp(c.name, "bias") && c.count == 2 && c.numbers[1] >= 0) {
    g.bias_c_sigmas = c.numbers[0];
    g.randbias_sigmas = c.numbers[1];
  } else if (!strcmp(c.name, "seed") && c.count == 1 && c.numbers[0] >= 0) {
    seedValue = (uint64_t)c.numbers[0];
    injector.reset(seedValue);
  } else if (!strcmp(c.name, "inject") && (!strcmp(c.word, "on") || !strcmp(c.word, "off"))) {
    injectOn = !strcmp(c.word, "on");
    injector.reset(seedValue);  // every injection run starts from the seed: repeatable
  } else if (!strcmp(c.name, "q") && c.count == 1 && c.numbers[0] >= 0) {
    kf.params.q = gated.params.q = imm.params.q = c.numbers[0];
  } else if (!strcmp(c.name, "kf") && c.count == 2 && c.numbers[0] >= 0 && c.numbers[1] > 0) {
    kf.params.q = c.numbers[0];
    kf.params.r = c.numbers[1];
  } else if (!strcmp(c.name, "gated") && c.count == 4 && c.numbers[0] >= 0 && c.numbers[1] > 0 &&
             c.numbers[2] > 0 && c.numbers[3] > 0) {
    gated.params.q = c.numbers[0];
    gated.params.r = c.numbers[1];
    gated.params.gate = c.numbers[2];
    gated.params.coast_s = c.numbers[3];
  } else if (!strcmp(c.name, "imm") && c.count == 5 && c.numbers[0] >= 0 && c.numbers[1] > 0 &&
             c.numbers[2] >= 1 && isProb(c.numbers[3]) && isProb(c.numbers[4])) {
    imm.params.q = c.numbers[0];
    imm.params.r = c.numbers[1];
    imm.params.r_bad_factor = c.numbers[2];
    imm.params.p_nb = c.numbers[3];
    imm.params.p_bn = c.numbers[4];
  } else if (!strcmp(c.name, "reset")) {
    resetAll();
    changed = false;
  } else if (!strcmp(c.name, "params")) {
    changed = true;
  } else if (!strcmp(c.name, "header")) {
    printHeader();
    changed = false;
  } else if (!strcmp(c.name, "help")) {
    printHelp();
    changed = false;
  } else {
    Serial.printf("# error unknown or incomplete command: %s (type help)\n", line);
    changed = false;
  }
  if (changed) printParams();
}

// Several events can happen between two rows. The row shows the most
// important one; every event is also printed in full on its own '#' line.
// Codes (same in both builds): 1 latch-up trip, 2 latch-up lockout,
// 3 sensor fault, 4 recovery step, 5 recovered, 6 failed, 7 current sensor error.
static uint8_t priority(uint8_t code) {
  switch (code) {
    case 1: case 2: case 3: return 4;  // an outage starts
    case 5: return 3;                  // an outage ends
    case 6: case 7: return 2;
    case 4: return 1;
  }
  return 0;
}

#ifndef USE_SEL_GUARD
static void logEvent(uint64_t t, const GuardStep& s, bool ok) {
  if (s.event == RecoveryEvent::NONE) return;
  const uint8_t code = (uint8_t)s.event;
  Serial.printf("# event t_us=%llu type=%u reason=%s ok=%d outage_us=%lu\n", (unsigned long long)t, code,
                hangReasonName(s.reason), ok ? 1 : 0, (unsigned long)s.outage_us);
  if (priority(code) > priority(pendingEvent)) pendingEvent = code;
}
#endif

// One CSV row. clean_mm < 0 means "no usable reading this time".
static void processSample(uint64_t t, int raw, int clean) {
  double faulted = -1.0;
  if (clean >= 0) {
    if (injectOn) {
      const GeOutput o = injector.apply((double)clean);
      faulted = o.faulted_mm;
    } else {
      faulted = (double)clean;
    }
  }
  const bool bad = injectOn && injector.bad();
#ifndef USE_SEL_GUARD
  digitalWrite(board::LED, bad ? HIGH : LOW);  // (in the guard build the guard owns the LED)
#endif
  const int button = digitalRead(board::BUTTON) == LOW ? 1 : 0;  // pressed = pin pulled to GND

  // Estimators. dt = seconds since the previous row (0 for the first row after
  // a reset). z in metres. This is exactly what analysis/estimators.py replays.
  const double dt = (prevSampleUs == 0) ? 0.0 : (double)(t - prevSampleUs) / 1e6;
  prevSampleUs = t;
  const bool has = faulted >= 0.0;
  const double z = has ? faulted / 1000.0 : 0.0;
  uint64_t t0 = nowUs();
  kf.step(dt, has, z);
  uint64_t t1 = nowUs();
  gated.step(dt, has, z);
  uint64_t t2 = nowUs();
  imm.step(dt, has, z);
  uint64_t t3 = nowUs();

  // Positions in metres with 7 decimals (0.1 micrometre). "nan" until an
  // estimator has had its first reading.
  Serial.printf("%llu,%lu,%d,%d,%.3f,%d,%s,%d,%u,%.7f,%.7f,%.7f,%.7f,%.7f,%.7f,%.6f,%lu,%lu,%lu\n",
                (unsigned long long)t, (unsigned long)seq++, raw, clean, faulted, bad ? 1 : 0,
                injectOn ? corruptModeName(injector.config().mode) : "off", button, pendingEvent,
                kf.ready() ? kf.pos() : NAN, kf.ready() ? kf.vel() : NAN, gated.ready() ? gated.pos() : NAN,
                gated.ready() ? gated.vel() : NAN, imm.ready() ? imm.pos() : NAN, imm.ready() ? imm.vel() : NAN,
                imm.ready() ? imm.pBad() : NAN, (unsigned long)(t1 - t0), (unsigned long)(t2 - t1),
                (unsigned long)(t3 - t2));
  pendingEvent = 0;
  lastRowUs = t;
}

void setup() {
  Serial.begin(921600);
  pinMode(board::BUTTON, INPUT_PULLUP);
#ifndef USE_SEL_GUARD
  pinMode(board::LED, OUTPUT);
  digitalWrite(board::LED, LOW);
#endif
  delay(200);
  Serial.printf("# fw_commit=%s build=%s\n", FW_GIT_COMMIT, BUILD_NAME);
#ifdef USE_SEL_GUARD
  GuardSettings gs;  // defaults = the wiring in the Spec 2 repository (docs/circuit_explained.md there)
  // TODO(hardware): copy the latch-up operating point measured in Spec 2
  // (its results/tables/operating_point.json) before Block G:
  // gs.sel.threshold_mA = ...; gs.sel.debounce = ...; gs.sel.blank_us = ...; gs.sel.off_us = ...;
  if (!selGuard.begin(gs)) Serial.println("# warning INA219 not found: latch-up protection is blind");
  selGuard.start();
#else
  if (!reader.begin()) Serial.println("# warning sensor_init_failed (check wiring, see docs/BRING_UP.md)");
  guard.reset(nowUs());
#endif
  printParams();
  resetAll();
}

#ifdef USE_SEL_GUARD
// Block G: the Spec 2 guard reads the sensor and reports its events.
static void sensorStep() {
  if (selGuard.update()) {
    GuardEvent e;
    while (selGuard.popEvent(e)) {
      Serial.printf("# guard_event seq=%lu type=%u t_us=%llu fault=%s step=%s ok=%d duration_us=%lu peak_mA=%.1f\n",
                    (unsigned long)e.seq, (unsigned)e.type, (unsigned long long)e.t_us, faultName(e.fault),
                    stepName(e.step), e.ok ? 1 : 0, (unsigned long)e.duration_us, e.peak_mA);
      if (priority((uint8_t)e.type) > priority(pendingEvent)) pendingEvent = (uint8_t)e.type;
    }
  }
  const uint64_t t = nowUs();
  uint16_t mm;
  if (selGuard.newReading(mm)) {
    processSample(t, mm, mm);  // the guard has already rejected hung / stuck readings
  } else if (t - lastRowUs >= MISSING_ROW_US) {
    processSample(t, -1, -1);
  }
}
#else
// Blocks A-F: our own sensor guard.
static void sensorStep() {
  uint16_t mm = 0;
  const ReadResult r = reader.poll(mm);
  const uint64_t t = nowUs();
  GuardStep s = guard.onPoll(t, r, mm);
  logEvent(t, s, false);
  if (s.request_recovery) {
    // XSHUT low for 10 ms, then initialise again (about 20 ms in total).
    const bool ok = reader.xshutReset();
    const uint64_t t2 = nowUs();
    logEvent(t2, guard.onRecoveryAttempt(t2, ok), ok);
  }
  // One row per reading, or a "no reading" row if the sensor has gone quiet.
  if (r == ReadResult::READY) {
    processSample(t, mm, s.reading_ok ? (int)mm : -1);
  } else if (t - lastRowUs >= MISSING_ROW_US) {
    processSample(t, -1, -1);
  }
}
#endif

void loop() {
  // 1. Commands typed in the serial monitor (or sent by logger.py).
  while (Serial.available()) {
    if (lineBuf.feed((char)Serial.read())) handleCommand(lineBuf.line());
  }
  // 2. Sensor -> injector -> estimators -> one CSV row.
  sensorStep();
}
