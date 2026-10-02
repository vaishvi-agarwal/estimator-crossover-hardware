// sel_sefi_guard.h — the complete guard for the ESP32: latch-up protection
// plus the lock-up watchdog, behind a small interface.
//
//   #include <sel_sefi_guard.h>
//   SelSefiGuard guard;
//   void setup() { guard.begin(); guard.start(); }
//   void loop() {
//     if (guard.update()) {                       // true when something happened
//       const GuardEvent& ev = guard.lastEvent();  // log it
//     }
//     uint16_t mm;
//     if (guard.newReading(mm)) { /* use the distance */ }
//     // guard.state() == GuardState::OK when readings can be trusted
//   }
//
// How it runs:
//   * A FreeRTOS task on core 0 does ONLY latch-up protection: read the
//     INA219 (its own I2C bus), run SelGuard, switch the rail. Nothing else
//     can delay it, not even a slow sensor re-initialisation (tens of ms).
//     It runs at the idle priority, so the idle task still gets its turn
//     (otherwise the ESP32's task watchdog would reset the board).
//   * update(), called from your loop() on core 1, runs the sensor side:
//     polling, hang detection and the recovery ladder (SensorSupervisor).
//   * The two sides only exchange a few atomic flags and an event queue.
//
// ESP32 + Arduino only. The decision logic it uses (sel_guard.h,
// sefi_ladder.h, vl53_guard.h) is plain C++ and unit-tested on a PC.
#pragma once
#ifdef ARDUINO
#include <Arduino.h>
#include <Wire.h>
#include <esp_timer.h>
#include <atomic>

#include "guard_types.h"
#include "ina219_fast.h"
#include "pins.h"
#include "sel_guard.h"
#include "sefi_ladder.h"
#include "vl53_guard.h"
#include "vl53_port_esp32.h"

// Pins and buses. The defaults are the wiring in pins.h / docs/circuit_explained.md.
struct GuardPins {
  uint8_t sensor_sda = pins::SENSOR_SDA;
  uint8_t sensor_scl = pins::SENSOR_SCL;
  uint8_t ina_sda = pins::INA_SDA;
  uint8_t ina_scl = pins::INA_SCL;
  uint8_t rail_enable = pins::RAIL_ENABLE;
  uint8_t xshut = pins::SENSOR_XSHUT;
  uint8_t status_led = pins::STATUS_LED;  // 255 = no LED
};

struct GuardSettings {
  SelConfig sel;                   // latch-up thresholds (see sel_guard.h)
  SupervisorConfig supervisor;     // lock-up watchdog + ladder (see vl53_guard.h)
  GuardPins pins;
  // TODO(hardware): measure the INA219 shunt resistor.
  float shunt_ohm = 0.100f;
  ina219::Pga pga = ina219::Pga::DIV8_320mV;
  uint32_t i2c_hz = 400000;
  uint32_t timing_budget_us = 33000;  // VL53L0X: ~30 readings per second
  uint8_t monitor_core = 0;           // the latch-up task runs on this core
  // In normal use, stop re-powering after this many trips in a row.
  // (The experiment commands set 0 = never lock out.)
  // TODO(hardware): if the soak test shows isolated false trips, keep this high
  // enough that they never cause a lockout.
  uint8_t max_consecutive_trips = 5;
};

// What callers usually need to know, in one word.
enum class GuardState : uint8_t {
  STOPPED = 0,     // guard not started (rail off)
  STARTING = 1,    // rail coming up / sensor starting
  OK = 2,          // readings flowing and trustworthy
  RECOVERING = 3,  // a lock-up was detected; the ladder is working on it
  TRIPPED = 4,     // latch-up detected; power is off for off_ms
  FAILED = 5,      // the ladder gave up (call resetFailed())
  LOCKOUT = 6,     // repeated latch-ups; power stays off (call resetFailed())
};

inline const char* guardStateName(GuardState s) {
  switch (s) {
    case GuardState::STOPPED: return "stopped";
    case GuardState::STARTING: return "starting";
    case GuardState::OK: return "ok";
    case GuardState::RECOVERING: return "recovering";
    case GuardState::TRIPPED: return "tripped";
    case GuardState::FAILED: return "failed";
    case GuardState::LOCKOUT: return "lockout";
  }
  return "?";
}

class SelSefiGuard {
 public:
  SelSefiGuard() : port_(sensor_), supervisor_(port_) {}

  // Set up pins, buses and the INA219. The rail stays OFF and the latch-up
  // task is created paused. Returns false if the INA219 does not answer.
  bool begin(const GuardSettings& s = GuardSettings()) {
    settings_ = s;
    pinMode(s.pins.rail_enable, OUTPUT);
    setRail(false);
    if (s.pins.status_led != 255) pinMode(s.pins.status_led, OUTPUT);

    Wire1.begin(s.pins.ina_sda, s.pins.ina_scl, s.i2c_hz);
    const bool inaOk = ina_.begin(Wire1, INA219_ADDR, s.shunt_ohm, s.pga);
    sensor_.configure(Wire, s.pins.sensor_sda, s.pins.sensor_scl, s.pins.xshut, s.i2c_hz, s.timing_budget_us);

    SelConfig sc = s.sel;
    sc.max_consecutive_trips = s.max_consecutive_trips;
    sel_.setConfig(sc);
    supervisor_.setConfig(s.supervisor);

    if (!task_) {
      pause_req_ = true;
      selEvents_ = xQueueCreate(16, sizeof(GuardEvent));
      xTaskCreatePinnedToCore(taskEntry, "sel_guard", 4096, this, tskIDLE_PRIORITY, &task_, s.monitor_core);
    }
    return inaOk;
  }

  // Start guarding: power the rail through the latch-up guard and let the
  // task run. The sensor starts itself once blanking is over.
  void start() {
    pauseMonitor();
    sel_.reset();
    if (sel_.powerOn(nowUs()) == RailCmd::ON) railOnHw();
    running_ = true;
    resumeMonitor();
  }

  // Stop guarding: pause the task, make the sensor pins safe, rail off.
  void stop() {
    pauseMonitor();
    sensor_.releaseBus();
    setRail(false);
    sel_.powerOff();
    running_ = false;
  }

  // Call from loop() as often as possible. Returns true if new events arrived
  // (the newest is lastEvent(); all of them can be read with popEvent()).
  bool update() {
    if (!running_) return false;
    // 1. Move latch-up events from the task into the single ordered stream.
    GuardEvent ev;
    while (xQueueReceive(selEvents_, &ev, 0) == pdTRUE) supervisor_.pushEvent(ev);
    // 2. Run the sensor side.
    supervisor_.update(rail_ready_.load(), rail_epoch_.load());
    if (supervisor_.takePowerCycleRequest()) pc_request_ = true;  // the task does it
    // 3. Collect events.
    bool any = false;
    while (supervisor_.popEvent(ev)) {
      last_ = ev;
      any = true;
      if (q_count_ == QUEUE) { q_head_ = (q_head_ + 1) % QUEUE; q_count_--; }
      q_[(q_head_ + q_count_) % QUEUE] = ev;
      q_count_++;
    }
    return any;
  }

  GuardState state() const {
    if (!running_) return GuardState::STOPPED;
    const SelState ss = (SelState)sel_state_.load();
    if (ss == SelState::LOCKOUT) return GuardState::LOCKOUT;
    if (ss == SelState::TRIPPED) return GuardState::TRIPPED;
    const SupervisorState sv = supervisor_.state();
    if (sv == SupervisorState::FAILED) return GuardState::FAILED;
    if (sv == SupervisorState::VERIFYING || sv == SupervisorState::WAIT_POWER || ss == SelState::CYCLING)
      return GuardState::RECOVERING;
    if (sv == SupervisorState::POWERED_OFF || ss != SelState::MONITOR) return GuardState::STARTING;
    if (supervisor_.outageOpen()) return GuardState::RECOVERING;
    return GuardState::OK;
  }

  const GuardEvent& lastEvent() const { return last_; }

  // Read events one by one, oldest first (keeps the newest 32).
  bool popEvent(GuardEvent& ev) {
    if (q_count_ == 0) return false;
    ev = q_[q_head_];
    q_head_ = (q_head_ + 1) % QUEUE;
    q_count_--;
    return true;
  }

  // True once for every new good distance reading.
  bool newReading(uint16_t& mm) {
    const uint32_t seq = supervisor_.readingSeq();
    if (seq == seen_reading_) return false;
    seen_reading_ = seq;
    mm = supervisor_.lastRange();
    return true;
  }

  // Clear FAILED / LOCKOUT after a human has checked the hardware, and power-cycle.
  void resetFailed() {
    pauseMonitor();
    supervisor_.resetFailed();
    sel_.reset();
    sensor_.releaseBus();
    setRail(false);
    delay(settings_.sel.off_us / 1000);
    if (running_ && sel_.powerOn(nowUs()) == RailCmd::ON) railOnHw();
    resumeMonitor();
  }

  // ---- Lower-level access, used by the experiment firmware (main.cpp) ----
  // Only touch these while the task is paused (stop() or pauseMonitor()).
  void pauseMonitor() {
    pause_req_ = true;
    const uint32_t t0 = millis();
    while (!pause_ack_ && millis() - t0 < 100) delay(1);
  }
  void resumeMonitor() {
    pause_ack_ = false;  // so the next pauseMonitor() waits for a FRESH acknowledgement
    pause_req_ = false;
  }
  bool running() const { return running_; }
  void setRail(bool on) {
    digitalWrite(settings_.pins.rail_enable, on ? HIGH : LOW);
    if (settings_.pins.status_led != 255) digitalWrite(settings_.pins.status_led, on ? HIGH : LOW);
    rail_on_ = on;
    if (!on) rail_ready_ = false;
  }
  bool railIsOn() const { return rail_on_; }
  Ina219Fast& ina() { return ina_; }
  Vl53Port& sensorPort() { return sensor_; }
  SelGuard& sel() { return sel_; }
  SensorSupervisor& supervisor() { return supervisor_; }
  const GuardSettings& settings() const { return settings_; }
  static uint64_t nowUs() { return (uint64_t)esp_timer_get_time(); }

 private:
  // The SensorPort the supervisor uses: the real VL53L0X.
  class Esp32Port : public SensorPort {
   public:
    explicit Esp32Port(Vl53Port& s) : s_(s) {}
    bool start() override { return s_.start(); }
    bool reinit() override { return s_.reinit(); }
    bool xshutReset() override { return s_.xshutReset(); }
    PollResult poll(uint16_t& mm) override { return s_.poll(mm); }
    void releaseBus() override { s_.releaseBus(); }
    uint64_t nowUs() override { return (uint64_t)esp_timer_get_time(); }

   private:
    Vl53Port& s_;
  };

  void railOnHw() {
    setRail(true);
    rail_epoch_++;
  }

  static void taskEntry(void* self) { static_cast<SelSefiGuard*>(self)->taskLoop(); }

  // The latch-up task (core 0). Keep it short and simple.
  void taskLoop() {
    uint16_t inaErrors = 0;
    uint32_t tripsSeen = 0;
    for (;;) {
      if (pause_req_) {
        pause_ack_ = true;
        vTaskDelay(1);
        continue;
      }
      pause_ack_ = false;

      const uint64_t t = nowUs();
      if (pc_request_.exchange(false)) apply(sel_.requestPowerCycle(t));

      float mA;
      if (!ina_.readMilliamps(mA)) {
        if (++inaErrors == 10) sendEvent(EventType::CURRENT_SENSOR_ERROR, t);
        continue;
      }
      inaErrors = 0;

      apply(sel_.update(t, mA));
      if (sel_.tripCount() != tripsSeen) {
        tripsSeen = sel_.tripCount();
        GuardEvent ev;
        ev.type = EventType::SEL_TRIP;
        ev.t_us = sel_.lastTrip().t_trip_us;
        ev.peak_mA = sel_.lastTrip().peak_mA;
        xQueueSend(selEvents_, &ev, 0);
        if (sel_.state() == SelState::LOCKOUT) sendEvent(EventType::SEL_LOCKOUT, t);
      }
      rail_ready_ = (sel_.state() == SelState::MONITOR);
      sel_state_ = (uint8_t)sel_.state();
    }
  }

  void apply(RailCmd cmd) {
    if (cmd == RailCmd::OFF) {
      setRail(false);          // cut power first...
      sensor_.holdInReset();   // ...then hold the sensor in reset (I2C pins are
                               // floated by update() on the other core)
    } else if (cmd == RailCmd::ON) {
      railOnHw();
    }
  }

  void sendEvent(EventType type, uint64_t t) {
    GuardEvent ev;
    ev.type = type;
    ev.t_us = t;
    xQueueSend(selEvents_, &ev, 0);
  }

  GuardSettings settings_;
  Ina219Fast ina_;
  Vl53Port sensor_;
  Esp32Port port_;
  SelGuard sel_;
  SensorSupervisor supervisor_;

  TaskHandle_t task_ = nullptr;
  QueueHandle_t selEvents_ = nullptr;
  std::atomic<bool> pause_req_{true};
  std::atomic<bool> pause_ack_{false};
  std::atomic<bool> pc_request_{false};
  std::atomic<bool> rail_ready_{false};
  std::atomic<uint32_t> rail_epoch_{0};
  std::atomic<uint8_t> sel_state_{0};
  volatile bool rail_on_ = false;
  bool running_ = false;

  static const int QUEUE = 32;
  GuardEvent q_[QUEUE];
  int q_head_ = 0, q_count_ = 0;
  GuardEvent last_;
  uint32_t seen_reading_ = 0;
};

#endif  // ARDUINO
