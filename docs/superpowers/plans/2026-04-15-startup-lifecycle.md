# Startup & Lifecycle State Machine Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace immediate boot-to-standing with a proper lifecycle: boot → rest pose → stand → IDLE (10s) → rest and detach; wake on control lock, sleep on release.

**Architecture:** Firmware owns the lifecycle state machine (BOOTING/WAKING/IDLE/ACTIVE/SLEEPING/RESTING) and runs it non-blocking inside `lifecycle_update()`. Host fires `cmd_wake`/`cmd_sleep` JSON commands when the control lock is acquired/released. Sim mirrors the same state machine.

**Tech Stack:** C++ / ArduinoJson / ESP32 LEDC (firmware), Python asyncio / aiohttp (host), vanilla JS (web)

---

## File Map

**Modified firmware files:**
- `firmware/include/config.h` — add `REST_POSE[8]`, `IDLE_TIMEOUT_MS`, `REST_SETTLE_MS`
- `firmware/include/servos.h` — add `servos_attach_at()`, `servos_ramp_to()`
- `firmware/src/servos.cpp` — implement new primitives, refactor init/shutdown
- `firmware/include/gait.h` — add `LifecycleState` enum + declarations
- `firmware/src/gait.cpp` — implement lifecycle state machine
- `firmware/include/protocol.h` — add `MSG_CMD_WAKE`, `MSG_CMD_SLEEP`
- `firmware/include/command_handlers.h` — add lifecycle accessor
- `firmware/src/command_handlers.cpp` — add wake/sleep handlers, gate motion commands
- `firmware/src/main.cpp` — new boot sequence, lifecycle integration
- `firmware/test/test_servos.cpp` — update tests for new servo API
- **New:** `firmware/test/test_lifecycle.cpp` — lifecycle transition tests

**Modified host files:**
- `host/comms.py` — add `send_json()` to Transport ABC, add `DogComms.wake()`/`sleep()`
- `host/json_transport_base.py` — parse `lifecycle` from `telem_status`
- `host/server.py` — lock→lifecycle bridge, broadcast lifecycle, remove REPL idle hack
- `host/sim/sim_transport.py` — lifecycle state machine
- `host/sim/physics.py` — add `REST_HIP`/`REST_KNEE`, rest pose ramp support

**Modified web file:**
- `web/dog3d/overlay.js` — lifecycle status indicator

---

## Task 1: Tune and record REST_POSE via hardware

> **Hardware required.** Place the dog on a flat surface before starting.

**Files:** Modify: `firmware/include/config.h`

- [ ] **Step 1: Enter test mode with frail protection**

  Connect to host and open browser. Acquire control lock. In browser console or via a temporary dev script, send:
  ```json
  {"type": "cmd_test_mode", "enable": true, "frail": true}
  ```
  LEDs should turn purple. Frail mode limits movement to ±200µs from standing.

- [ ] **Step 2: Move each servo to rest position using cmd_servo**

  Target: hips rotated back (body lowered to ground), knees tucked inward. Use the 3D view to monitor. Send one at a time:
  ```json
  {"type": "cmd_servo", "index": 0, "pulse_us": 1300}
  ```
  Indices: 0=FL_hip, 1=FL_knee, 2=FR_hip, 3=FR_knee, 4=RL_hip, 5=RL_knee, 6=RR_hip, 7=RR_knee

  **Starting guess** (hips half-retracted from standing, knees at center):
  ```
  FL_hip(0):  1800  (standing=2096, center=1500; ~halfway back)
  FL_knee(1): 1500  (adjust after hip settled)
  FR_hip(2):  1870  (standing=2170)
  FR_knee(3): 1500
  RL_hip(4):  1202  (standing=904; higher=back for RL)
  RL_knee(5): 1500
  RR_hip(6):  1165  (standing=830; higher=back for RR)
  RR_knee(7): 1500
  ```
  Adjust until the dog sits flat and stable with the body close to the ground.

- [ ] **Step 3: Record the tuned values and update config.h**

  Open `firmware/include/config.h`. After the `LYING_DOWN_POSE` block (line 68), add:

  ```cpp
  // --- Rest Pose (servo pulse widths in μs) ---
  // Tuned: hips back, knees tucked, body low to ground. Used on boot and shutdown.
  // Replace values below with results from hardware tuning (Task 1).
  // Order: FL_hip, FL_knee, FR_hip, FR_knee, RL_hip, RL_knee, RR_hip, RR_knee
  static const uint16_t REST_POSE[8] = {
      1800, 1500, 1870, 1500, 1202, 1500, 1165, 1500
  };

  // --- Lifecycle Timing ---
  #define IDLE_TIMEOUT_MS   10000   // ms from lock release to sleep
  #define REST_SETTLE_MS      500   // pause at rest pose after sleeping before detach
  ```

- [ ] **Step 4: Exit test mode and verify pose visually**

  Send `{"type": "cmd_test_mode", "enable": false}`. Dog should ramp back to standing. Note the final REST_POSE values for use in Step 5 below (sim physics.py).

- [ ] **Step 5: Commit**

  ```bash
  git add firmware/include/config.h
  git commit -m "feat(firmware): add REST_POSE and IDLE_TIMEOUT_MS to config"
  ```

---

## Task 2: Refactor servo primitives — servos_attach_at() and servos_ramp_to()

**Files:** Modify: `firmware/include/servos.h`, `firmware/src/servos.cpp`, `firmware/test/test_servos.cpp`

- [ ] **Step 1: Write new test for servos_attach_at()**

  In `firmware/test/test_servos.cpp`, add before `int main()`:

  ```cpp
  // ------------------------------------------------------------------ //
  // Test 8: servos_attach_at() starts at given pose without ramping
  // ------------------------------------------------------------------ //
  static void test_attach_at_rest() {
      printf("\nTest: servos_attach_at starts at REST_POSE\n");
      servo_log_reset();
      mock_reset_clock();

      servos_attach_at(REST_POSE);

      bool all_correct = true;
      for (int i = 0; i < 8; i++) {
          uint32_t expected = expected_duty(REST_POSE[i]);
          uint32_t actual   = _servo_duty[SERVO_PINS[i]];
          if (actual != expected) {
              printf("  servo %d: expected duty %u got %u\n", i, expected, actual);
              all_correct = false;
          }
      }
      check(all_correct, "all servos at REST_POSE duty after attach_at");
      check(servos_active(), "servos_active() true after attach_at");

      // Verify no ramp occurred — log should be exactly 8 writes (one per servo)
      int writes = 0;
      for (int i = 0; i < servo_log_count(); i++) writes++;
      check(writes == 8, "attach_at writes exactly once per servo (no ramp)");
  }
  ```

  Add call at end of `main()`:
  ```cpp
  test_attach_at_rest();
  ```

- [ ] **Step 2: Run test — verify it fails**

  ```bash
  cd /Users/gwen/workspace/bark-buddy
  pio test -e native -f test_servos
  ```
  Expected: FAIL — `servos_attach_at` is not defined.

- [ ] **Step 3: Add declarations to servos.h**

  In `firmware/include/servos.h`, replace the existing content with:

  ```cpp
  #pragma once
  #include <stdint.h>

  // Attach all 8 servos at the given pose (no ramp — writes pose directly).
  // Returns false if PINS_VERIFIED is 0.
  // Use before lifecycle_init() on boot; servos must already be at this pose physically.
  bool servos_attach_at(const uint16_t pose[8]);

  // Ramp all servos from current positions to target[] over duration_ms in `steps` steps.
  // Blocking. Servos must be attached (servos_active() == true).
  void servos_ramp_to(const uint16_t target[8], uint16_t duration_ms, uint8_t steps);

  // Initialize 8-servo LEDC hardware PWM and soft-start to standing pose.
  // Kept for backward compat (test_mode / cmd_servo re-attach path).
  // Attaches at REST_POSE, settles BOOT_SETTLE_MS, ramps to STANDING_POSE over SOFTSTART_DURATION_MS.
  bool servos_init();

  // Ramp to REST_POSE, settle REST_SETTLE_MS, then detach. Blocking (~2s).
  // Returns false if servos were not attached.
  bool servos_shutdown_to_rest();

  // Set servo pulse width in microseconds (clamped to min/max).
  // In frail mode: range clamp ±FRAIL_MAX_OFFSET_US + slew rate limit.
  void servo_write_us(uint8_t index, uint16_t pulse_us);

  // Read current pulse width for a servo.
  uint16_t servo_read_us(uint8_t index);

  // Detach all servos (stop PWM output).
  void servos_detach_all();

  // Are servos currently attached and active?
  bool servos_active();

  // --- Frail mode ---
  void servos_set_frail(bool enabled);
  bool servos_frail();
  bool servos_update_duty(unsigned long now_ms);
  ```

- [ ] **Step 4: Implement servos_attach_at() and servos_ramp_to() in servos.cpp**

  In `firmware/src/servos.cpp`, replace the existing `servos_init()` and `servos_shutdown_to_lying_down()` with:

  ```cpp
  bool servos_attach_at(const uint16_t pose[8]) {
  #if !PINS_VERIFIED
      Serial.println("{\"type\":\"error\",\"msg\":\"PINS_VERIFIED=0, servos disabled\"}");
      return false;
  #else
      for (int i = 0; i < 8; i++) {
          ledcSetup(i, SERVO_FREQ_HZ, LEDC_RESOLUTION);
          ledcAttachPin(SERVO_PINS[i], i);
      }
      attached = true;
      for (int i = 0; i < 8; i++) {
          uint16_t pos = clamp_us(pose[i]);
          current_us[i] = pos;
          ledcWrite(SERVO_PINS[i], us_to_duty(pos));
      }
      return true;
  #endif
  }

  void servos_ramp_to(const uint16_t target[8], uint16_t duration_ms, uint8_t steps) {
      if (!attached || steps == 0) return;
      uint16_t start_us[8];
      for (int i = 0; i < 8; i++) {
          start_us[i] = current_us[i] > 0 ? current_us[i] : target[i];
      }
      for (int step = 0; step <= steps; step++) {
          float t = (float)step / (float)steps;
          for (int i = 0; i < 8; i++) {
              int16_t s   = (int16_t)start_us[i];
              int16_t e   = (int16_t)target[i];
              uint16_t pos = clamp_us((uint16_t)(s + (int16_t)((float)(e - s) * t)));
              current_us[i] = pos;
              ledcWrite(SERVO_PINS[i], us_to_duty(pos));
          }
          delay(duration_ms / steps);
      }
  }

  bool servos_init() {
  #if !PINS_VERIFIED
      Serial.println("{\"type\":\"error\",\"msg\":\"PINS_VERIFIED=0, servos disabled\"}");
      return false;
  #else
      if (!servos_attach_at(REST_POSE)) return false;
      delay(BOOT_SETTLE_MS);
      servos_ramp_to(STANDING_POSE, SOFTSTART_DURATION_MS, SOFTSTART_STEPS);
      return true;
  #endif
  }

  bool servos_shutdown_to_rest() {
      if (!attached) return false;
      servos_ramp_to(REST_POSE, SHUTDOWN_RAMP_MS, SHUTDOWN_RAMP_STEPS);
      delay(REST_SETTLE_MS);
      servos_detach_all();
      return true;
  }
  ```

  Remove the old `servos_shutdown_to_lying_down()` function entirely.

- [ ] **Step 5: Run tests — verify they pass**

  ```bash
  pio test -e native -f test_servos
  ```
  Expected: all tests pass including new `test_attach_at_rest`.

  Note: `test_shutdown_ramp` tests `servos_shutdown_to_lying_down()`. Update that test to call `servos_shutdown_to_rest()` and check it ends at `REST_POSE` duties (all zero after detach).

- [ ] **Step 6: Commit**

  ```bash
  git add firmware/include/servos.h firmware/src/servos.cpp firmware/test/test_servos.cpp
  git commit -m "feat(firmware): add servos_attach_at/ramp_to, refactor init/shutdown to REST_POSE"
  ```

---

## Task 3: Lifecycle state machine in gait.h / gait.cpp

**Files:** Modify: `firmware/include/gait.h`, `firmware/src/gait.cpp`

The lifecycle runs non-blocking inside `lifecycle_update()`. WAKING and SLEEPING states interpolate servo positions each call using millis timestamps — same approach as the existing `s_stand_ramp_start` taper in `gait_update()`.

- [ ] **Step 1: Write lifecycle tests (test_lifecycle.cpp)**

  Create `firmware/test/test_lifecycle.cpp`:

  ```cpp
  // test_lifecycle.cpp — Tests for the lifecycle state machine.

  #include "mock_arduino.h"
  #include "mock_preferences.h"
  #include "../include/config.h"
  #include "../include/servos.h"
  #include "../include/gait.h"
  #include "../include/offsets.h"
  #include "../include/balance.h"
  #include "../src/servos.cpp"
  #include "../src/offsets.cpp"
  #include "../src/balance.cpp"
  #include "../src/gait.cpp"

  #include <cstdio>
  #include <cstdlib>

  static int g_pass = 0;
  static int g_fail = 0;

  static void check(bool cond, const char* label) {
      if (cond) { printf("  PASS  %s\n", label); g_pass++; }
      else       { printf("  FAIL  %s\n", label); g_fail++; }
  }

  // ------------------------------------------------------------------ //
  // Test 1: Fresh lifecycle_init() starts in WAKING
  // ------------------------------------------------------------------ //
  static void test_init_starts_waking() {
      printf("\nTest: lifecycle_init starts in WAKING\n");
      mock_reset_clock();
      offsets_init();
      servos_attach_at(REST_POSE);
      gait_init();

      check(lifecycle_current() == LifecycleState::WAKING, "lifecycle starts in WAKING");
      check(lifecycle_can_command() == false, "can_command false while WAKING");
  }

  // ------------------------------------------------------------------ //
  // Test 2: WAKING transitions to IDLE after SOFTSTART_DURATION_MS
  // ------------------------------------------------------------------ //
  static void test_waking_to_idle() {
      printf("\nTest: WAKING → IDLE after softstart duration\n");
      mock_reset_clock();
      offsets_init();
      servos_attach_at(REST_POSE);
      gait_init();

      // Advance past SOFTSTART_DURATION_MS
      mock_advance_clock(SOFTSTART_DURATION_MS + 10);
      lifecycle_update(millis());

      check(lifecycle_current() == LifecycleState::IDLE, "WAKING → IDLE after duration");
      check(lifecycle_can_command() == false, "can_command false in IDLE");
  }

  // ------------------------------------------------------------------ //
  // Test 3: cmd_wake during WAKING → ACTIVE when ramp completes
  // ------------------------------------------------------------------ //
  static void test_wake_during_waking_goes_active() {
      printf("\nTest: cmd_wake during WAKING sets goal to ACTIVE\n");
      mock_reset_clock();
      offsets_init();
      servos_attach_at(REST_POSE);
      gait_init();

      lifecycle_cmd_wake();  // wake called before ramp done

      mock_advance_clock(SOFTSTART_DURATION_MS + 10);
      lifecycle_update(millis());

      check(lifecycle_current() == LifecycleState::ACTIVE, "WAKING+wake → ACTIVE after duration");
      check(lifecycle_can_command() == true, "can_command true in ACTIVE");
  }

  // ------------------------------------------------------------------ //
  // Test 4: cmd_wake from IDLE → ACTIVE immediately
  // ------------------------------------------------------------------ //
  static void test_wake_from_idle() {
      printf("\nTest: cmd_wake from IDLE → ACTIVE immediately\n");
      mock_reset_clock();
      offsets_init();
      servos_attach_at(REST_POSE);
      gait_init();

      // Advance to IDLE
      mock_advance_clock(SOFTSTART_DURATION_MS + 10);
      lifecycle_update(millis());
      check(lifecycle_current() == LifecycleState::IDLE, "precondition: in IDLE");

      lifecycle_cmd_wake();
      lifecycle_update(millis());

      check(lifecycle_current() == LifecycleState::ACTIVE, "IDLE + wake → ACTIVE");
  }

  // ------------------------------------------------------------------ //
  // Test 5: IDLE times out → SLEEPING
  // ------------------------------------------------------------------ //
  static void test_idle_timeout_sleeps() {
      printf("\nTest: IDLE times out → SLEEPING\n");
      mock_reset_clock();
      offsets_init();
      servos_attach_at(REST_POSE);
      gait_init();

      mock_advance_clock(SOFTSTART_DURATION_MS + 10);
      lifecycle_update(millis());
      check(lifecycle_current() == LifecycleState::IDLE, "precondition: in IDLE");

      mock_advance_clock(IDLE_TIMEOUT_MS + 10);
      lifecycle_update(millis());

      check(lifecycle_current() == LifecycleState::SLEEPING, "IDLE timeout → SLEEPING");
  }

  // ------------------------------------------------------------------ //
  // Test 6: ACTIVE + cmd_sleep → IDLE
  // ------------------------------------------------------------------ //
  static void test_sleep_from_active() {
      printf("\nTest: ACTIVE + cmd_sleep → IDLE\n");
      mock_reset_clock();
      offsets_init();
      servos_attach_at(REST_POSE);
      gait_init();

      mock_advance_clock(SOFTSTART_DURATION_MS + 10);
      lifecycle_update(millis());
      lifecycle_cmd_wake();
      lifecycle_update(millis());
      check(lifecycle_current() == LifecycleState::ACTIVE, "precondition: ACTIVE");

      lifecycle_cmd_sleep();
      lifecycle_update(millis());

      check(lifecycle_current() == LifecycleState::IDLE, "ACTIVE + sleep → IDLE");
      check(lifecycle_can_command() == false, "can_command false after sleep");
  }

  // ------------------------------------------------------------------ //
  // Test 7: SLEEPING completes → RESTING (servos detached)
  // ------------------------------------------------------------------ //
  static void test_sleeping_to_resting() {
      printf("\nTest: SLEEPING completes → RESTING\n");
      mock_reset_clock();
      offsets_init();
      servos_attach_at(REST_POSE);
      gait_init();

      // Drive to SLEEPING state
      mock_advance_clock(SOFTSTART_DURATION_MS + 10);
      lifecycle_update(millis());
      mock_advance_clock(IDLE_TIMEOUT_MS + 10);
      lifecycle_update(millis());
      check(lifecycle_current() == LifecycleState::SLEEPING, "precondition: SLEEPING");

      // Advance past ramp + settle
      mock_advance_clock(SHUTDOWN_RAMP_MS + REST_SETTLE_MS + 10);
      lifecycle_update(millis());

      check(lifecycle_current() == LifecycleState::RESTING, "SLEEPING → RESTING");
      check(!servos_active(), "servos detached in RESTING");
  }

  // ------------------------------------------------------------------ //
  // Test 8: RESTING + cmd_wake → WAKING (re-attaches at REST_POSE)
  // ------------------------------------------------------------------ //
  static void test_wake_from_resting() {
      printf("\nTest: RESTING + cmd_wake → WAKING\n");
      mock_reset_clock();
      offsets_init();
      servos_attach_at(REST_POSE);
      gait_init();

      // Drive to RESTING
      mock_advance_clock(SOFTSTART_DURATION_MS + 10);
      lifecycle_update(millis());
      mock_advance_clock(IDLE_TIMEOUT_MS + 10);
      lifecycle_update(millis());
      mock_advance_clock(SHUTDOWN_RAMP_MS + REST_SETTLE_MS + 10);
      lifecycle_update(millis());
      check(lifecycle_current() == LifecycleState::RESTING, "precondition: RESTING");

      lifecycle_cmd_wake();
      lifecycle_update(millis());

      check(lifecycle_current() == LifecycleState::WAKING, "RESTING + wake → WAKING");
      check(servos_active(), "servos active after wake from RESTING");
  }

  int main() {
      printf("=== lifecycle tests ===\n");
      test_init_starts_waking();
      test_waking_to_idle();
      test_wake_during_waking_goes_active();
      test_wake_from_idle();
      test_idle_timeout_sleeps();
      test_sleep_from_active();
      test_sleeping_to_resting();
      test_wake_from_resting();
      printf("\n%d passed, %d failed\n", g_pass, g_fail);
      return g_fail > 0 ? 1 : 0;
  }
  ```

- [ ] **Step 2: Run test — verify it fails (lifecycle symbols undefined)**

  ```bash
  pio test -e native -f test_lifecycle
  ```
  Expected: compile error — `lifecycle_current`, `LifecycleState`, etc. not defined.

- [ ] **Step 3: Add lifecycle declarations to gait.h**

  In `firmware/include/gait.h`, add after the `GaitState` enum:

  ```cpp
  enum class LifecycleState {
      BOOTING,   // setup() — not yet attached
      WAKING,    // ramping REST_POSE → STANDING_POSE
      IDLE,      // standing, no operator; 10s countdown to sleep
      ACTIVE,    // operator has control lock
      SLEEPING,  // ramping current → REST_POSE
      RESTING    // servos detached
  };

  // Initialise lifecycle state machine. Call from gait_init().
  // Pre-condition: servos_attach_at(REST_POSE) has been called.
  void lifecycle_init(unsigned long now_ms);

  // Drive the lifecycle state machine. Call every main loop iteration.
  void lifecycle_update(unsigned long now_ms);

  // Host acquired the control lock.
  void lifecycle_cmd_wake();

  // Host released the control lock.
  void lifecycle_cmd_sleep();

  // Safety: heartbeat lost — force SLEEPING from any state.
  void lifecycle_heartbeat_lost();

  LifecycleState lifecycle_current();
  const char*    lifecycle_state_name();  // "waking", "idle", etc.
  bool           lifecycle_can_command(); // true only when ACTIVE
  ```

  Also update `gait_init()` signature in gait.h:
  ```cpp
  void gait_init(unsigned long now_ms = 0);
  ```

- [ ] **Step 4: Implement lifecycle state machine in gait.cpp**

  Add at the top of `firmware/src/gait.cpp`, after the existing static variables:

  ```cpp
  // ---- Lifecycle state machine ----
  static LifecycleState s_lifecycle = LifecycleState::BOOTING;
  static bool           s_lifecycle_wake_to_active = false;
  static unsigned long  s_lifecycle_ramp_start = 0;
  static unsigned long  s_lifecycle_idle_start = 0;
  static uint16_t       s_lifecycle_ramp_from[8] = {};

  static void lifecycle_capture_current() {
      for (int i = 0; i < 8; i++) {
          uint16_t r = servo_read_us(i);
          s_lifecycle_ramp_from[i] = (r > 0) ? r : STANDING_POSE[i];
      }
  }

  void lifecycle_init(unsigned long now_ms) {
      s_lifecycle = LifecycleState::WAKING;
      s_lifecycle_wake_to_active = false;
      s_lifecycle_ramp_start = now_ms;
      for (int i = 0; i < 8; i++) {
          s_lifecycle_ramp_from[i] = REST_POSE[i];
      }
      s_lifecycle_idle_start = 0;
  }

  void lifecycle_cmd_wake() {
      switch (s_lifecycle) {
          case LifecycleState::IDLE:
              s_lifecycle = LifecycleState::ACTIVE;
              gait_set_state(GaitState::STAND);
              break;
          case LifecycleState::WAKING:
              s_lifecycle_wake_to_active = true;
              break;
          case LifecycleState::RESTING:
              servos_attach_at(REST_POSE);
              lifecycle_capture_current();
              s_lifecycle_ramp_start = millis();
              s_lifecycle_wake_to_active = true;
              s_lifecycle = LifecycleState::WAKING;
              break;
          case LifecycleState::SLEEPING:
              // Reverse: go back to waking from current position
              lifecycle_capture_current();
              s_lifecycle_ramp_start = millis();
              s_lifecycle_wake_to_active = true;
              s_lifecycle = LifecycleState::WAKING;
              break;
          default:
              break;  // ACTIVE, BOOTING: no-op
      }
  }

  void lifecycle_cmd_sleep() {
      if (s_lifecycle == LifecycleState::ACTIVE) {
          s_lifecycle = LifecycleState::IDLE;
          s_lifecycle_idle_start = millis();
          gait_set_state(GaitState::STAND);
      }
  }

  void lifecycle_heartbeat_lost() {
      if (s_lifecycle == LifecycleState::ACTIVE ||
          s_lifecycle == LifecycleState::IDLE) {
          lifecycle_capture_current();
          s_lifecycle_ramp_start = millis();
          s_lifecycle = LifecycleState::SLEEPING;
          gait_set_state(GaitState::STOP);
      }
  }

  LifecycleState lifecycle_current() { return s_lifecycle; }

  bool lifecycle_can_command() { return s_lifecycle == LifecycleState::ACTIVE; }

  const char* lifecycle_state_name() {
      switch (s_lifecycle) {
          case LifecycleState::BOOTING:  return "booting";
          case LifecycleState::WAKING:   return "waking";
          case LifecycleState::IDLE:     return "idle";
          case LifecycleState::ACTIVE:   return "active";
          case LifecycleState::SLEEPING: return "sleeping";
          case LifecycleState::RESTING:  return "resting";
          default:                       return "unknown";
      }
  }

  void lifecycle_update(unsigned long now_ms) {
      switch (s_lifecycle) {

          case LifecycleState::WAKING: {
              unsigned long elapsed = now_ms - s_lifecycle_ramp_start;
              float t = (SOFTSTART_DURATION_MS > 0)
                        ? fminf((float)elapsed / SOFTSTART_DURATION_MS, 1.0f)
                        : 1.0f;
              // Interpolate REST → STANDING
              for (int i = 0; i < 8; i++) {
                  int16_t s   = (int16_t)s_lifecycle_ramp_from[i];
                  int16_t e   = (int16_t)STANDING_POSE[i];
                  uint16_t pos = (uint16_t)(s + (int16_t)((float)(e - s) * t));
                  servo_write_us(i, apply_offset(i, pos));
              }
              if (elapsed >= SOFTSTART_DURATION_MS) {
                  s_lifecycle = s_lifecycle_wake_to_active
                                ? LifecycleState::ACTIVE
                                : LifecycleState::IDLE;
                  s_lifecycle_wake_to_active = false;
                  if (s_lifecycle == LifecycleState::IDLE) {
                      s_lifecycle_idle_start = now_ms;
                  } else {
                      gait_set_state(GaitState::STAND);
                  }
              }
              break;
          }

          case LifecycleState::IDLE: {
              if (now_ms - s_lifecycle_idle_start >= IDLE_TIMEOUT_MS) {
                  lifecycle_capture_current();
                  s_lifecycle_ramp_start = now_ms;
                  s_lifecycle = LifecycleState::SLEEPING;
                  gait_set_state(GaitState::STOP);
              }
              break;
          }

          case LifecycleState::SLEEPING: {
              unsigned long elapsed = now_ms - s_lifecycle_ramp_start;
              float t = (SHUTDOWN_RAMP_MS > 0)
                        ? fminf((float)elapsed / SHUTDOWN_RAMP_MS, 1.0f)
                        : 1.0f;
              // Interpolate current → REST_POSE
              for (int i = 0; i < 8; i++) {
                  int16_t s   = (int16_t)s_lifecycle_ramp_from[i];
                  int16_t e   = (int16_t)REST_POSE[i];
                  uint16_t pos = (uint16_t)(s + (int16_t)((float)(e - s) * t));
                  servo_write_us(i, pos);
              }
              if (elapsed >= SHUTDOWN_RAMP_MS + REST_SETTLE_MS) {
                  servos_detach_all();
                  s_lifecycle = LifecycleState::RESTING;
              }
              break;
          }

          default:
              break;  // RESTING, ACTIVE, BOOTING: no timer action
      }
  }
  ```

  Update `gait_init()` to call `lifecycle_init()`:
  ```cpp
  void gait_init(unsigned long now_ms) {
      s_state = GaitState::STOP;   // changed from STAND — lifecycle drives standing
      s_speed = 0.0f;
      s_phase = 0.0f;
      s_last_update = now_ms;
      s_last_active = now_ms;
      s_idle_detached = false;
      s_current_transform = {};
      s_target_transform  = {};

      BalanceConfig bcfg = {
          0.3f, 0.0f, 0.05f,
          0.3f, 0.0f, 0.05f,
          8.0f, 0.5f
      };
      balance_init(bcfg);
      lifecycle_init(now_ms);
  }
  ```

  Remove the old idle-detach block from `gait_update()` (lines 116-122 in original) — lifecycle now handles this:
  ```cpp
  // DELETE these lines from gait_update():
  if (!s_idle_detached && servos_active()
      && s_state == GaitState::STOP
      && (now_ms - s_last_active > SERVO_IDLE_TIMEOUT_MS)) {
      servos_detach_all();
      s_idle_detached = true;
      return;
  }
  ```

  Also remove the `s_idle_detached` re-attach from `gait_set_state()` (lines 70-73):
  ```cpp
  // DELETE this block from gait_set_state():
  if (s_idle_detached) {
      servos_init();
      s_idle_detached = false;
  }
  ```

- [ ] **Step 5: Run tests — verify they pass**

  ```bash
  pio test -e native -f test_lifecycle
  pio test -e native -f test_servos
  ```
  Expected: all tests pass.

- [ ] **Step 6: Commit**

  ```bash
  git add firmware/include/gait.h firmware/src/gait.cpp firmware/test/test_lifecycle.cpp
  git commit -m "feat(firmware): add LifecycleState machine to gait — non-blocking WAKING/SLEEPING ramps"
  ```

---

## Task 4: Protocol constants and command handlers for wake/sleep

**Files:** Modify: `firmware/include/protocol.h`, `firmware/include/command_handlers.h`, `firmware/src/command_handlers.cpp`

- [ ] **Step 1: Add protocol constants**

  In `firmware/include/protocol.h`, add after `MSG_CMD_SHUTDOWN`:

  ```cpp
  constexpr const char* MSG_CMD_WAKE     = "cmd_wake";
  constexpr const char* MSG_CMD_SLEEP    = "cmd_sleep";
  ```

- [ ] **Step 2: Add lifecycle accessor to command_handlers.h**

  In `firmware/include/command_handlers.h`, add:

  ```cpp
  // True when lifecycle is ACTIVE — motion commands are accepted.
  bool handlers_lifecycle_active();
  ```

- [ ] **Step 3: Implement wake/sleep handlers and motion command gate**

  In `firmware/src/command_handlers.cpp`:

  Add accessor:
  ```cpp
  bool handlers_lifecycle_active() { return lifecycle_can_command(); }
  ```

  Add handlers before the dispatch table:
  ```cpp
  static void handle_cmd_wake(const JsonDocument&) {
      lifecycle_cmd_wake();
      send_ack(MSG_CMD_WAKE, true);
  }

  static void handle_cmd_sleep(const JsonDocument&) {
      lifecycle_cmd_sleep();
      send_ack(MSG_CMD_SLEEP, true);
  }
  ```

  Gate `handle_cmd_move()` — add at the start of the function body:
  ```cpp
  static void handle_cmd_move(const JsonDocument& doc) {
      if (!lifecycle_can_command()) {
          send_ack(MSG_CMD_MOVE, false, "not_active");
          return;
      }
      s_manual_servo_mode = false;
      // ... rest of existing code unchanged
  ```

  Gate `handle_cmd_stand()`:
  ```cpp
  static void handle_cmd_stand(const JsonDocument&) {
      if (!lifecycle_can_command()) {
          send_ack(MSG_CMD_STAND, false, "not_active");
          return;
      }
      s_manual_servo_mode = false;
      gait_set_state(GaitState::STAND);
      send_ack(MSG_CMD_STAND, true);
  }
  ```

  Add `gait.h` include at top of command_handlers.cpp (it's needed for `lifecycle_can_command()`):
  ```cpp
  #include "gait.h"   // already present — no change needed
  ```

  Add entries to `k_handlers[]` dispatch table:
  ```cpp
  { MSG_CMD_WAKE,         handle_cmd_wake         },
  { MSG_CMD_SLEEP,        handle_cmd_sleep        },
  ```

  Also update `handle_cmd_shutdown()` to use the renamed function:
  ```cpp
  static void handle_cmd_shutdown(const JsonDocument&) {
      s_manual_servo_mode = false;
      s_test_mode = false;
      servos_set_frail(false);
      gait_set_state(GaitState::STOP);
      bool ok = servos_shutdown_to_rest();   // renamed from servos_shutdown_to_lying_down
      send_ack(MSG_CMD_SHUTDOWN, ok);
  }
  ```

- [ ] **Step 4: Build firmware — verify no errors**

  ```bash
  cd /Users/gwen/workspace/bark-buddy
  pio run -e esp32dev 2>&1 | tail -20
  ```
  Expected: `SUCCESS` with no errors.

- [ ] **Step 5: Run all firmware tests**

  ```bash
  pio test -e native
  ```
  Expected: all test suites pass.

- [ ] **Step 6: Commit**

  ```bash
  git add firmware/include/protocol.h firmware/include/command_handlers.h firmware/src/command_handlers.cpp
  git commit -m "feat(firmware): add cmd_wake/cmd_sleep handlers, gate motion commands to ACTIVE lifecycle"
  ```

---

## Task 5: Update main.cpp — new boot sequence and lifecycle integration

**Files:** Modify: `firmware/src/main.cpp`

- [ ] **Step 1: Refactor setup() and update heartbeat**

  Replace `setup()` (lines 91-122 in original) with:

  ```cpp
  void setup() {
      Serial.begin(SERIAL_BAUD);
      delay(100);

      sensor_task_start();
      offsets_init();

      // Attach servos at rest pose — no ramp. Dog must already be lying down.
      bool servos_ok = servos_attach_at(REST_POSE);
      delay(BOOT_SETTLE_MS);

      // gait_init() starts lifecycle in WAKING state (REST → STANDING ramp begins)
      unsigned long now = millis();
      gait_init(now);
      handlers_init();

  #if WIFI_ENABLED
      WiFi.mode(WIFI_STA);
      WiFi.begin(WIFI_SSID, WIFI_PASS);
  #endif

      last_msg_received = now;

      SensorSnapshot snap;
      sensor_snapshot_get(snap);
      JsonDocument doc;
      doc["type"]          = "boot";
      doc["imu"]           = snap.imu_ok;
      doc["sonar"]         = snap.sonar_ok;
      doc["servos"]        = servos_ok;
      doc["pins_verified"] = (bool)PINS_VERIFIED;
      doc["lifecycle"]     = lifecycle_state_name();
      send_json(doc);
  }
  ```

  Replace the heartbeat watchdog block (lines 168-174 in original):

  ```cpp
  // Heartbeat watchdog — safety net if host vanishes
  if (connected && (now - last_msg_received > HEARTBEAT_TIMEOUT_MS)) {
      connected = false;
      lifecycle_heartbeat_lost();  // non-blocking: sets lifecycle to SLEEPING
      sensor_led_set(1, LED_R_LAVENDER, LED_G_LAVENDER, LED_B_LAVENDER);
      sensor_led_set(2, LED_R_LAVENDER, LED_G_LAVENDER, LED_B_LAVENDER);
  }
  ```

- [ ] **Step 2: Add lifecycle_update() call in main loop**

  In `loop()`, add `lifecycle_update(now)` call alongside `gait_update(now)`. Replace the gait tick block (lines 261-264) with:

  ```cpp
  // Lifecycle + gait tick
  if (now - last_gait >= 1000 / GAIT_UPDATE_HZ) {
      if (!low_battery) {
          lifecycle_update(now);
          if (!handlers_manual_servo_mode()) {
              // Only run gait when ACTIVE; lifecycle drives servos in WAKING/SLEEPING
              if (lifecycle_can_command()) {
                  gait_update(now);
              }
          }
      }
      last_gait = now;
  }
  ```

- [ ] **Step 3: Add lifecycle field to telem_status**

  In `loop()`, update the status streaming block (lines 231-247):

  ```cpp
  if (connected && now - last_status >= 1000 / TELEM_STATUS_HZ) {
      JsonDocument doc;
      doc["type"]        = MSG_TELEM_STATUS;
      doc["mode"]        = "idle";
      doc["lifecycle"]   = lifecycle_state_name();
      doc["balance"]     = balance_is_enabled();
      doc["servos"]      = servos_active();
      doc["low_battery"] = low_battery;
  #if WIFI_ENABLED
      doc["wifi"] = wifi_connected;
      if (wifi_connected) {
          doc["wifi_ip"]  = WiFi.localIP().toString();
          doc["tcp_port"] = WIFI_TCP_PORT;
      }
  #endif
      send_json(doc);
      last_status = now;
  }
  ```

- [ ] **Step 4: Build firmware**

  ```bash
  pio run -e esp32dev 2>&1 | tail -20
  ```
  Expected: `SUCCESS`.

- [ ] **Step 5: Run all firmware tests**

  ```bash
  pio test -e native
  ```
  Expected: all suites pass.

- [ ] **Step 6: Commit**

  ```bash
  git add firmware/src/main.cpp
  git commit -m "feat(firmware): new boot sequence — attach at REST_POSE, lifecycle_update in loop, heartbeat uses lifecycle"
  ```

---

## Task 6: Host comms.py — send_json() on Transport, wake()/sleep() on DogComms

**Files:** Modify: `host/comms.py`

- [ ] **Step 1: Add send_json() to Transport ABC**

  In `host/comms.py`, add the following method to the `Transport` class, after `exec_repl()`:

  ```python
  async def send_json(self, msg: dict) -> None:
      """Send a raw JSON command. No-op for transports that don't support JSON."""
      pass
  ```

- [ ] **Step 2: Add wake() and sleep() to DogComms**

  In `host/comms.py`, add after the `stand()` method in `DogComms`:

  ```python
  async def wake(self) -> None:
      """Notify firmware that an operator has acquired control."""
      await self._transport.send_json({"type": "cmd_wake"})

  async def sleep(self) -> None:
      """Notify firmware that the operator has released control."""
      await self._transport.send_json({"type": "cmd_sleep"})
  ```

- [ ] **Step 3: Verify json_transport_base.py already has send_json()**

  `JsonStreamTransport` already has `async def send_json(self, msg: dict)` at line 129. It calls `self._send_json(msg)` which writes JSON to the stream. No change needed — the ABC default is a no-op, and `JsonStreamTransport` overrides it correctly.

- [ ] **Step 4: Commit**

  ```bash
  git add host/comms.py
  git commit -m "feat(host): add Transport.send_json(), DogComms.wake()/sleep() for lifecycle commands"
  ```

---

## Task 7: json_transport_base.py — parse lifecycle from telem_status

**Files:** Modify: `host/json_transport_base.py`

- [ ] **Step 1: Add lifecycle cache field**

  In `JsonStreamTransport.__init__()`, add after `self._firmware_info = {}`:

  ```python
  self._lifecycle = "unknown"  # cached lifecycle state from telem_status
  ```

- [ ] **Step 2: Parse lifecycle in _handle_telem()**

  In `_handle_telem()`, update the `telem_status` branch:

  ```python
  elif msg_type == "telem_status":
      if msg.get("lifecycle"):
          self._lifecycle = msg["lifecycle"]
      if msg.get("wifi") and msg.get("wifi_ip"):
          self._firmware_info["wifi_ip"] = msg["wifi_ip"]
          self._firmware_info["tcp_port"] = msg.get("tcp_port", 9000)
  ```

- [ ] **Step 3: Add public accessor**

  After the `get_battery_mv()` method, add:

  ```python
  def get_lifecycle(self) -> str:
      """Current lifecycle state from firmware telemetry cache."""
      return self._lifecycle
  ```

- [ ] **Step 4: Commit**

  ```bash
  git add host/json_transport_base.py
  git commit -m "feat(host): parse lifecycle field from telem_status in firmware transport"
  ```

---

## Task 8: server.py — lock bridge, broadcast lifecycle, remove REPL idle hack

**Files:** Modify: `host/server.py`

- [ ] **Step 1: Add lifecycle tracking field in __init__()**

  In `Server.__init__()`, after `self._servos_idle = False`, add:

  ```python
  self._lifecycle = "unknown"   # last known lifecycle from firmware
  ```

  Remove these fields (no longer needed):
  ```python
  # DELETE: self._last_motion_time = 0.0
  # DELETE: self._servos_idle = False
  # DELETE: self._servo_idle_timeout = 30.0
  ```

- [ ] **Step 2: Add _acquire_lock() and _release_lock() helpers**

  Add two new coroutines after `_broadcast_lock_status()`:

  ```python
  async def _acquire_lock(self, ws, name: str) -> None:
      """Set lock holder and notify firmware to wake."""
      self._lock_holder = ws
      self._lock_name = name
      self._lock_time = _time.monotonic()
      await self._broadcast_lock_status()
      await self._dog.wake()

  async def _release_lock(self) -> None:
      """Clear lock holder and notify firmware to sleep."""
      self._lock_holder = None
      self._lock_name = ""
      await self._broadcast_lock_status()
      await self._dog.sleep()
  ```

- [ ] **Step 3: Update all lock acquisition points to use _acquire_lock()**

  In `_handle_ws_message()`:

  Replace the `cmd_lock` handler block (lines 441-447):
  ```python
  if msg_type == "cmd_lock":
      name = msg.get("name", "Anonymous")
      if self._can_control(ws):
          await self._acquire_lock(ws, name)
      elif self._lock_holder is not None:
          # ... challenge code unchanged
  ```

  Replace the auto-acquire block (lines 486-490):
  ```python
  if self._lock_holder is None:
      await self._acquire_lock(ws, self._client_names.get(ws, "Operator"))
  ```

  Replace all `_lock_holder = None` + `_lock_name = ""` + `broadcast_lock_status` patterns (cmd_unlock, cmd_lock_yield, client disconnect at lines 461-465, 470-474, 416-419):
  ```python
  # cmd_unlock:
  if self._lock_holder is ws:
      await self._release_lock()

  # cmd_lock_yield:
  if self._lock_holder is ws:
      await self._release_lock()

  # client disconnect:
  if self._lock_holder is ws:
      await self._release_lock()
  ```

- [ ] **Step 4: Handle lock timeout — track state change in telemetry loop**

  Keep `_check_lock_timeout()` unchanged (sync, called from `_is_locked_by()`, `_is_locked()`, `_can_control()`, `_lock_status_msg()` — prevents stale-lock issues for challenger clients).

  In `Server.__init__()`, add:
  ```python
  self._prev_lock_held = False
  ```

  In the telemetry loop, add a check after the battery block to detect when the lock auto-releases and fire `dog.sleep()`:

  ```python
  # Detect lock auto-release via timeout → fire sleep
  was_locked = self._prev_lock_held
  self._check_lock_timeout()   # sync release if timed out
  now_locked = self._lock_holder is not None
  self._prev_lock_held = now_locked
  if was_locked and not now_locked:
      await self._broadcast_lock_status()
      await self._dog.sleep()
  ```

- [ ] **Step 5: Remove the old REPL idle timeout hack**

  Delete lines 749-763 (the servo idle timeout block):
  ```python
  # DELETE this entire block:
  # Servo idle timeout — lie down to rest (PWM stays alive for wake-up)
  if (self._last_motion_time > 0
          and not self._servos_idle
          and self._motion == "stop"
          and now - self._last_motion_time > self._servo_idle_timeout):
      self._servos_idle = True
      try:
          for _ in range(5):
              await self._transport.exec_repl(
                  "_dog.transform([0, 0, -1], [0, 0, 0], 80)")
          logger.info("Servo idle timeout — resting")
      except NotImplementedError:
          pass
      except Exception:
          pass
  ```

  Also remove the wake-from-idle REPL call in the `cmd_move` handler (lines 501-506):
  ```python
  # DELETE:
  if self._servos_idle and direction != "stop":
      try:
          await self._transport.exec_repl("_dog.set_default_pose()")
      except Exception:
          pass
      self._servos_idle = False
  ```

- [ ] **Step 6: Parse and broadcast lifecycle from telem_status**

  In the `_telemetry_loop`, find where firmware telemetry is read and broadcast `telem_status` to browsers. Update `_broadcast_status()` to include lifecycle:

  ```python
  async def _broadcast_status(self, battery_mv=None):
      lifecycle = getattr(self._transport, 'get_lifecycle', lambda: 'unknown')()
      status = {
          "type": "telem_status",
          "mode": self._mode,
          "lifecycle": lifecycle,
          "balance": self._balance.enabled,
          "fallen": self._balance.fallen,
          "connected": self._dog.connected,
          "scanning": self._scan.running,
          "transport": self._transport_label,
      }
      if battery_mv is not None:
          status["battery_mv"] = battery_mv
      # ... rest unchanged
      await self._broadcast(status)
  ```

  Also include lifecycle in the initial status message sent in `_ws_handler()`:
  ```python
  lifecycle = getattr(self._transport, 'get_lifecycle', lambda: 'unknown')()
  status = {
      "type": "telem_status",
      "mode": self._mode,
      "lifecycle": lifecycle,
      "balance": self._balance.enabled,
      # ... rest unchanged
  }
  ```

- [ ] **Step 7: Commit**

  ```bash
  git add host/server.py
  git commit -m "feat(host): wire control lock to lifecycle wake/sleep, remove REPL idle hack, broadcast lifecycle state"
  ```

---

## Task 9: Sim transport — lifecycle state machine

**Files:** Modify: `host/sim/sim_transport.py`

- [ ] **Step 1: Add lifecycle state and send_json() to SimTransport**

  In `SimTransport.__init__()`, add after `self._motion_cmd = 1`:

  ```python
  # Lifecycle state machine
  self._lifecycle = "waking"          # mirrors LifecycleState names
  self._lifecycle_wake_to_active = False
  self._lifecycle_ramp_start = 0.0    # time.monotonic()
  self._lifecycle_idle_start = 0.0
  ```

  Add `send_json()` to `SimTransport`:

  ```python
  async def send_json(self, msg: dict) -> None:
      """Handle lifecycle JSON commands from host."""
      msg_type = msg.get("type")
      if msg_type == "cmd_wake":
          self._sim_lifecycle_wake()
      elif msg_type == "cmd_sleep":
          self._sim_lifecycle_sleep()
  ```

- [ ] **Step 2: Implement sim lifecycle helpers**

  Add private methods to `SimTransport`:

  ```python
  def _sim_lifecycle_wake(self):
      if self._lifecycle in ("resting", "sleeping"):
          self._lifecycle_ramp_start = time.monotonic()
          self._lifecycle_wake_to_active = True
          self._lifecycle = "waking"
          self._physics.set_lifecycle_target("standing")
      elif self._lifecycle == "waking":
          self._lifecycle_wake_to_active = True
      elif self._lifecycle == "idle":
          self._lifecycle = "active"

  def _sim_lifecycle_sleep(self):
      if self._lifecycle == "active":
          self._lifecycle = "idle"
          self._lifecycle_idle_start = time.monotonic()

  def _sim_lifecycle_update(self):
      """Called from _sim_loop each tick."""
      now = time.monotonic()
      if self._lifecycle == "waking":
          elapsed = now - self._lifecycle_ramp_start
          if elapsed >= 2.0:  # SOFTSTART_DURATION_MS / 1000
              self._lifecycle = "active" if self._lifecycle_wake_to_active else "idle"
              self._lifecycle_wake_to_active = False
              if self._lifecycle == "idle":
                  self._lifecycle_idle_start = now
      elif self._lifecycle == "idle":
          if now - self._lifecycle_idle_start >= 10.0:  # IDLE_TIMEOUT_MS / 1000
              self._lifecycle = "sleeping"
              self._lifecycle_ramp_start = now
              self._physics.set_lifecycle_target("rest")
      elif self._lifecycle == "sleeping":
          elapsed = now - self._lifecycle_ramp_start
          if elapsed >= 1.5 + 0.5:  # SHUTDOWN_RAMP_MS + REST_SETTLE_MS ms
              self._lifecycle = "resting"

  def get_lifecycle(self) -> str:
      return self._lifecycle
  ```

  Call `_sim_lifecycle_update()` in `_sim_loop()`, after the physics step:

  ```python
  async def _sim_loop(self):
      imu_interval = 1.0 / 50
      sonar_interval = 1.0 / 20
      last_sonar = 0.0
      steps_per_tick = max(1, int(imu_interval * self._speed_factor / SIM_TIMESTEP))
      try:
          while self._open:
              for _ in range(steps_per_tick):
                  self._physics.step(SIM_TIMESTEP)
                  self._physics.apply_movement_force(self._motion_cmd, SIM_TIMESTEP)
              self._update_imu()
              self._sim_lifecycle_update()   # ← add this line
              now = time.monotonic()
              if now - last_sonar >= sonar_interval / self._speed_factor:
                  self._update_sonar()
                  last_sonar = now
              await asyncio.sleep(imu_interval / self._speed_factor)
      except asyncio.CancelledError:
          pass
  ```

  Initialize `_lifecycle_ramp_start` in `open()`, after the settle loop:

  ```python
  self._lifecycle_ramp_start = time.monotonic()  # boot waking starts now
  self._open = True
  ```

- [ ] **Step 3: Gate motion commands in _process_cmd()**

  In `_process_cmd()`, add lifecycle guard for motion (func == "3"):

  ```python
  elif func == "3":
      if self._lifecycle != "active":
          return "CMD|3|NOT_ACTIVE|$"
      self._motion_cmd = int(parts[1]) if len(parts) > 1 else 1
      self._physics.set_motion(self._motion_cmd)
      return "CMD|3|OK|$"
  ```

- [ ] **Step 4: Commit**

  ```bash
  git add host/sim/sim_transport.py
  git commit -m "feat(sim): add lifecycle state machine mirroring firmware, gate motion commands to active state"
  ```

---

## Task 10: physics.py — REST_HIP/REST_KNEE and set_lifecycle_target()

**Files:** Modify: `host/sim/physics.py`

- [ ] **Step 1: Add REST_HIP / REST_KNEE constants**

  In `host/sim/physics.py`, after the `STAND_HIP` / `STAND_KNEE` lines (line 38-39), add:

  ```python
  # Rest pose (radians) — hips back, knees tucked. Must match firmware REST_POSE geometry.
  # Computed from: FL_hip=1800µs. Standing=2096, center=1500. Range ≈ ±600µs ≈ ±30deg.
  # (1800-2096)/(2096-1500) * 30deg ≈ -14.8deg from standing; standing=30deg → rest≈15deg
  REST_HIP  = 0.262   # ~15 degrees
  REST_KNEE = -0.349  # ~-20 degrees (less tucked than standing -35deg)
  ```

  Note: Update these values once actual REST_POSE hardware values are tuned in Task 1.

- [ ] **Step 2: Add set_lifecycle_target() to DogPhysics**

  In `DogPhysics`, add after `set_motion()`:

  ```python
  def set_lifecycle_target(self, target: str) -> None:
      """Set leg targets for lifecycle transitions ('standing' or 'rest')."""
      if target == "standing":
          for leg in self.legs:
              leg.hip_target = STAND_HIP
              leg.knee_target = STAND_KNEE
      elif target == "rest":
          for leg in self.legs:
              leg.hip_target = REST_HIP
              leg.knee_target = REST_KNEE
  ```

- [ ] **Step 3: Commit**

  ```bash
  git add host/sim/physics.py
  git commit -m "feat(sim): add REST_HIP/REST_KNEE constants and set_lifecycle_target() for rest/stand transitions"
  ```

---

## Task 11: Web UI — lifecycle status indicator

**Files:** Modify: `web/dog3d/overlay.js`

- [ ] **Step 1: Add lifecycle state tracking in overlay**

  In `web/dog3d/overlay.js`, find the `updateOverlay()` or equivalent function that handles `telem_status`. Add lifecycle state tracking.

  First, check the current structure:
  ```bash
  grep -n "lifecycle\|telem_status\|mode" /Users/gwen/workspace/bark-buddy/web/dog3d/overlay.js | head -30
  ```

- [ ] **Step 2: Display lifecycle in status panel**

  Add a lifecycle indicator to the overlay. In the function that processes `telem_status` messages, handle `lifecycle`:

  ```javascript
  // In the telem_status handler:
  if (msg.lifecycle !== undefined) {
      updateLifecycleIndicator(msg.lifecycle);
  }
  ```

  Add `updateLifecycleIndicator()`:

  ```javascript
  function updateLifecycleIndicator(state) {
      const el = document.getElementById('lifecycle-state');
      if (!el) return;
      const labels = {
          waking:   'WAKING UP',
          idle:     'IDLE',
          active:   'ACTIVE',
          sleeping: 'SLEEPING',
          resting:  'RESTING',
          booting:  'BOOTING',
          unknown:  '',
      };
      el.textContent = labels[state] || state.toUpperCase();
      el.dataset.state = state;
  }
  ```

- [ ] **Step 3: Add lifecycle element to overlay HTML in index.html**

  In `web/index.html`, add inside the telem-overlay div (find it by looking for `id="telem-overlay"`):

  ```html
  <div id="lifecycle-state" class="lifecycle-badge" data-state="unknown"></div>
  ```

- [ ] **Step 4: Add lifecycle CSS to style.css**

  In `web/style.css`, add:

  ```css
  .lifecycle-badge {
      font-size: 10px;
      font-weight: bold;
      letter-spacing: 0.08em;
      padding: 2px 6px;
      border-radius: 3px;
      background: rgba(255,255,255,0.08);
      color: #aaa;
  }
  .lifecycle-badge[data-state="active"] { color: #4f4; background: rgba(0,255,0,0.1); }
  .lifecycle-badge[data-state="idle"]   { color: #fa0; background: rgba(255,160,0,0.1); }
  .lifecycle-badge[data-state="waking"] { color: #4af; background: rgba(64,160,255,0.1); }
  .lifecycle-badge[data-state="sleeping"],
  .lifecycle-badge[data-state="resting"] { color: #888; background: rgba(128,128,128,0.08); }
  ```

- [ ] **Step 5: Disable D-pad when not active**

  In `web/modules/controls.js`, find the D-pad send logic. Add lifecycle gating:

  ```javascript
  // At the top of controls.js or in the module init, track lifecycle:
  let currentLifecycle = 'unknown';

  // In the ws message handler, update it:
  if (msg.type === 'telem_status' && msg.lifecycle) {
      currentLifecycle = msg.lifecycle;
      const dpad = document.getElementById('dpad');
      if (dpad) {
          dpad.classList.toggle('dpad-disabled', currentLifecycle !== 'active');
      }
  }

  // In the button send handler, guard motion commands:
  function sendMove(direction) {
      if (currentLifecycle !== 'active') return;
      ws.send(JSON.stringify({ type: 'cmd_move', direction }));
  }
  ```

  Add CSS for disabled D-pad in `style.css`:
  ```css
  .dpad-disabled { opacity: 0.35; pointer-events: none; }
  ```

- [ ] **Step 6: Start dev server and test lifecycle UI**

  ```bash
  cd /Users/gwen/workspace/bark-buddy
  python host/server.py --sim
  ```

  Open browser at `http://localhost:8080`. Verify:
  - Lifecycle badge shows "WAKING UP" then "IDLE" after 2 seconds
  - D-pad is disabled/dimmed in IDLE state
  - Acquiring control lock → badge turns green "ACTIVE", D-pad enables
  - Releasing lock → badge turns amber "IDLE"
  - After 10s → "SLEEPING" then "RESTING"

- [ ] **Step 7: Commit**

  ```bash
  git add web/dog3d/overlay.js web/modules/controls.js web/index.html web/style.css
  git commit -m "feat(web): add lifecycle state badge and disable D-pad controls when not active"
  ```

---

## Verification Checklist

After all tasks are complete:

- [ ] **Boot sequence:** Flash firmware. Dog should: attach at rest pose → ramp to standing over 2s → IDLE state (amber LED or similar indicator). Confirm in host logs: `lifecycle: "waking"` → `"idle"`.

- [ ] **Lock cycle:** Open browser, acquire lock → dog stays standing (ACTIVE). Release lock → wait 10s → dog ramps to rest pose → servos detach.

- [ ] **Re-wake:** With dog resting, acquire lock in browser → firmware re-attaches at REST_POSE → ramps to standing → ACTIVE.

- [ ] **Heartbeat safety:** Acquire lock. Kill host process (`Ctrl+C`). Within 5s, dog should ramp to rest and detach (heartbeat timeout triggers SLEEPING).

- [ ] **Sim parity:** Run `python host/server.py --sim`. Lifecycle badge transitions correctly. After 2s → IDLE. Acquire lock → ACTIVE. Sim 3D view responds to D-pad. Release lock → IDLE → RESTING.

- [ ] **All firmware tests pass:** `pio test -e native`

- [ ] **Motion commands rejected when not active:** Send `cmd_move` via browser without holding control lock. Expect `lock_denied` message (server gate) or firmware returns `{"ok": false, "reason": "not_active"}` (firmware gate).

- [ ] **Rapid toggle:** Acquire/release lock rapidly 5 times. No servo snapping. Smooth ramp direction changes each time.
