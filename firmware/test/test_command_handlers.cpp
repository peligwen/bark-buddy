// test_command_handlers.cpp — Tests for F3, F4, F9 fixes in command_handlers.cpp
//
// Provides minimal stubs for all command_handlers.cpp dependencies,
// then includes the real implementation to test validation logic.

#include "mock_arduino.h"
#include <ArduinoJson.h>
#include <cstdio>
#include <cstdlib>
#include <cstring>

// physics stub — mock_arduino.h calls physics::on_servo_duty under MOCK_FIRMWARE=1
namespace physics { void on_servo_duty(uint8_t, uint32_t) {} }

// ---- Captured send_ack output ----
static const char* g_last_ref   = nullptr;
static bool        g_last_ok    = true;
static const char* g_last_error = nullptr;

// ---- Gait pause counter ----
static int g_gait_pause_count = 0;

// ---- Stubs for all command_handlers.cpp dependencies ----

// comms.h
#include "../include/comms.h"
void send_json(const JsonDocument&) {}
void send_ack(const char* ref_type, bool ok, const char* error) {
    g_last_ref   = ref_type;
    g_last_ok    = ok;
    g_last_error = error ? error : nullptr;
}
void comms_write_line(const char*) {}
String get_tcp_client_ip() { return String(); }

// comms_out.h
#include "../include/comms_out.h"
void comms_out_init() {}
void comms_emit_json_from_task(const JsonDocument&) {}
void comms_out_drain() {}

// servos.h stubs
#include "../include/servos.h"
uint8_t SERVO_PINS[8] = {25, 26, 27, 14, 16, 17, 4, 2};
bool servos_engaged()              { return true; }   // simulate engaged so handlers run
bool servos_battery_cutoff()       { return false; }
bool servos_is_ramping()           { return false; }
bool servos_engage_start()         { return true; }
void servos_disengage_start()      {}
void servo_write_us(uint8_t, uint16_t) {}
uint16_t servo_read_us(uint8_t)    { return 1500; }
void servos_detach_all()           {}
void servos_set_battery_cutoff()   {}
RampResult servos_ramp_tick(uint32_t) { return RampResult::NONE; }
bool servos_set_pin(uint8_t, uint8_t, String& err) { (void)err; return true; }

#if AUX_SERVOS_ENABLED
void     aux_servo_init()                      {}
void     aux_servo_write_us(uint8_t, uint16_t) {}
uint16_t aux_servo_read_us(uint8_t)            { return 1500; }
#endif

// gait.h stubs
#include "../include/gait.h"
void gait_pause()              { g_gait_pause_count++; }
GaitState gait_current_state() { return GaitState::STOP; }
void gait_set_state(GaitState, float) {}
void gait_update_imu(float, float)    {}
void gait_update(unsigned long)       {}
void gait_init(unsigned long)         {}
void gait_set_config(const GaitConfig&) {}
void gait_set_body_transform(const BodyPose&, uint16_t) {}

// balance.h stubs
#include "../include/balance.h"
bool balance_is_enabled()      { return false; }
void balance_enable(bool)      {}
void balance_reset()           {}
void balance_init(const BalanceConfig&) {}
BodyPose balance_update(float, float, float) { return BodyPose{}; }
void balance_set_gains(float, float, float, float, float, float) {}
void balance_get_gains(float* pkp, float* pki, float* pkd,
                       float* rkp, float* rki, float* rkd) {
    if (pkp) *pkp = 0; if (pki) *pki = 0; if (pkd) *pkd = 0;
    if (rkp) *rkp = 0; if (rki) *rki = 0; if (rkd) *rkd = 0;
}

// offsets.h stubs
#include "../include/offsets.h"
int16_t  offset_get(uint8_t)          { return 0; }
void     offset_set(uint8_t, int16_t) {}
void     offsets_save()               {}
void     offsets_reset()              {}
void     offsets_init()               {}
uint16_t apply_offset(uint8_t, uint16_t us) { return us; }

// sensor_task.h stubs
#include "../include/sensor_task.h"
void sensor_task_start()        {}
void sensor_snapshot_get(SensorSnapshot& s) { s = {}; }
void sensor_led_set(uint8_t, uint8_t, uint8_t, uint8_t) {}
bool sensor_i2c_op(const I2cCmd&, I2cResult& out) { out = {}; out.ok = true; return true; }
bool sensor_i2c_write(uint8_t, uint8_t, uint8_t)  { return true; }
bool sensor_gpio_subscribe(uint8_t, GpioMode)      { return true; }
void sensor_gpio_unsubscribe(uint8_t)              {}
void sensor_imu_signal_ready()                     {}

// pin_registry.h stubs
#include "../include/pin_registry.h"
bool pin_is_reserved(uint8_t, const char** r) { if (r) *r = nullptr; return false; }

// update_led.h — extern bool declared in update_led.h, defined in main.cpp normally
#include "../include/update_led.h"
bool s_update_led_active = false;

// buzzer.h stubs
#include "../include/buzzer.h"
void buzzer_tone(uint16_t, uint32_t) {}
void buzzer_stop()                   {}
void buzzer_init()                   {}

// gpio_aux.h stubs
#include "../include/gpio_aux.h"
bool gpio_aux_allowlisted(uint8_t)          { return true; }
int  gpio_aux_read_digital(uint8_t)         { return 0; }
int  gpio_aux_read_analog(uint8_t)          { return -1; }
void gpio_aux_set_mode(uint8_t, GpioMode)   {}
void gpio_aux_write(uint8_t, uint8_t)       {}

// command_handlers.h (for handlers_init / handle_message declarations)
#include "../include/command_handlers.h"

// Include the real implementation
#include "../src/command_handlers.cpp"

// ---- Test harness ----
static int g_pass = 0;
static int g_fail = 0;

static void check(bool cond, const char* label) {
    if (cond) { printf("  PASS  %s\n", label); g_pass++; }
    else       { printf("  FAIL  %s\n", label); g_fail++; }
}

static void reset() {
    g_last_ref         = nullptr;
    g_last_ok          = true;
    g_last_error       = nullptr;
    g_gait_pause_count = 0;
}

static void dispatch(const char* json_str) {
    JsonDocument doc;
    deserializeJson(doc, json_str);
    handle_message(doc);
}

int main() {
    handlers_init();

    // F3: cmd_i2c with unknown op → ack ok=false, error="unknown_op"
    printf("--- F3: cmd_i2c unknown op ---\n");
    reset();
    dispatch(R"({"type":"cmd_i2c","op":"reed","bus":1,"addr":80})");
    check(g_last_ref   != nullptr,                                       "ref_type set");
    check(!g_last_ok,                                                    "ok=false");
    check(g_last_error && strcmp(g_last_error, "unknown_op") == 0,       "error=unknown_op");

    // F3: cmd_i2c with valid op="scan" should NOT return unknown_op
    printf("--- F3: cmd_i2c valid op=scan ---\n");
    reset();
    dispatch(R"({"type":"cmd_i2c","op":"scan","bus":1})");
    // sensor_i2c_op stub returns ok=true, so this should succeed (no unknown_op error)
    check(g_last_error == nullptr || strcmp(g_last_error, "unknown_op") != 0,
          "valid op=scan not rejected as unknown_op");

    // F4: cmd_servo index=11 → ack ok=false, error="bad_index" (always, regardless of AUX flag)
    printf("--- F4: cmd_servo bad_index (idx=11) ---\n");
    reset();
    dispatch(R"({"type":"cmd_servo","index":11,"pulse_us":1500})");
    check(!g_last_ok,                                                    "ok=false");
    check(g_last_error && strcmp(g_last_error, "bad_index") == 0,        "error=bad_index");

    // F4: cmd_servo index=8 with AUX_SERVOS_ENABLED=0 → ack ok=false, error="aux_disabled"
    printf("--- F4: cmd_servo aux_disabled (idx=8) ---\n");
#if !AUX_SERVOS_ENABLED
    reset();
    dispatch(R"({"type":"cmd_servo","index":8,"pulse_us":1500})");
    check(!g_last_ok,                                                    "ok=false");
    check(g_last_error && strcmp(g_last_error, "aux_disabled") == 0,     "error=aux_disabled");
#else
    printf("  SKIP  (AUX_SERVOS_ENABLED=1, aux_disabled path not reachable)\n");
#endif

    // F4: cmd_servo index=0 (valid main servo) should succeed
    printf("--- F4: cmd_servo valid main servo (idx=0) ---\n");
    reset();
    dispatch(R"({"type":"cmd_servo","index":0,"pulse_us":1500})");
    check(g_last_ok,                                                     "ok=true for valid index");

    // F9: cmd_servo index=8 with AUX_SERVOS_ENABLED=1 → does NOT call gait_pause
    printf("--- F9: aux servo skips gait_pause ---\n");
#if AUX_SERVOS_ENABLED
    reset();
    dispatch(R"({"type":"cmd_servo","index":8,"pulse_us":1500})");
    check(g_gait_pause_count == 0, "gait_pause NOT called for aux servo");
#else
    printf("  SKIP  (AUX_SERVOS_ENABLED=0, aux path not reachable)\n");
#endif

    // F9 control: cmd_servo index=0 (main servo) DOES call gait_pause
    printf("--- F9: main servo calls gait_pause ---\n");
    reset();
    dispatch(R"({"type":"cmd_servo","index":0,"pulse_us":1500})");
    check(g_gait_pause_count == 1, "gait_pause called for main servo");

    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
