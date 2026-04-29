// test_gait_tilt_fault.cpp — Tilt-over safety cutoff in gait_update.
//
// Verifies:
//   - high pitch or roll triggers tilt_fault, stops gait, disables balance
//   - the fault re-arms only after BALANCE_TILT_HOLD_MS of below-cutoff readings
//   - balance_was_enabled state restores on re-arm
//
// Compiles real gait.cpp with stubbed deps, same pattern as test_gait_taper.

#include "mock_arduino.h"
#include <cstdio>
#include <cstring>
#include <cmath>

// ---- Servos stub ----
#include "../include/servos.h"
static bool     s_engaged = true;
static bool     s_ramping = false;
static uint16_t s_servo_us[8] = {};
bool         servos_engaged()                  { return s_engaged; }
bool         servos_is_ramping()               { return s_ramping; }
uint16_t     servo_read_us(uint8_t idx)        { return (idx < 8) ? s_servo_us[idx] : 0; }
bool         servo_write_us(uint8_t idx, uint16_t us) { if (idx < 8) s_servo_us[idx] = us; return true; }
bool         servos_engage_start()             { return true; }
void         servos_disengage_start()          {}
RampResult   servos_ramp_tick(uint32_t)        { return RampResult::NONE; }
void         servos_detach_all()               {}
void         servos_set_battery_cutoff()       {}
bool         servos_battery_cutoff()           { return false; }

// ---- Balance stub ----
#include "../include/balance.h"
static bool s_balance_enabled = false;
bool         balance_is_enabled()                       { return s_balance_enabled; }
BodyPose     balance_update(float, float, float)        { return BodyPose{}; }
void         balance_init(const BalanceConfig&)         {}
void         balance_reset()                            {}
void         balance_enable(bool en)                    { s_balance_enabled = en; }
void         balance_set_gains(float,float,float,float,float,float) {}
void         balance_get_gains(float*,float*,float*,float*,float*,float*) {}

// ---- Offsets stub ----
#include "../include/offsets.h"
int16_t      offset_get(uint8_t)               { return 0; }
void         offset_set(uint8_t, int16_t)      {}
void         offsets_save()                    {}
void         offsets_reset()                   {}
void         offsets_init()                    {}
uint16_t     apply_offset(uint8_t, uint16_t us){ return us; }
float        yaw_trim_load()                   { return 0.0f; }
void         yaw_trim_save(float)              {}

#include "../include/gait.h"
#include "../include/config.h"
#include "../src/gait.cpp"

// ---- Harness ----
static int g_pass = 0;
static int g_fail = 0;
static void check(bool cond, const char* msg) {
    if (cond) { printf("  PASS  %s\n", msg); g_pass++; }
    else       { printf("  FAIL  %s\n", msg); g_fail++; }
}

static void reset_all() {
    mock_reset_clock();
    memset(s_servo_us, 0, sizeof(s_servo_us));
    s_engaged = true;
    s_ramping = false;
    s_balance_enabled = false;
    gait_init(0);
}

static void tick_with_imu(unsigned long ms_step, float pitch_deg, float roll_deg) {
    mock_advance_ms(ms_step);
    gait_update_imu(pitch_deg, roll_deg);
    gait_update(millis());
}

static void test_walk_above_cutoff_triggers_fault() {
    printf("\nTest: walking + tilt above cutoff triggers fault\n");
    reset_all();
    balance_enable(true);
    gait_set_state(GaitState::WALK_FORWARD, 1.0f);
    // Build up some walking ticks
    for (int i = 0; i < 5; i++) tick_with_imu(20, 0.0f, 0.0f);
    check(gait_current_state() == GaitState::WALK_FORWARD, "walking before tilt");
    check(s_balance_enabled,                              "balance on before tilt");

    // Tilt past cutoff
    tick_with_imu(20, BALANCE_TILT_CUTOFF_DEG + 1.0f, 0.0f);

    check(gait_current_state() == GaitState::STOP,        "gait stopped on tilt");
    check(!s_balance_enabled,                             "balance disabled on tilt");
}

static void test_walk_blocked_during_hold_window() {
    printf("\nTest: walk command blocked during hold window\n");
    reset_all();
    gait_set_state(GaitState::WALK_FORWARD, 1.0f);
    for (int i = 0; i < 2; i++) tick_with_imu(20, 0.0f, 0.0f);
    tick_with_imu(20, BALANCE_TILT_CUTOFF_DEG + 5.0f, 0.0f);
    check(gait_current_state() == GaitState::STOP, "gait stopped");

    // While still tilted, attempt to walk again — must remain stopped.
    gait_set_state(GaitState::WALK_FORWARD, 1.0f);
    tick_with_imu(20, BALANCE_TILT_CUTOFF_DEG + 5.0f, 0.0f);
    check(gait_current_state() == GaitState::STOP, "still stopped while tilted");

    // Tilt resolved, but hold window not yet elapsed → still blocked.
    gait_set_state(GaitState::WALK_FORWARD, 1.0f);
    tick_with_imu(50, 0.0f, 0.0f);
    check(gait_current_state() == GaitState::STOP, "still blocked during hold");
}

static void test_re_arm_after_hold_restores_balance() {
    printf("\nTest: re-arm after hold ms restores prior balance state\n");
    reset_all();
    balance_enable(true);
    gait_set_state(GaitState::WALK_FORWARD, 1.0f);
    tick_with_imu(20, BALANCE_TILT_CUTOFF_DEG + 5.0f, 0.0f);
    check(!s_balance_enabled, "balance off after fault");

    // Stay below cutoff long enough to re-arm.
    unsigned long elapsed = 0;
    while (elapsed < (unsigned long)BALANCE_TILT_HOLD_MS + 200) {
        tick_with_imu(50, 0.0f, 0.0f);
        elapsed += 50;
    }
    check(s_balance_enabled, "balance re-enabled after hold");
}

static void test_roll_axis_also_triggers() {
    printf("\nTest: roll above cutoff also triggers fault\n");
    reset_all();
    gait_set_state(GaitState::WALK_FORWARD, 1.0f);
    for (int i = 0; i < 3; i++) tick_with_imu(20, 0.0f, 0.0f);
    tick_with_imu(20, 0.0f, -(BALANCE_TILT_CUTOFF_DEG + 5.0f));
    check(gait_current_state() == GaitState::STOP, "stopped on negative roll");
}

int main() {
    test_walk_above_cutoff_triggers_fault();
    test_walk_blocked_during_hold_window();
    test_re_arm_after_hold_restores_balance();
    test_roll_axis_also_triggers();
    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}
