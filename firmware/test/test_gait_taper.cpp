// test_gait_taper.cpp — Walk→stand return taper integration tests
// Build: make -C firmware/test test_gait_taper
// Run:   firmware/test/test_gait_taper
//
// Compiles the real gait.cpp with stubbed dependencies.
// Uses mock_arduino.h's controllable clock to drive gait_update ticks.

#include "mock_arduino.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>

// ---- Stubs for gait.cpp dependencies ----

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

#include "../include/balance.h"
static bool s_balance_enabled = false;
bool         balance_is_enabled()                            { return s_balance_enabled; }
BodyPose     balance_update(float, float, float)             { return BodyPose{}; }
void         balance_init(const BalanceConfig&)              {}
void         balance_reset()                                 {}
void         balance_enable(bool en)                         { s_balance_enabled = en; }
void         balance_set_gains(float,float,float,float,float,float) {}
void         balance_get_gains(float*,float*,float*,float*,float*,float*) {}

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
#include "../src/gait.cpp"

// ---- Test harness ----
static int g_pass = 0;
static int g_fail = 0;

static void check(bool cond, const char* name) {
    printf("{\"test\":\"%s\",\"pass\":%s}\n", name, cond ? "true" : "false");
    cond ? g_pass++ : g_fail++;
}

static void reset_all() {
    mock_reset_clock();
    memset(s_servo_us, 0, sizeof(s_servo_us));
    s_engaged         = true;
    s_ramping         = false;
    s_balance_enabled = false;
    gait_init(0);
}

static void walk_for_ms(int duration_ms) {
    gait_set_state(GaitState::WALK_FORWARD, 1.0f);
    for (int elapsed = 0; elapsed < duration_ms; elapsed += 10) {
        mock_advance_ms(10);
        gait_update(millis());
    }
}

static void get_stand_target(uint16_t target[8]) {
    BodyPose zero = {};
    body_pose_to_pulses(zero, target);
    // apply_offset is pass-through in this test, so no additional transform needed
}

static void tick_ms(int duration_ms) {
    for (int elapsed = 0; elapsed < duration_ms; elapsed += 10) {
        mock_advance_ms(10);
        gait_update(millis());
    }
}

// ============================================================
// Test 1: walk→STAND does not snap to stand on the first tick
// ============================================================
static void test_no_snap() {
    reset_all();
    walk_for_ms(250);  // phase ≈ 2.36 rad — feet well displaced from standing

    uint16_t walk_pos[8];
    for (int i = 0; i < 8; i++) walk_pos[i] = s_servo_us[i];

    uint16_t stand_target[8];
    get_stand_target(stand_target);

    gait_set_state(GaitState::STAND, 0.0f);
    mock_advance_ms(5);  // t ≈ 5/600 — smoothstep ≈ 0 — servo should stay near walk_pos
    gait_update(millis());

    bool any_displaced = false;
    bool no_snap = true;
    for (int i = 0; i < 8; i++) {
        if (walk_pos[i] != stand_target[i]) {
            any_displaced = true;
            if (s_servo_us[i] == stand_target[i]) {
                no_snap = false;
                printf("  servo %d snapped: walk=%u stand=%u\n", i, walk_pos[i], stand_target[i]);
            }
        }
    }
    check(any_displaced,      "taper_walk_displaces_servos");
    check(any_displaced && no_snap, "taper_no_snap_on_first_tick");
}

// ============================================================
// Test 2: taper approaches stand target monotonically
//         Also verifies STAND→STAND mid-taper is idempotent (no non-monotonic jump)
// ============================================================
static void test_monotonic() {
    reset_all();
    walk_for_ms(250);

    uint16_t walk_pos[8];
    for (int i = 0; i < 8; i++) walk_pos[i] = s_servo_us[i];

    uint16_t stand_target[8];
    get_stand_target(stand_target);

    gait_set_state(GaitState::STAND, 0.0f);

    static uint16_t seq[8][80];
    int ticks = 0;
    for (int ms = 10; ms <= STAND_RETURN_MS + 20 && ticks < 80; ms += 10) {
        mock_advance_ms(10);
        gait_update(millis());
        for (int i = 0; i < 8; i++) seq[i][ticks] = s_servo_us[i];
        ticks++;

        // Re-call STAND mid-taper — must not cause a backward jump
        if (ms == STAND_RETURN_MS / 2) {
            gait_set_state(GaitState::STAND, 0.0f);
        }
    }

    bool all_monotonic = true;
    for (int i = 0; i < 8; i++) {
        if (walk_pos[i] == stand_target[i]) continue;
        bool going_up = (stand_target[i] > walk_pos[i]);
        for (int t = 1; t < ticks; t++) {
            // Allow ±1 for integer rounding in the lerp
            bool regressed = going_up  ? (seq[i][t] + 1 < seq[i][t-1])
                                       : (seq[i][t]     > seq[i][t-1] + 1);
            if (regressed) {
                all_monotonic = false;
                printf("  servo %d tick %d regressed: %u→%u target=%u\n",
                       i, t, seq[i][t-1], seq[i][t], stand_target[i]);
                break;
            }
        }
    }
    check(all_monotonic, "taper_monotonic");
}

// ============================================================
// Test 3: taper reaches exact stand target after STAND_RETURN_MS
// ============================================================
static void test_endpoint() {
    reset_all();
    walk_for_ms(250);

    uint16_t stand_target[8];
    get_stand_target(stand_target);

    gait_set_state(GaitState::STAND, 0.0f);
    tick_ms(STAND_RETURN_MS + 50);  // past the taper window

    bool exact = true;
    for (int i = 0; i < 8; i++) {
        if (s_servo_us[i] != stand_target[i]) {
            exact = false;
            printf("  servo %d: got %u, want %u\n", i, s_servo_us[i], stand_target[i]);
        }
    }
    check(exact, "taper_endpoint_exact");
}

// ============================================================
// Test 4: STAND→STAND after taper completes stays at stand target
// ============================================================
static void test_stand_idempotent() {
    reset_all();
    walk_for_ms(250);

    uint16_t stand_target[8];
    get_stand_target(stand_target);

    gait_set_state(GaitState::STAND, 0.0f);
    tick_ms(STAND_RETURN_MS + 50);  // taper complete — at stand target

    // Re-call STAND (already in STAND)
    gait_set_state(GaitState::STAND, 0.0f);
    mock_advance_ms(10);
    gait_update(millis());

    bool still_at_stand = true;
    for (int i = 0; i < 8; i++) {
        if (s_servo_us[i] != stand_target[i]) {
            still_at_stand = false;
            printf("  servo %d moved after re-STAND: %u (want %u)\n",
                   i, s_servo_us[i], stand_target[i]);
        }
    }
    check(still_at_stand, "taper_stand_idempotent");
}

// ============================================================
// Test 5: walk→STOP uses the same taper (not only STAND)
// ============================================================
static void test_stop_path() {
    reset_all();
    walk_for_ms(250);

    uint16_t walk_pos[8];
    for (int i = 0; i < 8; i++) walk_pos[i] = s_servo_us[i];

    uint16_t stand_target[8];
    get_stand_target(stand_target);

    gait_set_state(GaitState::STOP, 0.0f);

    mock_advance_ms(5);
    gait_update(millis());

    bool any_displaced = false;
    bool no_snap = true;
    for (int i = 0; i < 8; i++) {
        if (walk_pos[i] != stand_target[i]) {
            any_displaced = true;
            if (s_servo_us[i] == stand_target[i]) no_snap = false;
        }
    }
    check(any_displaced && no_snap, "taper_stop_no_snap");

    tick_ms(STAND_RETURN_MS + 50);

    bool exact = true;
    for (int i = 0; i < 8; i++) {
        if (s_servo_us[i] != stand_target[i]) { exact = false; break; }
    }
    check(exact, "taper_stop_endpoint");
}

// ============================================================
// Test 6: C2 — TURN_LEFT→BACKWARD reversal is deferred
// ============================================================
static void test_turn_to_backward_deferred() {
    reset_all();
    walk_for_ms(200);

    gait_set_state(GaitState::TURN_LEFT, 1.0f);
    mock_advance_ms(10);
    gait_update(millis());
    // Speed ramp may not have fully reached 1.0 yet; set directly via another set_state
    // Just verify: TURN_LEFT → WALK_BACKWARD triggers the deferred path (state stays TURN_LEFT).
    gait_set_state(GaitState::WALK_BACKWARD, 1.0f);
    check(gait_current_state() == GaitState::TURN_LEFT, "reversal_turn_to_backward_deferred");

    // After coasting ~400ms, the pending state should apply
    for (int ms = 0; ms < 500; ms += 10) {
        mock_advance_ms(10);
        gait_update(millis());
    }
    check(gait_current_state() == GaitState::WALK_BACKWARD, "reversal_turn_to_backward_applied");
}

// ============================================================
// Test 7: C1 — balance re-enabled after tilt fault re-arm
// ============================================================
static void test_tilt_rearm_restores_balance() {
    reset_all();
    s_balance_enabled = true;

    // Trigger tilt fault
    gait_update_imu(60.0f, 0.0f);
    mock_advance_ms(10);
    gait_update(millis());
    check(!s_balance_enabled, "tilt_fault_disables_balance");

    // Untilt but hold hasn't elapsed yet
    gait_update_imu(0.0f, 0.0f);
    mock_advance_ms(500);
    gait_update(millis());
    check(!s_balance_enabled, "tilt_hold_still_blocking_balance");

    // Advance past BALANCE_TILT_HOLD_MS (1000ms)
    mock_advance_ms(600);
    gait_update(millis());
    check(s_balance_enabled, "tilt_rearm_restores_balance");
}

// ============================================================
// Test 8: C4 — tilt at boot (now_ms=0) holds for full 1s
// ============================================================
static void test_tilt_at_boot_holds() {
    mock_reset_clock();  // millis() = 0
    s_balance_enabled = true;
    s_engaged = true;
    s_ramping = false;
    gait_init(0);

    // Trigger tilt at t=0
    gait_update_imu(60.0f, 0.0f);
    gait_update(0);
    check(!s_balance_enabled, "tilt_boot_fault_triggered");

    // Untilt at t=500ms — hold not elapsed
    gait_update_imu(0.0f, 0.0f);
    gait_update(500);
    check(!s_balance_enabled, "tilt_boot_hold_blocks_rearm_at_500ms");

    // t=1001ms — hold elapsed, re-arm
    gait_update(1001);
    check(s_balance_enabled, "tilt_boot_rearms_after_hold");
}

// ============================================================
// Test 9: Q3 — gait_init uses its now_ms parameter
// ============================================================
static void test_gait_init_uses_param() {
    mock_reset_clock();  // millis() = 0
    s_engaged = true;
    s_ramping = false;
    s_balance_enabled = false;
    memset(s_servo_us, 0, sizeof(s_servo_us));

    // Init with now_ms=5000 (millis() is still 0)
    gait_init(5000);
    gait_set_state(GaitState::WALK_FORWARD, 1.0f);

    // gait_update at 5010 → dt should be 10ms (not 5010ms which would be > 0.5s guard)
    gait_update(5010);
    // Phase should have advanced (servos written); if dt was wrong it would be dropped
    bool phase_advanced = false;
    for (int i = 0; i < 8; i++) {
        if (s_servo_us[i] != 0) { phase_advanced = true; break; }
    }
    check(phase_advanced, "gait_init_uses_now_ms_param");
}

int main() {
    test_no_snap();
    test_monotonic();
    test_endpoint();
    test_stand_idempotent();
    test_stop_path();
    test_turn_to_backward_deferred();
    test_tilt_rearm_restores_balance();
    test_tilt_at_boot_holds();
    test_gait_init_uses_param();

    printf("{\"summary\":\"gait_taper_tests\",\"pass\":%d,\"fail\":%d}\n", g_pass, g_fail);
    return g_fail > 0 ? 1 : 0;
}
