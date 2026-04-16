// test_lifecycle.cpp — Host-side tests for the lifecycle state machine.
// Validates: state transitions, idle timeout, ramp reversal, heartbeat safety.

#include "mock_arduino.h"
#include "mock_preferences.h"
#include "../src/servos.cpp"
#include "../src/offsets.cpp"
#include "../src/balance.cpp"
#include "../src/gait.cpp"
#include <stdio.h>
#include <string.h>

static int g_pass = 0, g_fail = 0;
static void check(bool cond, const char* label) {
    if (cond) { printf("  PASS: %s\n", label); g_pass++; }
    else       { printf("  FAIL: %s\n", label); g_fail++; }
}

// Helper: drive from scratch to ACTIVE state using mock clock.
// Returns the timestamp at which ACTIVE was entered.
static unsigned long reach_active(unsigned long start_ms) {
    _mock_millis = start_ms;
    servos_detach_all();
    lifecycle_init(start_ms);
    // BOOTING → force to SLEEPING via heartbeat_lost
    lifecycle_heartbeat_lost(start_ms);
    // Advance past SLEEPING ramp + settle → RESTING
    unsigned long t = start_ms + SHUTDOWN_RAMP_MS + REST_SETTLE_MS + 1;
    _mock_millis = t;
    lifecycle_update(t);
    // RESTING → cmd_wake → WAKING
    lifecycle_cmd_wake(t);
    // Advance past WAKING ramp → ACTIVE
    t += SOFTSTART_DURATION_MS + 1;
    _mock_millis = t;
    lifecycle_update(t);
    return t;
}

// Test 1: lifecycle_init → BOOTING state, cannot command
static void test_init_is_booting() {
    printf("test_init_is_booting\n");
    mock_reset_clock();
    servos_detach_all();
    lifecycle_init(0);
    check(lifecycle_current() == LifecycleState::BOOTING, "init → BOOTING");
    check(strcmp(lifecycle_state_name(), "booting") == 0, "name = 'booting'");
    check(!lifecycle_can_command(), "cannot command in BOOTING");
}

// Test 2: RESTING → cmd_wake → WAKING → ACTIVE
static void test_resting_to_active_via_wake() {
    printf("test_resting_to_active_via_wake\n");
    mock_reset_clock();
    reach_active(0);
    check(lifecycle_current() == LifecycleState::ACTIVE, "ACTIVE after wake sequence");
    check(lifecycle_can_command(), "can command in ACTIVE");
    check(strcmp(lifecycle_state_name(), "active") == 0, "name = 'active'");
}

// Test 3: ACTIVE → cmd_sleep → IDLE
static void test_active_to_idle_via_sleep() {
    printf("test_active_to_idle_via_sleep\n");
    mock_reset_clock();
    unsigned long t = reach_active(0);
    lifecycle_cmd_sleep(t);
    check(lifecycle_current() == LifecycleState::IDLE, "cmd_sleep → IDLE");
    check(!lifecycle_can_command(), "cannot command in IDLE");
    check(strcmp(lifecycle_state_name(), "idle") == 0, "name = 'idle'");
}

// Test 4: IDLE → timeout → SLEEPING
static void test_idle_timeout_to_sleeping() {
    printf("test_idle_timeout_to_sleeping\n");
    mock_reset_clock();
    unsigned long t = reach_active(0);
    lifecycle_cmd_sleep(t);
    check(lifecycle_current() == LifecycleState::IDLE, "in IDLE");
    // Before timeout: still IDLE
    lifecycle_update(t + IDLE_TIMEOUT_MS - 1);
    check(lifecycle_current() == LifecycleState::IDLE, "still IDLE before timeout");
    // After timeout: SLEEPING
    lifecycle_update(t + IDLE_TIMEOUT_MS + 1);
    check(lifecycle_current() == LifecycleState::SLEEPING, "IDLE timeout → SLEEPING");
}

// Test 5: SLEEPING → ramp+settle → RESTING, servos detached
static void test_sleeping_to_resting() {
    printf("test_sleeping_to_resting\n");
    mock_reset_clock();
    unsigned long t = reach_active(0);
    lifecycle_cmd_sleep(t);
    lifecycle_update(t + IDLE_TIMEOUT_MS + 1);  // → SLEEPING
    unsigned long ramp_start = t + IDLE_TIMEOUT_MS + 1;
    check(lifecycle_current() == LifecycleState::SLEEPING, "in SLEEPING");
    // Before ramp+settle done: still SLEEPING
    lifecycle_update(ramp_start + SHUTDOWN_RAMP_MS);
    check(lifecycle_current() == LifecycleState::SLEEPING, "still SLEEPING during ramp+settle");
    // After ramp+settle: RESTING
    lifecycle_update(ramp_start + SHUTDOWN_RAMP_MS + REST_SETTLE_MS + 1);
    check(lifecycle_current() == LifecycleState::RESTING, "SLEEPING → RESTING");
    check(!servos_active(), "servos detached in RESTING");
}

// Test 6: cmd_wake during SLEEPING reverses ramp → WAKING → ACTIVE
static void test_wake_during_sleeping_reverses_ramp() {
    printf("test_wake_during_sleeping_reverses_ramp\n");
    mock_reset_clock();
    unsigned long t = reach_active(0);
    lifecycle_cmd_sleep(t);
    lifecycle_update(t + IDLE_TIMEOUT_MS + 1);  // → SLEEPING
    unsigned long ramp_start = t + IDLE_TIMEOUT_MS + 1;
    // Interrupt with cmd_wake mid-ramp
    unsigned long mid = ramp_start + SHUTDOWN_RAMP_MS / 2;
    lifecycle_cmd_wake(mid);
    check(lifecycle_current() == LifecycleState::WAKING, "cmd_wake during SLEEPING → WAKING");
    // Advance past waking ramp → ACTIVE
    lifecycle_update(mid + SOFTSTART_DURATION_MS + 1);
    check(lifecycle_current() == LifecycleState::ACTIVE, "WAKING → ACTIVE");
}

// Test 7: heartbeat_lost from ACTIVE → SLEEPING, cannot command
static void test_heartbeat_lost_forces_sleeping() {
    printf("test_heartbeat_lost_forces_sleeping\n");
    mock_reset_clock();
    unsigned long t = reach_active(0);
    check(lifecycle_current() == LifecycleState::ACTIVE, "ACTIVE");
    lifecycle_heartbeat_lost(t);
    check(lifecycle_current() == LifecycleState::SLEEPING, "heartbeat_lost → SLEEPING");
    check(!lifecycle_can_command(), "cannot command after heartbeat loss");
}

// Test 8: cmd_wake from IDLE → ACTIVE immediately (no ramp needed)
static void test_cmd_wake_from_idle_no_ramp() {
    printf("test_cmd_wake_from_idle_no_ramp\n");
    mock_reset_clock();
    unsigned long t = reach_active(0);
    lifecycle_cmd_sleep(t);  // → IDLE
    check(lifecycle_current() == LifecycleState::IDLE, "in IDLE");
    // cmd_wake from IDLE should go straight to ACTIVE (already standing)
    lifecycle_cmd_wake(t + 1);
    check(lifecycle_current() == LifecycleState::ACTIVE, "cmd_wake from IDLE → ACTIVE immediately");
    check(lifecycle_can_command(), "can command after wake from IDLE");
}

// Test 9: double cmd_sleep is a no-op
static void test_duplicate_sleep_is_noop() {
    printf("test_duplicate_sleep_is_noop\n");
    mock_reset_clock();
    unsigned long t = reach_active(0);
    lifecycle_cmd_sleep(t);  // → IDLE
    lifecycle_cmd_sleep(t);  // second call, no-op
    check(lifecycle_current() == LifecycleState::IDLE, "double cmd_sleep stays IDLE");
}

// Helper: drive to RESTING state using mock clock.
static unsigned long reach_resting(unsigned long start_ms) {
    _mock_millis = start_ms;
    servos_detach_all();
    lifecycle_init(start_ms);
    lifecycle_heartbeat_lost(start_ms);
    unsigned long t = start_ms + SHUTDOWN_RAMP_MS + REST_SETTLE_MS + 1;
    _mock_millis = t;
    lifecycle_update(t);
    return t;
}

// Test 10: auto-wake cmd_move from RESTING — queued MOVE executes on ACTIVE entry
static void test_auto_wake_cmd_move_from_resting() {
    printf("test_auto_wake_cmd_move_from_resting\n");
    mock_reset_clock();
    unsigned long t = reach_resting(0);
    check(lifecycle_current() == LifecycleState::RESTING, "in RESTING");

    // Queue a MOVE command and trigger wake
    PendingCmd cmd;
    cmd.type = PendingCmdType::MOVE;
    cmd.gait_state = GaitState::WALK_FORWARD;
    cmd.speed = 0.8f;
    lifecycle_set_pending(cmd);
    check(lifecycle_has_pending(), "pending command stored");

    lifecycle_cmd_wake(t);
    check(lifecycle_current() == LifecycleState::WAKING, "RESTING→WAKING after cmd_wake");

    // Advance past 2s ramp — pending command should execute at ACTIVE entry
    t += SOFTSTART_DURATION_MS + 1;
    _mock_millis = t;
    lifecycle_update(t);

    check(lifecycle_current() == LifecycleState::ACTIVE, "WAKING→ACTIVE after ramp");
    check(!lifecycle_has_pending(), "pending command cleared after execution");
    check(gait_current_state() == GaitState::WALK_FORWARD, "queued MOVE executed on ACTIVE entry");
}

// Test 11: auto-wake cmd_stand from RESTING — queued STAND executes on ACTIVE entry
static void test_auto_wake_cmd_stand_from_resting() {
    printf("test_auto_wake_cmd_stand_from_resting\n");
    mock_reset_clock();
    unsigned long t = reach_resting(0);
    check(lifecycle_current() == LifecycleState::RESTING, "in RESTING");

    PendingCmd cmd;
    cmd.type = PendingCmdType::STAND;
    lifecycle_set_pending(cmd);
    lifecycle_cmd_wake(t);
    check(lifecycle_current() == LifecycleState::WAKING, "RESTING→WAKING");

    t += SOFTSTART_DURATION_MS + 1;
    _mock_millis = t;
    lifecycle_update(t);

    check(lifecycle_current() == LifecycleState::ACTIVE, "WAKING→ACTIVE");
    check(!lifecycle_has_pending(), "pending command cleared");
    check(gait_current_state() == GaitState::STAND, "queued STAND executed on ACTIVE entry");
}

// Test 12: auto-wake from IDLE executes pending command immediately (no ramp)
static void test_auto_wake_from_idle_executes_immediately() {
    printf("test_auto_wake_from_idle_executes_immediately\n");
    mock_reset_clock();
    unsigned long t = reach_active(0);
    lifecycle_cmd_sleep(t);  // → IDLE
    check(lifecycle_current() == LifecycleState::IDLE, "in IDLE");

    PendingCmd cmd;
    cmd.type = PendingCmdType::MOVE;
    cmd.gait_state = GaitState::TURN_RIGHT;
    cmd.speed = 1.0f;
    lifecycle_set_pending(cmd);

    // Wake from IDLE — transitions instantly, should execute pending immediately
    lifecycle_cmd_wake(t + 1);

    check(lifecycle_current() == LifecycleState::ACTIVE, "IDLE→ACTIVE immediately");
    check(!lifecycle_has_pending(), "pending command cleared on instant ACTIVE");
    check(gait_current_state() == GaitState::TURN_RIGHT, "queued MOVE executed on instant ACTIVE");
}

// Test 13: pending command is overwritten by a second set — only last executes
static void test_pending_command_overwritten() {
    printf("test_pending_command_overwritten\n");
    mock_reset_clock();
    unsigned long t = reach_resting(0);

    // First pending: MOVE forward
    PendingCmd cmd1;
    cmd1.type = PendingCmdType::MOVE;
    cmd1.gait_state = GaitState::WALK_FORWARD;
    cmd1.speed = 1.0f;
    lifecycle_set_pending(cmd1);

    // Overwrite with STAND
    PendingCmd cmd2;
    cmd2.type = PendingCmdType::STAND;
    lifecycle_set_pending(cmd2);

    check(lifecycle_has_pending(), "pending command stored");

    lifecycle_cmd_wake(t);
    t += SOFTSTART_DURATION_MS + 1;
    _mock_millis = t;
    lifecycle_update(t);

    check(lifecycle_current() == LifecycleState::ACTIVE, "ACTIVE");
    check(!lifecycle_has_pending(), "pending cleared");
    // Should be STAND (the overwrite), not WALK_FORWARD
    check(gait_current_state() == GaitState::STAND, "only last pending command executed");
}

int main() {
    printf("=== lifecycle tests ===\n\n");

    test_init_is_booting();
    test_resting_to_active_via_wake();
    test_active_to_idle_via_sleep();
    test_idle_timeout_to_sleeping();
    test_sleeping_to_resting();
    test_wake_during_sleeping_reverses_ramp();
    test_heartbeat_lost_forces_sleeping();
    test_cmd_wake_from_idle_no_ramp();
    test_duplicate_sleep_is_noop();
    test_auto_wake_cmd_move_from_resting();
    test_auto_wake_cmd_stand_from_resting();
    test_auto_wake_from_idle_executes_immediately();
    test_pending_command_overwritten();

    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail > 0 ? 1 : 0;
}
