// test_gait_ik.cpp — Tests for gait_tick_ik() and the IK gait pipeline
// Build: make -C firmware/test test_gait_ik
// Run:   firmware/test/test_gait_ik

#include "mock_arduino.h"
#include "../include/ik.h"
#include "../include/body_transform.h"
#include "../include/gait_math.h"
#include <cstdio>
#include <cmath>

static int pass = 0;
static int fail = 0;

static void check(bool cond, const char* name) {
    printf("{\"test\":\"%s\",\"pass\":%s}\n", name, cond ? "true" : "false");
    cond ? pass++ : fail++;
}

// ============================================================
// Test 1: Foot clearance — during swing phase, at least one leg has z > 0
// ============================================================
static void test_foot_clearance() {
    GaitConfig cfg = { 20.0f, 15.0f, 1.5f };
    bool any_lift_found = false;
    int steps = 360;

    for (int i = 0; i < steps; i++) {
        float phase = (float)i / steps * 2.0f * (float)M_PI;
        GaitFootOffsets offs = gait_tick_ik(phase, GaitDir::FORWARD, cfg, 1.0f);

        for (int leg = 0; leg < 4; leg++) {
            if (offs.feet[leg].z > 0.0f) {
                any_lift_found = true;
            }
        }
    }

    check(any_lift_found, "foot_clearance_swing_lifts");

    // Verify that z is never negative (feet don't go below standing plane)
    bool no_negative_z = true;
    for (int i = 0; i < steps; i++) {
        float phase = (float)i / steps * 2.0f * (float)M_PI;
        GaitFootOffsets offs = gait_tick_ik(phase, GaitDir::FORWARD, cfg, 1.0f);
        for (int leg = 0; leg < 4; leg++) {
            if (offs.feet[leg].z < -0.001f) {
                no_negative_z = false;
            }
        }
    }
    check(no_negative_z, "foot_clearance_no_negative_z");

    // Verify that max lift is approximately stride_height_mm (within 1mm)
    float max_z = 0.0f;
    for (int i = 0; i < steps; i++) {
        float phase = (float)i / steps * 2.0f * (float)M_PI;
        GaitFootOffsets offs = gait_tick_ik(phase, GaitDir::FORWARD, cfg, 1.0f);
        for (int leg = 0; leg < 4; leg++) {
            if (offs.feet[leg].z > max_z) max_z = offs.feet[leg].z;
        }
    }
    bool lift_height_ok = (max_z > cfg.stride_height_mm * 0.95f &&
                           max_z <= cfg.stride_height_mm + 0.001f);
    printf("{\"test\":\"foot_clearance_max_lift\",\"max_z_mm\":%.2f,"
           "\"expected_mm\":%.1f,\"pass\":%s}\n",
           max_z, cfg.stride_height_mm, lift_height_ok ? "true" : "false");
    lift_height_ok ? pass++ : fail++;
}

// ============================================================
// Test 2: Diagonal pairing — FL+RR in phase, FR+RL anti-phase
// Verify: when FL is in swing (z > 0), RR is also in swing;
//         when FR is in swing (z > 0), RL is also in swing.
//         And FL/FR are never simultaneously in swing.
// ============================================================
static void test_diagonal_pairing() {
    GaitConfig cfg = { 20.0f, 15.0f, 1.5f };
    int steps = 1000;
    bool fl_rr_same = true;   // FL and RR should always have same z sign
    bool fr_rl_same = true;   // FR and RL should always have same z sign
    bool diag_opposite = true; // FL and FR should never both be lifting

    for (int i = 0; i < steps; i++) {
        float phase = (float)i / steps * 2.0f * (float)M_PI;
        GaitFootOffsets offs = gait_tick_ik(phase, GaitDir::FORWARD, cfg, 1.0f);

        // FL=0, FR=1, RL=2, RR=3
        bool fl_up = offs.feet[0].z > 0.5f;
        bool fr_up = offs.feet[1].z > 0.5f;
        bool rl_up = offs.feet[2].z > 0.5f;
        bool rr_up = offs.feet[3].z > 0.5f;

        // FL and RR should match
        if (fl_up != rr_up) fl_rr_same = false;
        // FR and RL should match
        if (fr_up != rl_up) fr_rl_same = false;
        // FL and FR should not both be significantly lifted (diagonal pairs)
        if (fl_up && fr_up) diag_opposite = false;
    }

    check(fl_rr_same,    "diagonal_pairing_FL_RR_inphase");
    check(fr_rl_same,    "diagonal_pairing_FR_RL_inphase");
    check(diag_opposite, "diagonal_pairing_no_simultaneous_swing");
}

// ============================================================
// Test 3: Full pipeline round-trip — standing_foot_pos + gait_offset → IK
// For forward walk across a full phase cycle, all 4 legs should produce
// valid IK solutions.
// ============================================================
static void test_pipeline_roundtrip() {
    GaitConfig cfg = { 20.0f, 15.0f, 1.5f };
    int steps = 360;
    int total_ik = 0;
    int valid_ik = 0;

    for (int i = 0; i < steps; i++) {
        float phase = (float)i / steps * 2.0f * (float)M_PI;
        GaitFootOffsets offs = gait_tick_ik(phase, GaitDir::FORWARD, cfg, 1.0f);

        for (int leg = 0; leg < 4; leg++) {
            FootPos foot = standing_foot_pos(leg);
            foot.x += offs.feet[leg].x;
            foot.z += offs.feet[leg].z;

            uint16_t hip_us, knee_us;
            total_ik++;
            if (foot_to_pulses(leg, foot, hip_us, knee_us)) {
                valid_ik++;
            }
        }
    }

    float pct = total_ik > 0 ? (float)valid_ik / total_ik * 100.0f : 0.0f;
    bool ok = (valid_ik == total_ik);
    printf("{\"test\":\"pipeline_roundtrip\",\"valid\":%d,\"total\":%d,"
           "\"pct\":%.1f,\"pass\":%s}\n",
           valid_ik, total_ik, pct, ok ? "true" : "false");
    ok ? pass++ : fail++;
}

// ============================================================
// Test 4: Speed scaling — at speed 0.5 stride should be halved
// ============================================================
static void test_speed_scaling() {
    GaitConfig cfg = { 20.0f, 15.0f, 1.5f };

    // Find max x displacement at speed 1.0
    float max_x_full = 0.0f;
    float max_x_half = 0.0f;
    int steps = 360;

    for (int i = 0; i < steps; i++) {
        float phase = (float)i / steps * 2.0f * (float)M_PI;
        GaitFootOffsets offs_full = gait_tick_ik(phase, GaitDir::FORWARD, cfg, 1.0f);
        GaitFootOffsets offs_half = gait_tick_ik(phase, GaitDir::FORWARD, cfg, 0.5f);

        for (int leg = 0; leg < 4; leg++) {
            float ax = fabsf(offs_full.feet[leg].x);
            float bx = fabsf(offs_half.feet[leg].x);
            if (ax > max_x_full) max_x_full = ax;
            if (bx > max_x_half) max_x_half = bx;
        }
    }

    // Half speed should produce roughly half the stride (within 10%)
    float ratio = (max_x_full > 0.001f) ? max_x_half / max_x_full : 0.0f;
    bool ok = (ratio > 0.45f && ratio < 0.55f);
    printf("{\"test\":\"speed_scaling\",\"ratio\":%.3f,\"pass\":%s}\n",
           ratio, ok ? "true" : "false");
    ok ? pass++ : fail++;
}

// ============================================================
// Test 5: Turning differential — turn_left slows left legs (0.3×)
// ============================================================
static void test_turning_differential() {
    GaitConfig cfg = { 20.0f, 15.0f, 1.5f };
    int steps = 360;

    float max_fl = 0.0f, max_fr = 0.0f;

    for (int i = 0; i < steps; i++) {
        float phase = (float)i / steps * 2.0f * (float)M_PI;
        GaitFootOffsets offs = gait_tick_ik(phase, GaitDir::TURN_LEFT, cfg, 1.0f);

        float afl = fabsf(offs.feet[0].x);  // FL — left side, should be slow
        float afr = fabsf(offs.feet[1].x);  // FR — right side, should be fast
        if (afl > max_fl) max_fl = afl;
        if (afr > max_fr) max_fr = afr;
    }

    // Left side should be ~0.3× right side
    bool ok = (max_fl < max_fr * 0.5f);
    printf("{\"test\":\"turning_differential\",\"max_fl_mm\":%.2f,\"max_fr_mm\":%.2f,"
           "\"ratio\":%.3f,\"pass\":%s}\n",
           max_fl, max_fr, (max_fr > 0.001f ? max_fl / max_fr : 0.0f),
           ok ? "true" : "false");
    ok ? pass++ : fail++;
}

// ============================================================
// Test 6: Zero speed produces zero offsets
// ============================================================
static void test_zero_speed() {
    GaitConfig cfg = { 20.0f, 15.0f, 1.5f };
    bool all_zero = true;
    int steps = 360;

    for (int i = 0; i < steps; i++) {
        float phase = (float)i / steps * 2.0f * (float)M_PI;
        GaitFootOffsets offs = gait_tick_ik(phase, GaitDir::FORWARD, cfg, 0.0f);

        for (int leg = 0; leg < 4; leg++) {
            // x offsets should be zero (sl = stride_length * 0 = 0)
            // z offsets can still be non-zero (sh is not scaled by speed)
            if (fabsf(offs.feet[leg].x) > 0.001f) {
                all_zero = false;
            }
        }
    }
    check(all_zero, "zero_speed_no_x_offset");
}

// ============================================================
// Test 7: smoothstep — clamped, midpoint, monotonic, zero-velocity endpoints
// ============================================================
static void test_smoothstep() {
    check(smoothstep(0.0f) == 0.0f,  "smoothstep_zero");
    check(smoothstep(1.0f) == 1.0f,  "smoothstep_one");
    check(fabsf(smoothstep(0.5f) - 0.5f) < 1e-6f, "smoothstep_midpoint");
    check(smoothstep(-0.5f) == 0.0f, "smoothstep_clamp_below");
    check(smoothstep(1.5f)  == 1.0f, "smoothstep_clamp_above");

    bool monotonic = true;
    float prev = 0.0f;
    for (int i = 0; i <= 100; i++) {
        float v = smoothstep((float)i / 100.0f);
        if (v < prev - 1e-7f) { monotonic = false; break; }
        prev = v;
    }
    check(monotonic, "smoothstep_monotonic");

    // Derivative = 6t(1-t) → zero at t=0 and t=1; values near endpoints should be tiny
    check(smoothstep(0.01f) < 0.001f,        "smoothstep_zero_velocity_start");
    check(1.0f - smoothstep(0.99f) < 0.001f, "smoothstep_zero_velocity_end");
}

// ============================================================
// Main
// ============================================================
int main() {
    test_foot_clearance();
    test_diagonal_pairing();
    test_pipeline_roundtrip();
    test_speed_scaling();
    test_turning_differential();
    test_zero_speed();
    test_smoothstep();

    printf("{\"summary\":\"gait_ik_tests\",\"pass\":%d,\"fail\":%d}\n", pass, fail);
    return fail > 0 ? 1 : 0;
}
