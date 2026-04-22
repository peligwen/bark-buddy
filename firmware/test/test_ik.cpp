#include "mock_arduino.h"
#include "../include/ik.h"
#include <cstdio>
#include <cmath>

int main() {
    int pass = 0, fail = 0;

    // Test 1: Standing pose round-trip (pulse -> foot -> IK -> pulse)
    for (int leg = 0; leg < 4; leg++) {
        uint16_t hip_us = STANDING_POSE[leg * 2];
        uint16_t knee_us = STANDING_POSE[leg * 2 + 1];

        FootPos foot = pulses_to_foot(leg, hip_us, knee_us);
        uint16_t hip_out, knee_out;
        bool ok = foot_to_pulses(leg, foot, hip_out, knee_out);

        bool hip_ok = ok && abs((int)hip_out - (int)hip_us) <= 5;
        bool knee_ok = ok && abs((int)knee_out - (int)knee_us) <= 5;

        printf("{\"test\":\"standing_roundtrip\",\"leg\":%d,"
               "\"hip_in\":%d,\"hip_out\":%d,\"knee_in\":%d,\"knee_out\":%d,"
               "\"pass\":%s}\n",
               leg, hip_us, hip_out, knee_us, knee_out,
               (hip_ok && knee_ok) ? "true" : "false");
        (hip_ok && knee_ok) ? pass++ : fail++;
    }

    // Test 2: Stock ground truth — standing foot positions match stock firmware
    const float stock_feet[4][3] = {
        { 59.25f,  46.0f, -80.0f},  // FL: front-left
        { 59.25f, -46.0f, -80.0f},  // FR: front-right
        {-71.25f,  46.0f, -80.0f},  // RL: rear-left
        {-71.25f, -46.0f, -80.0f},  // RR: rear-right
    };
    for (int leg = 0; leg < 4; leg++) {
        FootPos foot = standing_foot_pos(leg);
        float dx = foot.x - stock_feet[leg][0];
        float dy = foot.y - stock_feet[leg][1];
        float dz = foot.z - stock_feet[leg][2];
        float err = sqrtf(dx*dx + dy*dy + dz*dz);
        bool ok = err < 2.0f;  // corrected geometry: expect <0.5mm; 2mm is a generous gate
        printf("{\"test\":\"stock_ground_truth\",\"leg\":%d,"
               "\"expected\":[%.1f,%.1f,%.1f],\"got\":[%.1f,%.1f,%.1f],"
               "\"error_mm\":%.2f,\"pass\":%s}\n",
               leg, stock_feet[leg][0], stock_feet[leg][1], stock_feet[leg][2],
               foot.x, foot.y, foot.z, err, ok ? "true" : "false");
        ok ? pass++ : fail++;
    }

    // Test 3: Reachability — unreachable point fails gracefully
    {
        FootPos unreachable = {0, 0, -200};
        uint16_t h, k;
        bool ok = foot_to_pulses(0, unreachable, h, k);
        bool test_pass = !ok;
        printf("{\"test\":\"unreachable\",\"pass\":%s}\n",
               test_pass ? "true" : "false");
        test_pass ? pass++ : fail++;
    }

    // Test 4: Workspace sweep — FK(IK(point)) == point for reachable points
    int sweep_pass = 0, sweep_total = 0;
    for (float x = -30; x <= 30; x += 10) {
        for (float z = -60; z >= -100; z -= 10) {
            FootPos target = {x + 60, 46, z};
            float hip_a, knee_a;
            if (leg_ik(0, target, hip_a, knee_a)) {
                FootPos result = leg_fk_mm(0, hip_a, knee_a);
                float err = sqrtf(
                    (result.x-target.x)*(result.x-target.x) +
                    (result.z-target.z)*(result.z-target.z));
                if (err < 0.5f) sweep_pass++;
                sweep_total++;
            }
        }
    }
    printf("{\"test\":\"workspace_sweep\",\"pass_count\":%d,\"total\":%d,"
           "\"pass\":%s}\n", sweep_pass, sweep_total,
           (sweep_pass == sweep_total && sweep_total > 0) ? "true" : "false");
    (sweep_pass == sweep_total && sweep_total > 0) ? pass++ : fail++;

    // Test 5: STANDING_POSE values are all within per-joint soft clamp limits
    {
        bool all_ok = true;
        for (int i = 0; i < 8; i++) {
            bool in_range = STANDING_POSE[i] >= SERVO_JOINT_MIN_US[i] &&
                            STANDING_POSE[i] <= SERVO_JOINT_MAX_US[i];
            if (!in_range) {
                printf("{\"test\":\"standing_clamp_check\",\"idx\":%d,"
                       "\"pulse\":%d,\"min\":%d,\"max\":%d,\"pass\":false}\n",
                       i, STANDING_POSE[i], SERVO_JOINT_MIN_US[i], SERVO_JOINT_MAX_US[i]);
                all_ok = false;
            }
        }
        printf("{\"test\":\"standing_within_joint_clamps\",\"pass\":%s}\n",
               all_ok ? "true" : "false");
        all_ok ? pass++ : fail++;
    }

    // Test 6: FK/IK round-trip at non-standing foot offsets
    {
        struct { float dx, dz; } offsets[] = {
            {10, 0}, {-10, 0}, {0, -10}, {0, 10}, {15, -10}, {-15, 5}
        };
        int rt_pass = 0, rt_total = 0;
        for (int leg = 0; leg < 4; leg++) {
            FootPos sp = standing_foot_pos(leg);
            for (auto& off : offsets) {
                FootPos target = {sp.x + off.dx, sp.y, sp.z + off.dz};
                float hip_a, knee_a;
                if (!leg_ik(leg, target, hip_a, knee_a)) continue;
                FootPos result = leg_fk_mm(leg, hip_a, knee_a);
                float err = sqrtf((result.x - target.x) * (result.x - target.x) +
                                  (result.z - target.z) * (result.z - target.z));
                if (err < 0.5f) rt_pass++;
                rt_total++;
            }
        }
        bool ok = (rt_pass == rt_total && rt_total > 0);
        printf("{\"test\":\"offset_roundtrip\","
               "\"pass_count\":%d,\"total\":%d,\"pass\":%s}\n",
               rt_pass, rt_total, ok ? "true" : "false");
        ok ? pass++ : fail++;
    }

    printf("{\"summary\":\"ik_tests\",\"pass\":%d,\"fail\":%d}\n", pass, fail);
    return fail > 0 ? 1 : 0;
}
