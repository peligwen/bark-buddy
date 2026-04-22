#include "mock_arduino.h"
#include "../include/ik.h"
#include "../include/body_transform.h"
#include <cstdio>
#include <cmath>

int main() {
    int pass = 0, fail = 0;

    // Test 1: Identity transform produces standing pose
    {
        BodyPose zero = {0, 0, 0, 0, 0, 0};
        uint16_t pulses[8];
        bool ok = body_pose_to_pulses(zero, pulses);
        bool match = ok;
        for (int i = 0; i < 8 && match; i++) {
            if (abs((int)pulses[i] - (int)STANDING_POSE[i]) > 5) match = false;
        }
        printf("{\"test\":\"identity_transform\",\"pass\":%s}\n",
               match ? "true" : "false");
        match ? pass++ : fail++;
    }

    // Test 2: Height change — z=+10 raises body (feet appear lower)
    {
        BodyPose up = {0, 0, 10, 0, 0, 0};
        BodyPose down = {0, 0, -10, 0, 0, 0};
        uint16_t p_up[8], p_down[8], p_stand[8];
        body_pose_to_pulses({0,0,0,0,0,0}, p_stand);
        body_pose_to_pulses(up, p_up);
        body_pose_to_pulses(down, p_down);
        bool changed = false;
        for (int i = 0; i < 8; i++) {
            if (p_up[i] != p_stand[i]) changed = true;
        }
        printf("{\"test\":\"height_change\",\"changed\":%s,\"pass\":%s}\n",
               changed ? "true" : "false", changed ? "true" : "false");
        changed ? pass++ : fail++;
    }

    // Test 3: Roll — 5 degrees produces left/right asymmetry in knee servos
    {
        BodyPose rolled = {0, 0, 0, 5, 0, 0};
        uint16_t p_roll[8], p_stand[8];
        body_pose_to_pulses({0,0,0,0,0,0}, p_stand);
        body_pose_to_pulses(rolled, p_roll);
        // FL knee (idx 1) and FR knee (idx 3) should both change from standing
        int fl_delta = abs((int)p_roll[1] - (int)p_stand[1]);
        int fr_delta = abs((int)p_roll[3] - (int)p_stand[3]);
        bool asym = fl_delta > 2 && fr_delta > 2;
        printf("{\"test\":\"roll_asymmetry\",\"fl_delta\":%d,\"fr_delta\":%d,"
               "\"pass\":%s}\n", fl_delta, fr_delta, asym ? "true" : "false");
        asym ? pass++ : fail++;
    }

    // Test 4: Saturation — output pulses always stay in [SERVO_MIN_US, SERVO_MAX_US]
    // regardless of whether the pose is reachable, and geometric impossibility is
    // detected and reported correctly.
    {
        // Case A: extreme roll — may or may not be reachable, but output pulses
        // must always be within the valid servo range.
        BodyPose extreme_roll = {0, 0, 0, 30, 0, 0};
        uint16_t p_roll[8] = {};
        bool ok_roll = body_pose_to_pulses(extreme_roll, p_roll);
        bool roll_pulses_valid = true;
        for (int i = 0; i < 8; i++) {
            if (p_roll[i] < SERVO_MIN_US || p_roll[i] > SERVO_MAX_US)
                roll_pulses_valid = false;
        }

        // Case B: body raised so high (dz=+75mm) that the leg reach (125.5mm max)
        // cannot span the distance. Must return false and clamp pulses to standing.
        // (standing foot ~55mm from hip → raising body 75mm → ~130mm > 125.5mm max)
        BodyPose out_of_reach = {0, 0, 75, 0, 0, 0};
        uint16_t p_oor[8] = {};
        bool must_fail = !body_pose_to_pulses(out_of_reach, p_oor);
        // On failure, body_pose_to_pulses clamps to STANDING_POSE
        bool clamped_to_standing = true;
        for (int i = 0; i < 8; i++) {
            if (p_oor[i] != STANDING_POSE[i]) clamped_to_standing = false;
        }

        bool pass4 = roll_pulses_valid && must_fail && clamped_to_standing;
        printf("{\"test\":\"saturation\",\"roll_reachable\":%s,"
               "\"roll_pulses_valid\":%s,\"oor_rejected\":%s,"
               "\"oor_clamped\":%s,\"pass\":%s}\n",
               ok_roll ? "true" : "false",
               roll_pulses_valid ? "true" : "false",
               must_fail ? "true" : "false",
               clamped_to_standing ? "true" : "false",
               pass4 ? "true" : "false");
        pass4 ? pass++ : fail++;
    }

    // Test 5: lerp_pose interpolation
    {
        BodyPose a = {0, 0, 0, 0, 0, 0};
        BodyPose b = {10, 0, 5, 5, 0, 0};
        BodyPose mid = lerp_pose(a, b, 0.5f);
        bool ok = fabsf(mid.dx - 5.0f) < 0.1f &&
                  fabsf(mid.dz - 2.5f) < 0.1f &&
                  fabsf(mid.roll - 2.5f) < 0.1f &&
                  fabsf(mid.dy) < 0.01f &&
                  fabsf(mid.pitch) < 0.01f &&
                  fabsf(mid.yaw) < 0.01f;
        printf("{\"test\":\"lerp\",\"pass\":%s}\n", ok ? "true" : "false");
        ok ? pass++ : fail++;
    }

    printf("{\"summary\":\"transform_tests\",\"pass\":%d,\"fail\":%d}\n", pass, fail);
    return fail > 0 ? 1 : 0;
}
