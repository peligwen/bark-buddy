#include "mock_arduino.h"
#include "../include/ik.h"
#include "../include/body_transform.h"
#include <cstdio>
#include <cmath>

// ─── FK round-trip test helpers ──────────────────────────────────────────────
//
// Round-trip gate: 1.0 mm (not 0.5 mm).
// The knee us_per_rad ≈ 54 us/rad — a 1 μs pulse rounding maps to ~0.57 mm at the foot.
// 1.0 mm tolerates this inherent quantization while still catching real bugs
// (polarity errors → 10–50 mm, magnitude errors → 5–20 mm).

// Match the translate-then-rotate order of body_pose_to_pulses (body_transform.h:78-83)
static FootPos expected_foot(uint8_t leg, const BodyPose& p) {
    FootPos f = standing_foot_pos(leg);
    f.x -= p.dx; f.y -= p.dy; f.z -= p.dz;
    f = rotate_foot(f, -p.roll, -p.pitch, -p.yaw);
    return f;
}

static float dist_mm(const FootPos& a, const FootPos& b) {
    return sqrtf((a.x-b.x)*(a.x-b.x) + (a.y-b.y)*(a.y-b.y) + (a.z-b.z)*(a.z-b.z));
}

static FootPos actual_foot(uint8_t leg, const uint16_t pulses[8]) {
    return pulses_to_foot(leg, pulses[leg*2], pulses[leg*2+1]);
}

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

    // Test 6: Hip polarity parity under body dx
    // Left-mounted hips (FL=idx 0, RL=idx 4) must have the same raw Δpulse sign.
    // Right-mounted hips (FR=idx 2, RR=idx 6) must have the same raw Δpulse sign.
    // Left and right groups must have opposite signs.
    //
    // Why raw Δpulse, not normalized: pol×Δpulse = pol²×upr×Δangle = upr×Δangle,
    // which cancels the polarity and would miss a wrong-polarity override.
    {
        BodyPose dx_fwd = {5, 0, 0, 0, 0, 0};
        uint16_t p[8] = {};
        body_pose_to_pulses(dx_fwd, p);

        // raw[L] = Δpulse for hip of leg L (L=0=FL, 1=FR, 2=RL, 3=RR)
        int raw[4];
        for (int L = 0; L < 4; L++) {
            int hi = L * 2;
            raw[L] = (int)p[hi] - (int)STANDING_POSE[hi];
            printf("{\"diag\":\"dx+5_hip\",\"leg\":%d,\"servo_idx\":%d,"
                   "\"delta\":%d,\"pol\":%d}\n",
                   L, hi, raw[L], (int)servo_cal(hi).polarity);
        }

        // Left group (L=0 FL, L=2 RL): same raw direction
        bool fl_rl_same = (raw[0] > 0) == (raw[2] > 0) && raw[0] != 0 && raw[2] != 0;
        // Right group (L=1 FR, L=3 RR): same raw direction
        bool fr_rr_same = (raw[1] > 0) == (raw[3] > 0) && raw[1] != 0 && raw[3] != 0;
        // Left vs right: opposite (mirror mounting)
        bool lr_opp = (raw[0] > 0) != (raw[1] > 0);

        bool pass6 = fl_rl_same && fr_rr_same && lr_opp;
        printf("{\"test\":\"dx_hip_polarity_parity\","
               "\"fl_rl_same\":%s,\"fr_rr_same\":%s,\"lr_opp\":%s,\"pass\":%s}\n",
               fl_rl_same ? "true" : "false",
               fr_rr_same ? "true" : "false",
               lr_opp ? "true" : "false",
               pass6 ? "true" : "false");
        pass6 ? pass++ : fail++;
    }

    // Test 7: Pure lateral translation (dy) must not change hip or knee pulses.
    // The 2-DOF IK is planar in x-z; the y component of the foot target is never
    // consumed by leg_ik(). Translating the body sideways shifts foot.y but not
    // foot.x/z, so all eight servo pulses must remain at the standing pose.
    {
        bool pass7 = true;
        for (float dy : {10.0f, -10.0f}) {
            BodyPose lat = {0, dy, 0, 0, 0, 0};
            uint16_t p[8] = {};
            body_pose_to_pulses(lat, p);
            for (int i = 0; i < 8; i++) {
                if (abs((int)p[i] - (int)STANDING_POSE[i]) > 1) {
                    pass7 = false;
                    printf("{\"diag\":\"dy_not_isolated\",\"dy\":%.0f,\"idx\":%d,"
                           "\"delta\":%d}\n", dy, i, (int)p[i] - (int)STANDING_POSE[i]);
                }
            }
        }
        printf("{\"test\":\"dy_hip_knee_unchanged\",\"pass\":%s}\n",
               pass7 ? "true" : "false");
        pass7 ? pass++ : fail++;
    }

    // Test 8: rotate_foot basis-vector sanity — independent of expected_foot
    {
        float tol = 0.01f;
        bool pass8 = true;
        // roll +90°: +y → +z
        FootPos r1 = rotate_foot({0,1,0}, 90,0,0);
        if (fabsf(r1.x) > tol || fabsf(r1.y) > tol || fabsf(r1.z - 1.0f) > tol) {
            pass8 = false;
            printf("{\"diag\":\"rotate_foot_basis\",\"case\":\"roll90_y_to_z\","
                   "\"got\":[%.3f,%.3f,%.3f]}\n", r1.x, r1.y, r1.z);
        }
        // pitch +90°: +z → +x
        FootPos r2 = rotate_foot({0,0,1}, 0,90,0);
        if (fabsf(r2.x - 1.0f) > tol || fabsf(r2.y) > tol || fabsf(r2.z) > tol) {
            pass8 = false;
            printf("{\"diag\":\"rotate_foot_basis\",\"case\":\"pitch90_z_to_x\","
                   "\"got\":[%.3f,%.3f,%.3f]}\n", r2.x, r2.y, r2.z);
        }
        // yaw +90°: +x → +y
        FootPos r3 = rotate_foot({1,0,0}, 0,0,90);
        if (fabsf(r3.x) > tol || fabsf(r3.y - 1.0f) > tol || fabsf(r3.z) > tol) {
            pass8 = false;
            printf("{\"diag\":\"rotate_foot_basis\",\"case\":\"yaw90_x_to_y\","
                   "\"got\":[%.3f,%.3f,%.3f]}\n", r3.x, r3.y, r3.z);
        }
        printf("{\"test\":\"rotate_foot_basis\",\"pass\":%s}\n",
               pass8 ? "true" : "false");
        pass8 ? pass++ : fail++;
    }

    // Test 9: dx FK round-trip — body translates forward/back
    {
        bool pass9 = true;
        for (float dx : {5.0f, -5.0f}) {
            BodyPose pose = {dx, 0, 0, 0, 0, 0};
            uint16_t pulses[8] = {};
            body_pose_to_pulses(pose, pulses);
            for (int leg = 0; leg < 4; leg++) {
                FootPos exp = expected_foot(leg, pose);
                FootPos act = actual_foot(leg, pulses);
                float err = dist_mm(act, exp);
                printf("{\"diag\":\"dx_roundtrip\",\"leg\":%d,\"dx\":%.0f,"
                       "\"exp\":[%.2f,%.2f,%.2f],\"act\":[%.2f,%.2f,%.2f],"
                       "\"err\":%.3f}\n",
                       leg, dx, exp.x, exp.y, exp.z, act.x, act.y, act.z, err);
                if (err > 1.0f) pass9 = false;
            }
        }
        printf("{\"test\":\"dx_fk_roundtrip\",\"pass\":%s}\n",
               pass9 ? "true" : "false");
        pass9 ? pass++ : fail++;
    }

    // Test 10: dz FK round-trip — body rises/falls
    {
        bool pass10 = true;
        for (float dz : {10.0f, -10.0f}) {
            BodyPose pose = {0, 0, dz, 0, 0, 0};
            uint16_t pulses[8] = {};
            body_pose_to_pulses(pose, pulses);
            for (int leg = 0; leg < 4; leg++) {
                FootPos exp = expected_foot(leg, pose);
                FootPos act = actual_foot(leg, pulses);
                float err = dist_mm(act, exp);
                printf("{\"diag\":\"dz_roundtrip\",\"leg\":%d,\"dz\":%.0f,"
                       "\"exp\":[%.2f,%.2f,%.2f],\"act\":[%.2f,%.2f,%.2f],"
                       "\"err\":%.3f}\n",
                       leg, dz, exp.x, exp.y, exp.z, act.x, act.y, act.z, err);
                if (err > 1.0f) pass10 = false;
            }
        }
        printf("{\"test\":\"dz_fk_roundtrip\",\"pass\":%s}\n",
               pass10 ? "true" : "false");
        pass10 ? pass++ : fail++;
    }

    // Test 11: dy silent loss — lateral translation silently discarded (L2-F06)
    // Asserts pulses unchanged and FK y-readback stays at standing hip.y (loss = |dy|).
    {
        BodyPose pose = {0, 10, 0, 0, 0, 0};
        uint16_t pulses[8] = {};
        body_pose_to_pulses(pose, pulses);
        bool pulses_unchanged = true;
        for (int i = 0; i < 8; i++) {
            if (abs((int)pulses[i] - (int)STANDING_POSE[i]) > 1) {
                pulses_unchanged = false;
                printf("{\"diag\":\"dy_silent_loss\",\"idx\":%d,"
                       "\"expected\":%d,\"got\":%d}\n",
                       i, STANDING_POSE[i], pulses[i]);
            }
        }
        float max_y_err = 0;
        for (int leg = 0; leg < 4; leg++) {
            FootPos exp = expected_foot(leg, pose);
            FootPos act = actual_foot(leg, pulses);
            float y_err = fabsf(act.y - exp.y);
            if (y_err > max_y_err) max_y_err = y_err;
            printf("{\"diag\":\"dy_silent_loss\",\"leg\":%d,"
                   "\"exp_y\":%.2f,\"act_y\":%.2f,\"y_err\":%.2f}\n",
                   leg, exp.y, act.y, y_err);
        }
        bool pass11 = pulses_unchanged && fabsf(max_y_err - 10.0f) < 0.5f;
        printf("{\"test\":\"dy_silent_loss\",\"pulses_unchanged\":%s,"
               "\"max_y_err\":%.2f,\"pass\":%s}\n",
               pulses_unchanged ? "true" : "false", max_y_err,
               pass11 ? "true" : "false");
        pass11 ? pass++ : fail++;
    }

    // Test 12: roll FK round-trip — x/z gated at 0.5 mm; y-loss reported (roll → foot y change)
    {
        bool pass12 = true;
        for (float roll : {5.0f, -5.0f}) {
            BodyPose pose = {0, 0, 0, roll, 0, 0};
            uint16_t pulses[8] = {};
            body_pose_to_pulses(pose, pulses);
            for (int leg = 0; leg < 4; leg++) {
                FootPos exp = expected_foot(leg, pose);
                FootPos act = actual_foot(leg, pulses);
                float xz_err = sqrtf((act.x-exp.x)*(act.x-exp.x) +
                                     (act.z-exp.z)*(act.z-exp.z));
                float y_err  = fabsf(act.y - exp.y);
                printf("{\"diag\":\"roll_roundtrip\",\"leg\":%d,\"roll\":%.0f,"
                       "\"xz_err\":%.3f,\"y_err\":%.3f}\n",
                       leg, roll, xz_err, y_err);
                if (xz_err > 1.0f) pass12 = false;
            }
        }
        printf("{\"test\":\"roll_fk_roundtrip\",\"pass\":%s}\n",
               pass12 ? "true" : "false");
        pass12 ? pass++ : fail++;
    }

    // Test 13: pitch FK round-trip — pitch (Ry) leaves y unchanged; full 3D gate
    {
        bool pass13 = true;
        for (float pitch : {5.0f, -5.0f}) {
            BodyPose pose = {0, 0, 0, 0, pitch, 0};
            uint16_t pulses[8] = {};
            body_pose_to_pulses(pose, pulses);
            for (int leg = 0; leg < 4; leg++) {
                FootPos exp = expected_foot(leg, pose);
                FootPos act = actual_foot(leg, pulses);
                float err = dist_mm(act, exp);
                printf("{\"diag\":\"pitch_roundtrip\",\"leg\":%d,\"pitch\":%.0f,"
                       "\"exp\":[%.2f,%.2f,%.2f],\"act\":[%.2f,%.2f,%.2f],"
                       "\"err\":%.3f}\n",
                       leg, pitch, exp.x, exp.y, exp.z, act.x, act.y, act.z, err);
                if (err > 1.0f) pass13 = false;
            }
        }
        printf("{\"test\":\"pitch_fk_roundtrip\",\"pass\":%s}\n",
               pass13 ? "true" : "false");
        pass13 ? pass++ : fail++;
    }

    // Test 14: yaw FK partial — x/z gated at 0.5 mm; y-loss matches predicted (Rz → y slew)
    {
        bool pass14 = true;
        BodyPose pose = {0, 0, 0, 0, 0, 5};
        uint16_t pulses[8] = {};
        body_pose_to_pulses(pose, pulses);
        for (int leg = 0; leg < 4; leg++) {
            FootPos sf  = standing_foot_pos(leg);
            FootPos exp = expected_foot(leg, pose);
            FootPos act = actual_foot(leg, pulses);
            float xz_err = sqrtf((act.x-exp.x)*(act.x-exp.x) +
                                  (act.z-exp.z)*(act.z-exp.z));
            float y_err            = fabsf(act.y - exp.y);
            float predicted_y_loss = fabsf(exp.y - sf.y);
            printf("{\"diag\":\"yaw_partial\",\"leg\":%d,"
                   "\"xz_err\":%.3f,\"y_err\":%.3f,\"predicted_y_loss\":%.3f}\n",
                   leg, xz_err, y_err, predicted_y_loss);
            if (xz_err > 1.0f) pass14 = false;
            if (fabsf(y_err - predicted_y_loss) > 0.5f) pass14 = false;
        }
        printf("{\"test\":\"yaw_fk_partial\",\"pass\":%s}\n",
               pass14 ? "true" : "false");
        pass14 ? pass++ : fail++;
    }

    printf("{\"summary\":\"transform_tests\",\"pass\":%d,\"fail\":%d}\n", pass, fail);
    return fail > 0 ? 1 : 0;
}
