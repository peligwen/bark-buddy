#include "mock_arduino.h"
#include "../include/ik.h"
#include "../include/body_transform.h"
#include "../include/balance.h"
#include <cstdio>
#include <cmath>

int main() {
    int pass = 0, fail = 0;

    // Test 1: Convergence — 5-degree initial pitch corrects within 2 seconds at 50Hz
    {
        BalanceConfig cfg = {
            0.3f, 0.01f, 0.05f,  // kp, ki, kd for pitch
            0.3f, 0.01f, 0.05f,  // kp, ki, kd for roll
            8.0f,                  // max correction degrees
            0.3f                   // deadband degrees
        };
        balance_init(cfg);
        balance_enable(true);
        balance_reset();

        float pitch = 5.0f;
        float roll = 0.0f;
        float dt = 0.02f;   // 50 Hz
        int steps = 100;    // 2 seconds

        for (int i = 0; i < steps; i++) {
            BodyPose correction = balance_update(pitch, roll, dt);
            // Simplified plant model: correction reduces error
            pitch -= correction.pitch * 0.5f;
            roll  -= correction.roll  * 0.5f;
        }

        bool converged = fabsf(pitch) < 1.0f;
        printf("{\"test\":\"convergence\",\"final_pitch\":%.2f,\"pass\":%s}\n",
               pitch, converged ? "true" : "false");
        converged ? pass++ : fail++;
    }

    // Test 2: Deadband — errors below threshold produce zero output
    {
        BalanceConfig cfg = {0.3f, 0.01f, 0.05f, 0.3f, 0.01f, 0.05f, 8.0f, 0.5f};
        balance_init(cfg);
        balance_enable(true);
        balance_reset();

        BodyPose out = balance_update(0.3f, 0.2f, 0.02f);  // below 0.5 deadband
        bool zero_out = fabsf(out.pitch) < 0.001f && fabsf(out.roll) < 0.001f;
        printf("{\"test\":\"deadband\",\"pass\":%s}\n",
               zero_out ? "true" : "false");
        zero_out ? pass++ : fail++;
    }

    // Test 3: Max correction clamp
    {
        BalanceConfig cfg = {2.0f, 0.0f, 0.0f, 2.0f, 0.0f, 0.0f, 5.0f, 0.0f};
        balance_init(cfg);
        balance_enable(true);
        balance_reset();

        BodyPose out = balance_update(30.0f, 0.0f, 0.02f);  // huge error
        bool clamped = fabsf(out.pitch) <= 5.1f;
        printf("{\"test\":\"max_clamp\",\"correction\":%.2f,\"pass\":%s}\n",
               out.pitch, clamped ? "true" : "false");
        clamped ? pass++ : fail++;
    }

    // Test 4: Disabled balance produces zero output
    {
        BalanceConfig cfg = {0.3f, 0.01f, 0.05f, 0.3f, 0.01f, 0.05f, 8.0f, 0.3f};
        balance_init(cfg);
        balance_enable(false);

        BodyPose out = balance_update(10.0f, 5.0f, 0.02f);
        bool zero = fabsf(out.pitch) < 0.001f && fabsf(out.roll) < 0.001f;
        printf("{\"test\":\"disabled\",\"pass\":%s}\n",
               zero ? "true" : "false");
        zero ? pass++ : fail++;
    }

    // Test 5: balance_set_gains updates all six PID gain fields
    {
        BalanceConfig cfg = {0.1f, 0.01f, 0.02f, 0.1f, 0.01f, 0.02f, 8.0f, 0.3f};
        balance_init(cfg);

        balance_set_gains(1.1f, 2.2f, 3.3f, 4.4f, 5.5f, 6.6f);

        float pkp, pki, pkd, rkp, rki, rkd;
        balance_get_gains(&pkp, &pki, &pkd, &rkp, &rki, &rkd);

        bool ok = fabsf(pkp - 1.1f) < 1e-5f &&
                  fabsf(pki - 2.2f) < 1e-5f &&
                  fabsf(pkd - 3.3f) < 1e-5f &&
                  fabsf(rkp - 4.4f) < 1e-5f &&
                  fabsf(rki - 5.5f) < 1e-5f &&
                  fabsf(rkd - 6.6f) < 1e-5f;
        printf("{\"test\":\"set_gains_updates_pid\",\"pass\":%s}\n",
               ok ? "true" : "false");
        ok ? pass++ : fail++;
    }

    // Test 6: balance_set_gains does NOT reset the integral accumulator.
    // Strategy: accumulate integral for 10 steps, call balance_set_gains with
    // the same gains, then take one more step and compare against a fresh
    // instance that starts from zero (balance_reset). The accumulated instance
    // should produce a noticeably higher output because its integral is non-zero.
    {
        BalanceConfig cfg = {0.3f, 0.1f, 0.02f, 0.3f, 0.1f, 0.02f, 8.0f, 0.0f};
        float dt = 0.02f;
        float error_pitch = 3.0f;
        float error_roll  = 2.0f;

        // --- accumulated instance ---
        balance_init(cfg);
        balance_enable(true);
        for (int i = 0; i < 10; i++) {
            balance_update(error_pitch, error_roll, dt);
        }
        float pkp, pki, pkd, rkp, rki, rkd;
        balance_get_gains(&pkp, &pki, &pkd, &rkp, &rki, &rkd);
        balance_set_gains(pkp, pki, pkd, rkp, rki, rkd);  // no-op on values
        BodyPose with_integral = balance_update(error_pitch, error_roll, dt);

        // --- fresh instance (integral reset to zero) ---
        balance_init(cfg);
        balance_enable(true);
        BodyPose fresh = balance_update(error_pitch, error_roll, dt);

        // The accumulated version should have a larger pitch correction because
        // ki*integral contributes positively. Any meaningful ki will cause a
        // difference well above the 1e-3 threshold.
        bool preserved = with_integral.pitch > fresh.pitch + 1e-3f;
        printf("{\"test\":\"set_gains_preserves_integral\","
               "\"accumulated_pitch\":%.4f,\"fresh_pitch\":%.4f,\"pass\":%s}\n",
               with_integral.pitch, fresh.pitch, preserved ? "true" : "false");
        preserved ? pass++ : fail++;
    }

    printf("{\"summary\":\"balance_tests\",\"pass\":%d,\"fail\":%d}\n", pass, fail);
    return fail > 0 ? 1 : 0;
}
