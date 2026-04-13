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

    printf("{\"summary\":\"balance_tests\",\"pass\":%d,\"fail\":%d}\n", pass, fail);
    return fail > 0 ? 1 : 0;
}
