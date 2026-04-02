// Gait test suite — runs on host, outputs JSON results
// Build: make -C firmware/test
// Run:   firmware/test/gait_tests [test_name]

#include "mock_arduino.h"
#include "kinematics.h"
#include "mock_imu.h"
#include "balance_pid.h"
#include <cstdio>
#include <cstring>
#include <cmath>
#include <vector>
#include <string>

// Gait parameters (matching firmware gait.cpp)
struct GaitParams {
    float hip_amplitude = 0.22f;
    float knee_amplitude = 0.35f;
    float knee_bias = 0.1f;
    float frequency = 2.0f;  // Hz
    std::string name = "default";
};

// Generate one frame of trot gait angles
void trot_gait(const GaitParams& p, float phase, float angles[8]) {
    float phA = sinf(phase);
    float phB = sinf(phase + M_PI);

    float kneeA = FK_STAND_KNEE - p.knee_bias - p.knee_amplitude * fabsf(phA);
    float kneeB = FK_STAND_KNEE - p.knee_bias - p.knee_amplitude * fabsf(phB);

    // FL
    angles[0] = FK_STAND_HIP + p.hip_amplitude * phA;
    angles[1] = kneeA;
    // FR
    angles[2] = FK_STAND_HIP + p.hip_amplitude * phB;
    angles[3] = kneeB;
    // RL
    angles[4] = FK_STAND_HIP + p.hip_amplitude * phB;
    angles[5] = kneeB;
    // RR
    angles[6] = FK_STAND_HIP + p.hip_amplitude * phA;
    angles[7] = kneeA;
}

// ============================================================
// Test: foot ground clearance
// ============================================================
void test_foot_clearance(const GaitParams& params) {
    float min_y[4] = {1e6, 1e6, 1e6, 1e6};
    float max_y[4] = {-1e6, -1e6, -1e6, -1e6};
    int ground_clips = 0;
    float body_h = standing_height();

    int steps = 1000;
    for (int i = 0; i < steps; i++) {
        float phase = (float)i / steps * 2 * M_PI * 3; // 3 full cycles
        float angles[8];
        trot_gait(params, phase, angles);
        BodyState state = compute_body_state(angles);

        for (int leg = 0; leg < 4; leg++) {
            float fy = state.legs[leg].foot.y;
            if (fy < min_y[leg]) min_y[leg] = fy;
            if (fy > max_y[leg]) max_y[leg] = fy;
            if (fy < FOOT_R_M) ground_clips++;
        }
    }

    printf("{\"test\":\"foot_clearance\",\"params\":\"%s\","
           "\"min_foot_y_mm\":[%.1f,%.1f,%.1f,%.1f],"
           "\"max_foot_y_mm\":[%.1f,%.1f,%.1f,%.1f],"
           "\"ground_clips\":%d,\"total_samples\":%d}\n",
           params.name.c_str(),
           min_y[0]*1000, min_y[1]*1000, min_y[2]*1000, min_y[3]*1000,
           max_y[0]*1000, max_y[1]*1000, max_y[2]*1000, max_y[3]*1000,
           ground_clips, steps * 4);
}

// ============================================================
// Test: gait symmetry
// ============================================================
void test_gait_symmetry(const GaitParams& params) {
    // Compare FL vs RR (pair A) and FR vs RL (pair B)
    float max_hip_diff_a = 0, max_hip_diff_b = 0;
    float max_knee_diff_a = 0, max_knee_diff_b = 0;

    int steps = 500;
    for (int i = 0; i < steps; i++) {
        float phase = (float)i / steps * 2 * M_PI;
        float angles[8];
        trot_gait(params, phase, angles);

        float hip_diff_a = fabsf(angles[0] - angles[6]); // FL vs RR
        float hip_diff_b = fabsf(angles[2] - angles[4]); // FR vs RL
        float knee_diff_a = fabsf(angles[1] - angles[7]);
        float knee_diff_b = fabsf(angles[3] - angles[5]);

        if (hip_diff_a > max_hip_diff_a) max_hip_diff_a = hip_diff_a;
        if (hip_diff_b > max_hip_diff_b) max_hip_diff_b = hip_diff_b;
        if (knee_diff_a > max_knee_diff_a) max_knee_diff_a = knee_diff_a;
        if (knee_diff_b > max_knee_diff_b) max_knee_diff_b = knee_diff_b;
    }

    printf("{\"test\":\"gait_symmetry\",\"params\":\"%s\","
           "\"max_hip_diff_pair_a_deg\":%.2f,\"max_hip_diff_pair_b_deg\":%.2f,"
           "\"max_knee_diff_pair_a_deg\":%.2f,\"max_knee_diff_pair_b_deg\":%.2f,"
           "\"symmetric\":%s}\n",
           params.name.c_str(),
           max_hip_diff_a * 180/M_PI, max_hip_diff_b * 180/M_PI,
           max_knee_diff_a * 180/M_PI, max_knee_diff_b * 180/M_PI,
           (max_hip_diff_a < 0.001 && max_hip_diff_b < 0.001) ? "true" : "false");
}

// ============================================================
// Test: body stability (pitch/roll oscillation during walk)
// ============================================================
void test_body_stability(const GaitParams& params) {
    float max_pitch = 0, max_roll = 0;
    float sum_pitch = 0, sum_roll = 0;

    int steps = 1000;
    for (int i = 0; i < steps; i++) {
        float phase = (float)i / steps * 2 * M_PI * 3;
        float angles[8];
        trot_gait(params, phase, angles);
        BodyState state = compute_body_state(angles);

        float p = fabsf(state.pitch * 180 / M_PI);
        float r = fabsf(state.roll * 180 / M_PI);
        if (p > max_pitch) max_pitch = p;
        if (r > max_roll) max_roll = r;
        sum_pitch += p;
        sum_roll += r;
    }

    printf("{\"test\":\"body_stability\",\"params\":\"%s\","
           "\"max_pitch_deg\":%.2f,\"max_roll_deg\":%.2f,"
           "\"avg_pitch_deg\":%.2f,\"avg_roll_deg\":%.2f}\n",
           params.name.c_str(),
           max_pitch, max_roll,
           sum_pitch / steps, sum_roll / steps);
}

// ============================================================
// Test: foot trajectory (full cycle path for each foot)
// ============================================================
void test_foot_trajectory(const GaitParams& params) {
    printf("{\"test\":\"foot_trajectory\",\"params\":\"%s\",\"legs\":{", params.name.c_str());

    const char* leg_names[] = {"FL", "FR", "RL", "RR"};
    for (int leg = 0; leg < 4; leg++) {
        if (leg > 0) printf(",");
        printf("\"%s\":[", leg_names[leg]);

        int steps = 100;
        for (int i = 0; i < steps; i++) {
            float phase = (float)i / steps * 2 * M_PI;
            float angles[8];
            trot_gait(params, phase, angles);
            BodyState state = compute_body_state(angles);

            if (i > 0) printf(",");
            printf("[%.3f,%.3f,%.3f]",
                   state.legs[leg].foot.x * 1000,
                   state.legs[leg].foot.y * 1000,
                   state.legs[leg].foot.z * 1000);
        }
        printf("]");
    }
    printf("}}\n");
}

// ============================================================
// Test: balance convergence
// ============================================================
void test_balance_convergence(const GaitParams& params, const IMUConfig& imu_config) {
    SimIMU imu;
    imu.config = imu_config;
    BalancePID balance;

    float dt = 0.02f; // 50Hz
    float angles[8];
    int converge_step = -1;

    // Start with a deliberate tilt (simulate bump)
    float initial_tilt = 5.0f; // degrees

    int steps = 500; // 10 seconds
    std::vector<float> pitch_history;

    for (int i = 0; i < steps; i++) {
        float phase = (float)i / steps * 2 * M_PI * 5; // 5 cycles
        trot_gait(params, phase, angles);

        // Apply balance corrections
        balance.apply(angles);

        BodyState state = compute_body_state(angles);

        // Add simulated initial tilt (decays as balance corrects)
        float sim_pitch = state.pitch * 180 / M_PI;
        if (i < 10) sim_pitch += initial_tilt;

        float imu_pitch, imu_roll, imu_yaw;
        imu.read(state, dt, imu_pitch, imu_roll, imu_yaw);

        balance.update(imu_pitch, imu_roll, dt);
        pitch_history.push_back(imu_pitch);

        // Check convergence: pitch < 1 degree
        if (converge_step < 0 && i > 10 && fabsf(imu_pitch) < 1.0f) {
            converge_step = i;
        }
    }

    float final_pitch = pitch_history.back();
    float max_overshoot = 0;
    for (float p : pitch_history) {
        if (fabsf(p) > max_overshoot) max_overshoot = fabsf(p);
    }

    printf("{\"test\":\"balance_convergence\",\"params\":\"%s\","
           "\"imu_noise\":%.1f,\"imu_drift\":%.2f,"
           "\"initial_tilt_deg\":%.1f,"
           "\"converge_step\":%d,\"converge_time_ms\":%d,"
           "\"final_pitch_deg\":%.2f,\"max_overshoot_deg\":%.2f,"
           "\"pid_kp\":%.3f,\"pid_ki\":%.4f,\"pid_kd\":%.4f}\n",
           params.name.c_str(),
           imu_config.noise_stddev_deg, imu_config.drift_rate_dps,
           initial_tilt,
           converge_step, (int)(converge_step * dt * 1000),
           final_pitch, max_overshoot,
           balance.pitch_gains.kp, balance.pitch_gains.ki, balance.pitch_gains.kd);
}

// ============================================================
// Test: parameter sweep — vary one param, measure clearance + stability
// ============================================================
void test_parameter_sweep() {
    printf("{\"test\":\"parameter_sweep\",\"results\":[\n");

    float hip_amps[] = {0.10, 0.15, 0.20, 0.25, 0.30, 0.35};
    float knee_amps[] = {0.20, 0.25, 0.30, 0.35, 0.40, 0.50};
    float knee_biases[] = {0.0, 0.05, 0.10, 0.15, 0.20};

    bool first = true;
    for (float ha : hip_amps) {
        for (float ka : knee_amps) {
            for (float kb : knee_biases) {
                GaitParams p;
                p.hip_amplitude = ha;
                p.knee_amplitude = ka;
                p.knee_bias = kb;

                float min_foot_y = 1e6;
                float max_pitch = 0;
                int clips = 0;

                for (int i = 0; i < 500; i++) {
                    float phase = (float)i / 500 * 2 * M_PI * 2;
                    float angles[8];
                    trot_gait(p, phase, angles);
                    BodyState state = compute_body_state(angles);

                    for (int leg = 0; leg < 4; leg++) {
                        float fy = state.legs[leg].foot.y;
                        if (fy < min_foot_y) min_foot_y = fy;
                        if (fy < FOOT_R_M) clips++;
                    }
                    float pitch = fabsf(state.pitch * 180 / M_PI);
                    if (pitch > max_pitch) max_pitch = pitch;
                }

                if (!first) printf(",\n");
                first = false;
                printf("{\"hip_amp\":%.2f,\"knee_amp\":%.2f,\"knee_bias\":%.2f,"
                       "\"min_foot_y_mm\":%.1f,\"ground_clips\":%d,\"max_pitch_deg\":%.2f}",
                       ha, ka, kb, min_foot_y * 1000, clips, max_pitch);
            }
        }
    }
    printf("\n]}\n");
}

// ============================================================
// Main
// ============================================================
int main(int argc, char* argv[]) {
    std::string test = (argc > 1) ? argv[1] : "all";

    GaitParams default_params;
    default_params.name = "default";

    GaitParams wide;
    wide.hip_amplitude = 0.30;
    wide.knee_amplitude = 0.40;
    wide.knee_bias = 0.15;
    wide.name = "wide_stride";

    GaitParams gentle;
    gentle.hip_amplitude = 0.12;
    gentle.knee_amplitude = 0.25;
    gentle.knee_bias = 0.08;
    gentle.name = "gentle";

    IMUConfig clean_imu;
    IMUConfig noisy_imu;
    noisy_imu.noise_stddev_deg = 2.0;
    noisy_imu.drift_rate_dps = 0.5;

    if (test == "all" || test == "clearance") {
        test_foot_clearance(default_params);
        test_foot_clearance(wide);
        test_foot_clearance(gentle);
    }

    if (test == "all" || test == "symmetry") {
        test_gait_symmetry(default_params);
    }

    if (test == "all" || test == "stability") {
        test_body_stability(default_params);
        test_body_stability(wide);
        test_body_stability(gentle);
    }

    if (test == "all" || test == "trajectory") {
        test_foot_trajectory(default_params);
    }

    if (test == "all" || test == "balance") {
        test_balance_convergence(default_params, clean_imu);
        test_balance_convergence(default_params, noisy_imu);
    }

    if (test == "sweep") {
        test_parameter_sweep();
    }

    return 0;
}
