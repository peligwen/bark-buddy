// Profile analyzer — compare captured IMU data to physical model predictions.
// Fits physical parameters to minimize prediction error.
//
// Usage:
//   ./profile_analyzer <captured_profile.json>
//   ./profile_analyzer <captured_profile.json> --optimize
//
// Input: JSON with arrays of {timestamp, pitch, roll, gz, servo_angles[8]}
// Output: JSON with error metrics and optimized parameters

#include "mock_arduino.h"
#include "kinematics.h"
#include "physical_model.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>

// Simple JSON value extraction (no library dependency)
static float json_float(const std::string& json, const char* key, float def = 0) {
    std::string needle = std::string("\"") + key + "\":";
    auto pos = json.find(needle);
    if (pos == std::string::npos) return def;
    pos += needle.length();
    while (pos < json.length() && json[pos] == ' ') pos++;
    return atof(json.c_str() + pos);
}

static std::vector<float> json_array(const std::string& json, const char* key) {
    std::vector<float> result;
    std::string needle = std::string("\"") + key + "\":[";
    auto pos = json.find(needle);
    if (pos == std::string::npos) return result;
    pos += needle.length();
    while (pos < json.length() && json[pos] != ']') {
        while (pos < json.length() && (json[pos] == ' ' || json[pos] == ',')) pos++;
        if (json[pos] == ']') break;
        result.push_back(atof(json.c_str() + pos));
        while (pos < json.length() && json[pos] != ',' && json[pos] != ']') pos++;
    }
    return result;
}

struct CapturedSample {
    float timestamp;
    float pitch;
    float roll;
    float gz;
    float angles[8];
};

// Parse captured profile from NDJSON file
static std::vector<CapturedSample> load_profile(const char* filename) {
    std::vector<CapturedSample> samples;
    std::ifstream file(filename);
    std::string line;

    while (std::getline(file, line)) {
        if (line.empty() || line[0] != '{') continue;

        CapturedSample s = {};
        s.timestamp = json_float(line, "t");
        s.pitch = json_float(line, "pitch");
        s.roll = json_float(line, "roll");
        s.gz = json_float(line, "gz");

        auto angles = json_array(line, "angles");
        for (int i = 0; i < 8 && i < (int)angles.size(); i++) {
            s.angles[i] = angles[i];
        }
        // If no explicit angles, use standing pose
        if (angles.empty()) {
            for (int i = 0; i < 4; i++) {
                s.angles[i * 2] = FK_STAND_HIP;
                s.angles[i * 2 + 1] = FK_STAND_KNEE;
            }
        }
        samples.push_back(s);
    }
    return samples;
}

// Run the physical model against captured data and compute error
static float evaluate(const PhysicalParams& params, const std::vector<CapturedSample>& samples) {
    if (samples.size() < 2) return 1e6;

    float dt = (samples.size() > 1)
        ? (samples.back().timestamp - samples.front().timestamp) / (samples.size() - 1)
        : 0.02f;

    std::vector<float> cap_pitch(samples.size());
    std::vector<float> cap_roll(samples.size());
    std::vector<float> servo_data(samples.size() * 8);

    for (size_t i = 0; i < samples.size(); i++) {
        cap_pitch[i] = samples[i].pitch;
        cap_roll[i] = samples[i].roll;
        memcpy(&servo_data[i * 8], samples[i].angles, sizeof(float) * 8);
    }

    return profile_error(params, cap_pitch.data(), cap_roll.data(),
                          servo_data.data(), samples.size(), dt);
}

// Nelder-Mead simplex optimization on key parameters
struct OptParam {
    const char* name;
    float* ptr;
    float min_val;
    float max_val;
    float step;
};

static void optimize(PhysicalParams& params, const std::vector<CapturedSample>& samples) {
    // Parameters to optimize
    OptParam opt_params[] = {
        {"com_offset_x", &params.com_offset_x, -0.05f, 0.05f, 0.005f},
        {"com_offset_z", &params.com_offset_z, -0.03f, 0.03f, 0.003f},
        {"body_mass_kg", &params.body_mass_kg, 0.3f, 1.5f, 0.05f},
        {"inertia_pitch", &params.inertia_pitch, 0.0002f, 0.01f, 0.0005f},
        {"inertia_roll", &params.inertia_roll, 0.0002f, 0.01f, 0.0005f},
        {"servo_lag_ms", &params.servo_lag_ms, 5.0f, 100.0f, 5.0f},
        {"pitch_damping", &params.pitch_damping, 0.01f, 0.2f, 0.01f},
        {"roll_damping", &params.roll_damping, 0.01f, 0.2f, 0.01f},
    };
    int n_params = sizeof(opt_params) / sizeof(opt_params[0]);

    float best_error = evaluate(params, samples);
    int iterations = 0;
    int max_iterations = 200;
    bool improved = true;

    fprintf(stderr, "Starting optimization, initial error: %.4f\n", best_error);

    // Coordinate descent — iterate through each parameter
    while (improved && iterations < max_iterations) {
        improved = false;
        for (int p = 0; p < n_params; p++) {
            float original = *opt_params[p].ptr;
            float best_val = original;

            // Try positive step
            *opt_params[p].ptr = fminf(original + opt_params[p].step, opt_params[p].max_val);
            float err_plus = evaluate(params, samples);

            // Try negative step
            *opt_params[p].ptr = fmaxf(original - opt_params[p].step, opt_params[p].min_val);
            float err_minus = evaluate(params, samples);

            if (err_plus < best_error && err_plus <= err_minus) {
                best_error = err_plus;
                best_val = fminf(original + opt_params[p].step, opt_params[p].max_val);
                improved = true;
            } else if (err_minus < best_error) {
                best_error = err_minus;
                best_val = fmaxf(original - opt_params[p].step, opt_params[p].min_val);
                improved = true;
            }

            *opt_params[p].ptr = best_val;
            iterations++;
        }
    }

    fprintf(stderr, "Optimization done: %d iterations, final error: %.4f\n", iterations, best_error);
}

// Print results as JSON
static void print_results(const PhysicalParams& params, float error,
                           const std::vector<CapturedSample>& samples, bool optimized) {
    // Run model to get predicted profile
    DynamicState state = {};
    float dt = (samples.size() > 1)
        ? (samples.back().timestamp - samples.front().timestamp) / (samples.size() - 1)
        : 0.02f;

    printf("{\"test\":\"profile_analysis\",\"optimized\":%s,"
           "\"error_mse\":%.4f,\"num_samples\":%zu,\"sample_dt\":%.4f,\n",
           optimized ? "true" : "false", error, samples.size(), dt);

    // Physical parameters
    printf("\"params\":{"
           "\"body_mass_kg\":%.3f,"
           "\"com_offset_x\":%.4f,\"com_offset_z\":%.4f,"
           "\"inertia_pitch\":%.5f,\"inertia_roll\":%.5f,"
           "\"servo_lag_ms\":%.1f,"
           "\"pitch_damping\":%.3f,\"roll_damping\":%.3f},\n",
           params.body_mass_kg,
           params.com_offset_x, params.com_offset_z,
           params.inertia_pitch, params.inertia_roll,
           params.servo_lag_ms,
           params.pitch_damping, params.roll_damping);

    // Predicted vs captured comparison (first 50 samples)
    printf("\"comparison\":[");
    int limit = std::min((int)samples.size(), 50);
    for (int i = 0; i < limit; i++) {
        float pred_pitch, pred_roll, pred_gz;
        simulate_step(params, state, samples[i].angles, dt, pred_pitch, pred_roll, pred_gz);
        if (i > 0) printf(",");
        printf("{\"t\":%.3f,\"cap_p\":%.1f,\"cap_r\":%.1f,\"pred_p\":%.1f,\"pred_r\":%.1f}",
               samples[i].timestamp, samples[i].pitch, samples[i].roll, pred_pitch, pred_roll);
    }
    printf("]}\n");
}

int main(int argc, char* argv[]) {
    if (argc < 2) {
        fprintf(stderr, "Usage: %s <profile.json> [--optimize]\n", argv[0]);
        fprintf(stderr, "\nGenerates a test profile for development:\n");
        fprintf(stderr, "  %s --generate > test_profile.json\n", argv[0]);
        return 1;
    }

    // Generate synthetic profile for testing
    if (strcmp(argv[1], "--generate") == 0) {
        // Simulate a 5-second walk with synthetic IMU
        float dt = 0.02f;
        int steps = 250;
        DynamicState state = {};
        PhysicalParams true_params;
        true_params.com_offset_x = 0.015f;  // slightly forward-heavy
        true_params.body_mass_kg = 0.55f;

        for (int i = 0; i < steps; i++) {
            float t = i * dt;
            float phase = t * 2 * M_PI * 2; // 2Hz gait
            float angles[8];

            // Trot gait
            float phA = sinf(phase);
            float phB = sinf(phase + M_PI);
            angles[0] = FK_STAND_HIP + 0.22f * phA;
            angles[1] = FK_STAND_KNEE - 0.1f - 0.35f * fabsf(phA);
            angles[2] = FK_STAND_HIP + 0.22f * phB;
            angles[3] = FK_STAND_KNEE - 0.1f - 0.35f * fabsf(phB);
            angles[4] = FK_STAND_HIP + 0.22f * phB;
            angles[5] = FK_STAND_KNEE - 0.1f - 0.35f * fabsf(phB);
            angles[6] = FK_STAND_HIP + 0.22f * phA;
            angles[7] = FK_STAND_KNEE - 0.1f - 0.35f * fabsf(phA);

            float pred_pitch, pred_roll, pred_gz;
            simulate_step(true_params, state, angles, dt, pred_pitch, pred_roll, pred_gz);

            // Add some noise like a real sensor
            float noise_p = ((float)rand() / RAND_MAX - 0.5f) * 1.0f;
            float noise_r = ((float)rand() / RAND_MAX - 0.5f) * 0.8f;

            printf("{\"t\":%.3f,\"pitch\":%.2f,\"roll\":%.2f,\"gz\":%.2f,"
                   "\"angles\":[%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f]}\n",
                   t, pred_pitch + noise_p, pred_roll + noise_r, pred_gz,
                   angles[0], angles[1], angles[2], angles[3],
                   angles[4], angles[5], angles[6], angles[7]);
        }
        return 0;
    }

    // Load captured profile
    auto samples = load_profile(argv[1]);
    if (samples.empty()) {
        fprintf(stderr, "Error: no samples loaded from %s\n", argv[1]);
        return 1;
    }
    fprintf(stderr, "Loaded %zu samples from %s\n", samples.size(), argv[1]);

    PhysicalParams params;
    bool do_optimize = (argc > 2 && strcmp(argv[2], "--optimize") == 0);

    // Evaluate with default parameters
    float error_before = evaluate(params, samples);
    fprintf(stderr, "Default params error: %.4f\n", error_before);

    if (do_optimize) {
        optimize(params, samples);
    }

    float error_after = evaluate(params, samples);
    print_results(params, error_after, samples, do_optimize);

    return 0;
}
