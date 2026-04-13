#pragma once
// No Arduino / hardware includes — compiles on host and firmware alike.
#include <math.h>
#include <stdint.h>
#include "ik.h"

// ============================================================
// Gait Math — header-only IK foot-position trot kernel
//
// Implements the diagonal-pair trot used by Bark-Buddy custom
// firmware. Shared between:
//   firmware/src/gait.cpp     — foot-position computation
//   firmware/test/            — native unit tests
// ============================================================

// Leg indices — FL=0, FR=1, RL=2, RR=3.
// Matches host/sim/physics.py FL/FR/RL/RR constants.
static constexpr uint8_t GAIT_FL = 0;
static constexpr uint8_t GAIT_FR = 1;
static constexpr uint8_t GAIT_RL = 2;
static constexpr uint8_t GAIT_RR = 3;

// Movement direction — subset of GaitState (excludes STOP / STAND).
enum class GaitDir : uint8_t {
    FORWARD    = 0,
    BACKWARD   = 1,
    TURN_LEFT  = 2,
    TURN_RIGHT = 3,
};


struct GaitConfig {
    float stride_length_mm;  // forward/back foot swing (default 20mm)
    float stride_height_mm;  // foot lift height above standing (default 15mm)
    float frequency_hz;      // step frequency (default 1.5Hz)
};

struct GaitFootOffsets {
    FootPos feet[4];  // FL, FR, RL, RR — offsets from standing_foot_pos()
};

// Foot-position gait tick: returns foot position OFFSETS from standing positions.
// Caller adds these to standing_foot_pos(leg) before IK solve.
//
// Phase convention: [0, 2π). Diagonal pairs: FL+RR share phase, FR+RL share phase+π.
// Swing phase (sin > 0): foot sweeps forward + lifts. Stance (sin < 0): foot pushes back.
// Turning: differential stride length — slow side 0.3×.
inline GaitFootOffsets gait_tick_ik(float phase_rad, GaitDir dir,
                                     const GaitConfig& config, float speed)
{
    GaitFootOffsets out = {};

    // Phase for each diagonal pair
    float phA = phase_rad;               // FL + RR
    float phB = phase_rad + (float)M_PI; // FR + RL

    float dir_sign   = (dir == GaitDir::BACKWARD)   ? -1.0f : 1.0f;
    float left_mul   = (dir == GaitDir::TURN_LEFT)  ?  0.3f : 1.0f;
    float right_mul  = (dir == GaitDir::TURN_RIGHT) ?  0.3f : 1.0f;

    float sl = config.stride_length_mm * speed;
    float sh = config.stride_height_mm;

    // Per-leg phase and multipliers
    struct LegParams { float ph; float mul; } legs[4] = {
        {phA, left_mul},   // FL
        {phB, right_mul},  // FR
        {phB, left_mul},   // RL
        {phA, right_mul},  // RR
    };

    for (int i = 0; i < 4; i++) {
        float ph = legs[i].ph;
        float mul = legs[i].mul;
        float s = sinf(ph);

        // Smooth cosine x-sweep: continuous at all phase transitions.
        // -cos(ph) maps: ph=0→-1 (back), ph=π/2→0 (center), ph=π→+1 (front)
        // dir_sign flips for backward motion; mul scales for turning.
        float dx = dir_sign * sl * mul * (-cosf(ph));

        // Lift only during swing phase (sin > 0), smooth sinusoidal arc.
        float dz = (s > 0.0f) ? sh * s : 0.0f;

        out.feet[i].x = dx;
        out.feet[i].y = 0.0f;
        out.feet[i].z = dz;
    }

    return out;
}
