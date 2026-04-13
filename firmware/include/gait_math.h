#pragma once
// No Arduino / hardware includes — compiles on host and firmware alike.
#include <math.h>
#include <stdint.h>
#include "ik.h"

// ============================================================
// Gait Math — header-only sinusoidal trot kernel
//
// Implements the diagonal-pair trot used by Bark-Buddy custom
// firmware ("Mutt"). Shared between:
//   firmware/src/gait.cpp     — servo angle computation
//   firmware/test/            — native unit tests
//   host/sim/physics.py       — mirrored as gait_tick()
//
// Python mirror: host/sim/physics.py :: gait_tick()
// Keep in sync: phase convention (radians, 0 → 2π), amplitude
// units (degrees), diagonal pairing (FL+RR = sinA, FR+RL = sinB),
// turning strategy (differential amplitude, not opposite-side sign).
// ============================================================

// Leg indices — FL=0, FR=1, RL=2, RR=3.
// Matches host/sim/physics.py FL/FR/RL/RR constants.
static constexpr uint8_t GAIT_FL = 0;
static constexpr uint8_t GAIT_FR = 1;
static constexpr uint8_t GAIT_RL = 2;
static constexpr uint8_t GAIT_RR = 3;

// Per-leg angle offsets from standing pose (degrees).
struct GaitAngles {
    float hip[4];   // FL, FR, RL, RR
    float knee[4];
};

// Movement direction — subset of GaitState (excludes STOP / STAND).
enum class GaitDir : uint8_t {
    FORWARD    = 0,
    BACKWARD   = 1,
    TURN_LEFT  = 2,
    TURN_RIGHT = 3,
};

// Compute per-leg hip/knee angle offsets for one gait tick.
//
// phase_rad   gait phase in [0, 2π); caller advances by 2π * freq * dt
// dir         movement direction
// hip_amp     hip swing amplitude (degrees, from GAIT_HIP_AMPLITUDE)
// knee_amp    knee lift amplitude (degrees, from GAIT_KNEE_AMPLITUDE)
// speed       speed scale [0, 1]
//
// Returns offsets from standing pose. Caller applies servo polarity
// and converts to pulse width via angle_to_us() in gait.cpp.
//
// Diagonal pairing: FL+RR share sinA; FR+RL share sinB (= sinA shifted π).
// Turning: differential amplitude — slow side 0.3×, fast side 1.0×.
// Both sides still swing forward; the arc comes from the speed difference.
// This differs from the sim's GaitGenerator which uses opposite-sign turns;
// see host/sim/physics.py for the known divergence note.
inline GaitAngles gait_tick(float phase_rad, GaitDir dir,
                             float hip_amp, float knee_amp,
                             float speed = 1.0f)
{
    GaitAngles out = {};

    const float sinA = sinf(phase_rad);
    const float sinB = sinf(phase_rad + (float)M_PI);

    const float dir_sign  = (dir == GaitDir::BACKWARD)   ? -1.0f : 1.0f;
    const float left_mul  = (dir == GaitDir::TURN_LEFT)  ?  0.3f : 1.0f;
    const float right_mul = (dir == GaitDir::TURN_RIGHT) ?  0.3f : 1.0f;

    const float ha = hip_amp  * speed;
    const float ka = knee_amp * speed;

    // Knee lifts only during positive (swing) phase — fmaxf(0, sin).
    out.hip[GAIT_FL]  =  dir_sign * ha * sinA * left_mul;
    out.knee[GAIT_FL] = -ka * fmaxf(0.0f, sinA);

    out.hip[GAIT_FR]  =  dir_sign * ha * sinB * right_mul;
    out.knee[GAIT_FR] = -ka * fmaxf(0.0f, sinB);

    out.hip[GAIT_RL]  =  dir_sign * ha * sinB * left_mul;
    out.knee[GAIT_RL] = -ka * fmaxf(0.0f, sinB);

    out.hip[GAIT_RR]  =  dir_sign * ha * sinA * right_mul;
    out.knee[GAIT_RR] = -ka * fmaxf(0.0f, sinA);

    return out;
}

// ============================================================
// IK-based gait — foot position offsets from standing pose
// ============================================================

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

        float dx, dz;
        if (s >= 0.0f) {
            // Swing phase: foot sweeps forward, lifts in half-sine arc
            // Map s from [0,1] to swing progress
            dz = sh * s;   // lift follows sinusoid — peaks at ph=π/2
            dx = dir_sign * sl * mul * (1.0f - s);  // sweeping back from peak
        } else {
            // Stance phase: foot on ground, pushes backward linearly
            dz = 0.0f;
            dx = dir_sign * sl * mul * s;  // s is negative → positive push
        }

        out.feet[i].x = dx;
        out.feet[i].y = 0.0f;   // no lateral foot movement
        out.feet[i].z = dz;
    }

    return out;
}
