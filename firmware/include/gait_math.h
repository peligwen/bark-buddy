#pragma once
// No Arduino / hardware includes — compiles on host and firmware alike.
#include <math.h>
#include <stdint.h>
#include "ik.h"

// ============================================================
// Gait Math — header-only AEP/PEP trot kernel
//
// Implements the diagonal-pair trot used by Bark-Buddy custom
// firmware. Shared between:
//   firmware/src/gait.cpp     — foot-position computation
//   firmware/test/            — native unit tests
//
// AEP (Anterior Extreme Position): most-forward foot position.
// PEP (Posterior Extreme Position): most-backward foot position.
//
// Stance: foot sweeps AEP→PEP linearly at z=0 (ground contact).
// Swing:  foot sweeps PEP→AEP with sin² lift profile — C¹,
//         zero velocity at lift-off and touchdown.
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
    float    stride_length_mm;          // forward/back foot swing (default 12mm)
    float    stride_height_mm;          // foot lift height above standing (default 10mm)
    float    frequency_hz;              // step frequency (default 1.5Hz)
    uint32_t swing_time_ms = 150;       // ms per swing phase — sets duty factor
    uint32_t stand_time_ms = 200;       // ms per stance phase — sets duty factor
};

struct GaitFootOffsets {
    FootPos feet[4];  // FL, FR, RL, RR — offsets from standing_foot_pos()
};

// C¹-smooth ease-in-out: zero velocity at both endpoints (derivative = 6t(1-t)).
inline float smoothstep(float t) {
    if (t <= 0.0f) return 0.0f;
    if (t >= 1.0f) return 1.0f;
    return t * t * (3.0f - 2.0f * t);
}

// Stance: linear sweep from AEP to PEP at ground level.
// percent ∈ [0,1]: 0 = at AEP, 1 = at PEP.
inline FootPos stance_process(const FootPos& aep, const FootPos& pep, float percent) {
    return {
        aep.x + (pep.x - aep.x) * percent,
        aep.y + (pep.y - aep.y) * percent,
        0.0f
    };
}

// Swing: arc from PEP to AEP with sin² vertical lift.
// percent ∈ [0,1]: 0 = at PEP (lift-off), 1 = at AEP (touch-down).
// sin²(π·t) profile: C¹ with zero vertical velocity at both endpoints.
inline FootPos swing_process(const FootPos& pep, const FootPos& aep,
                             float percent, float height_mm) {
    float sp = sinf(percent * (float)M_PI);
    return {
        pep.x + (aep.x - pep.x) * percent,
        pep.y + (aep.y - pep.y) * percent,
        height_mm * sp * sp
    };
}

// Foot-position gait tick: returns foot position OFFSETS from standing positions.
// Caller adds these to standing_foot_pos(leg) before IK solve.
//
// Phase convention: [0, 2π). Diagonal pairs: FL+RR share phase_A, FR+RL share phase_A+π.
// Duty factor = stand_time / (swing_time + stand_time).
// [0, duty):  stance — AEP→PEP, foot at ground.
// [duty, 1):  swing  — PEP→AEP, foot lifts with sin² profile.
//
// Turning: differential stride length — slow side 0.3×.
inline GaitFootOffsets gait_tick_ik(float phase_rad, GaitDir dir,
                                     const GaitConfig& config, float speed)
{
    GaitFootOffsets out = {};

    float sl      = config.stride_length_mm * speed;
    float sh      = config.stride_height_mm;
    float sl_half = sl * 0.5f;

    float dir_sign  = (dir == GaitDir::BACKWARD)   ? -1.0f : 1.0f;
    float left_mul  = (dir == GaitDir::TURN_LEFT)  ?  0.3f : 1.0f;
    float right_mul = (dir == GaitDir::TURN_RIGHT) ?  0.3f : 1.0f;

    // Duty factor: fraction of period in stance
    uint32_t total_ms = config.swing_time_ms + config.stand_time_ms;
    float duty = (total_ms > 0) ? (float)config.stand_time_ms / (float)total_ms : 0.5f;

    // Per-leg normalised phase [0,1): FL+RR=phA, FR+RL=phB (0.5 offset)
    float phA = fmodf(phase_rad / (2.0f * (float)M_PI), 1.0f);
    if (phA < 0.0f) phA += 1.0f;
    float phB = fmodf(phA + 0.5f, 1.0f);

    struct LegParams { float ph; float mul; } legs[4] = {
        {phA, left_mul},   // FL
        {phB, right_mul},  // FR
        {phB, left_mul},   // RL
        {phA, right_mul},  // RR
    };

    for (int i = 0; i < 4; i++) {
        float p   = legs[i].ph;
        float mul = legs[i].mul;

        FootPos aep = { dir_sign * sl_half * mul, 0.0f, 0.0f };
        FootPos pep = {-dir_sign * sl_half * mul, 0.0f, 0.0f };

        FootPos offset;
        if (p < duty) {
            float pct = (duty > 0.0f) ? p / duty : 0.0f;
            offset = stance_process(aep, pep, pct);
        } else {
            float pct = (duty < 1.0f) ? (p - duty) / (1.0f - duty) : 0.0f;
            offset = swing_process(pep, aep, pct, sh);
        }

        out.feet[i] = offset;
    }

    return out;
}
