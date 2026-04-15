# IK-Based Gait Pipeline Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the angle-based gait engine with a foot-position IK pipeline, enabling body transforms, active IMU balance, configurable stride parameters, and lean-into-turns — matching the capabilities discovered in the stock MechDog firmware.

**Architecture:** The servo pipeline becomes: gait tick (foot positions) -> body transform offset -> balance correction -> IK solve per leg -> apply servo offsets -> servo_write_us(). All new modules are header-only where possible (shared with native tests). The IK layer is calibrated against the stock firmware's ground-truth standing foot positions.

**Tech Stack:** C++17 (PlatformIO/ESP32), ArduinoJson, ESP32 Preferences (NVS). Tests: native clang++ with existing mock_arduino.h framework.

---

## Context

We reverse-engineered the stock MechDog firmware and discovered the full MechDog API. Key findings:
- Stock firmware works in **foot-position space** — `leg_set_ik(leg, [x,y,z])`, `pose` property returns foot coords
- Standing foot positions: `FL(59.25, 46.0, -80)  FR(-71.25, 46.0, -80)  RL(59.25, -46.0, -80)  RR(-71.25, -46.0, -80)` in mm
- Standing servo pulses confirmed: `[2096, 1621, 2170, 1611, 904, 1379, 830, 1389]`
- `transform([x,y,z], [roll,pitch,yaw], ms)` used extensively for body lean
- `homeostasis(bool)` runs an active IMU->servo balance loop
- `set_gait_params(stride_height, ?, ?)` makes gait tunable
- See `stock_firmware_dump/api_discoveries.md` for full API reference

Our custom firmware currently uses an angle-based sinusoidal gait (`gait_math.h`) that has never been tested on hardware. Balance is a stub (flag set, nothing reads it). No body transform or IK exists.

## File Structure

### New files
| File | Responsibility |
|---|---|
| `firmware/include/ik.h` | Header-only IK: pulse<->angle<->foot position conversions, per-leg FK and IK |
| `firmware/include/body_transform.h` | Header-only body transform: rotation/translation of foot positions |
| `firmware/include/balance.h` | Balance controller interface |
| `firmware/src/balance.cpp` | PID balance implementation |
| `firmware/include/offsets.h` | Servo offset persistence interface |
| `firmware/src/offsets.cpp` | NVS-backed servo offset storage |
| `firmware/test/test_ik.cpp` | IK round-trip and ground-truth tests |
| `firmware/test/test_transform.cpp` | Body transform tests |
| `firmware/test/test_balance.cpp` | Balance convergence tests |
| `firmware/test/mock_preferences.h` | NVS stub for native tests |

### Modified files
| File | Change |
|---|---|
| `firmware/include/gait_math.h` | Add `gait_tick_ik()` returning foot positions instead of angles |
| `firmware/include/gait.h` | Add `GaitConfig`, `gait_set_config()` |
| `firmware/src/gait.cpp` | Replace `gait_update()` pipeline: foot positions -> IK |
| `firmware/src/main.cpp` | Add `cmd_transform`, `cmd_gait_params`, `cmd_offset` handlers; wire balance |
| `firmware/include/protocol.h` | Add new message type constants |
| `firmware/include/config.h` | Add stride height/length defaults |
| `firmware/test/Makefile` | Add new test targets |

---

## Task 1: IK Foundation (`ik.h`)

**Files:**
- Create: `firmware/include/ik.h`
- Create: `firmware/test/test_ik.cpp`
- Modify: `firmware/test/Makefile`

### Coordinate System Reconciliation

The stock firmware and `kinematics.h` use different conventions. We adopt the stock firmware's convention for `ik.h` (since it's our ground truth):
- x: forward (positive = nose direction)
- y: lateral (positive = left side of dog)
- z: vertical (negative = down toward ground)

`kinematics.h` uses x=forward, y=down, z=lateral. The IK module handles this internally.

### Servo Calibration

Each servo has a known standing pulse width and a corresponding standing joint angle. From these two points plus the servo's center (1500us) and range (500-2500us), we derive the linear mapping:

```
pulse_us = standing_us + (angle - standing_angle) * us_per_radian * polarity
```

Polarity and scale are derived per-servo so that `angle_to_pulse(standing_angle)` returns `STANDING_POSE[i]` exactly.

- [ ] **Step 1: Write IK round-trip test**

```cpp
// firmware/test/test_ik.cpp
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
    // Stock: FL(59.25, 46.0, -80) FR(-71.25, 46.0, -80)
    //        RL(59.25, -46.0, -80) RR(-71.25, -46.0, -80)
    const float stock_feet[4][3] = {
        {59.25f, 46.0f, -80.0f},
        {-71.25f, 46.0f, -80.0f},
        {59.25f, -46.0f, -80.0f},
        {-71.25f, -46.0f, -80.0f},
    };
    for (int leg = 0; leg < 4; leg++) {
        FootPos foot = standing_foot_pos(leg);
        float dx = foot.x - stock_feet[leg][0];
        float dy = foot.y - stock_feet[leg][1];
        float dz = foot.z - stock_feet[leg][2];
        float err = sqrtf(dx*dx + dy*dy + dz*dz);
        // Allow 5mm tolerance — coordinate frame may differ slightly
        bool ok = err < 5.0f;
        printf("{\"test\":\"stock_ground_truth\",\"leg\":%d,"
               "\"expected\":[%.1f,%.1f,%.1f],\"got\":[%.1f,%.1f,%.1f],"
               "\"error_mm\":%.2f,\"pass\":%s}\n",
               leg, stock_feet[leg][0], stock_feet[leg][1], stock_feet[leg][2],
               foot.x, foot.y, foot.z, err, ok ? "true" : "false");
        ok ? pass++ : fail++;
    }

    // Test 3: Reachability — points at max extension fail gracefully
    {
        FootPos unreachable = {0, 0, -200};  // way too far down
        uint16_t h, k;
        bool ok = foot_to_pulses(0, unreachable, h, k);
        bool test_pass = !ok;  // should return false
        printf("{\"test\":\"unreachable\",\"pass\":%s}\n",
               test_pass ? "true" : "false");
        test_pass ? pass++ : fail++;
    }

    // Test 4: Workspace sweep — FK(IK(point)) == point for reachable points
    int sweep_pass = 0, sweep_total = 0;
    for (float x = -30; x <= 30; x += 10) {
        for (float z = -60; z <= -100; z -= 10) {
            FootPos target = {x + 60, 46, z};  // near FL standing
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

    printf("{\"summary\":\"ik_tests\",\"pass\":%d,\"fail\":%d}\n", pass, fail);
    return fail > 0 ? 1 : 0;
}
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd firmware/test && make test_ik && ./test_ik`
Expected: Compilation error — `ik.h` does not exist yet.

- [ ] **Step 3: Implement `ik.h`**

Header-only file at `firmware/include/ik.h`. Key contents:

```cpp
#pragma once
#include <math.h>
#include <stdint.h>

// Physical dimensions (mm) — from kinematics.h converted to mm
static constexpr float IK_UPPER_LEN = 55.0f;   // upper leg mm
static constexpr float IK_LOWER_LEN = 60.0f;   // lower leg mm
static constexpr float IK_HIP_X  = 85.0f;      // hip forward offset from body center mm
static constexpr float IK_HIP_Y  = 30.0f;      // hip lateral offset mm
static constexpr float IK_HIP_Z  = -25.0f;     // hip vertical offset mm (below body center)

// Standing angles (radians) — from kinematics.h
static constexpr float IK_STAND_HIP  = 0.524f;
static constexpr float IK_STAND_KNEE = -0.611f;

struct FootPos { float x, y, z; };

// Standing pose pulse widths — from config.h STANDING_POSE
// Order: FL_hip, FL_knee, FR_hip, FR_knee, RL_hip, RL_knee, RR_hip, RR_knee
#ifndef STANDING_POSE
static const uint16_t STANDING_POSE[8] = {
    2096, 1621, 2170, 1611, 904, 1379, 830, 1389
};
#endif

// Per-servo calibration: derived from standing_us and standing_angle
struct ServoCalEntry {
    uint16_t standing_us;
    float standing_angle;  // radians at standing pose
    float us_per_rad;      // pulse width change per radian
    int8_t polarity;       // +1 if increasing angle -> increasing us
};

// Calibration table — computed from STANDING_POSE and FK_STAND angles
// Front servos: standing_us > 1500 -> polarity +1
// Rear servos:  standing_us < 1500 -> polarity -1
inline const ServoCalEntry& servo_cal(uint8_t idx);  // implementation below

// Pulse width <-> joint angle
inline float pulse_to_angle(uint8_t idx, uint16_t us);
inline uint16_t angle_to_pulse(uint8_t idx, float angle_rad);

// Hip offset per leg (mm) in stock firmware coordinate convention
// x=forward, y=lateral (pos=left), z=vertical (neg=down)
inline FootPos hip_position(uint8_t leg);

// Forward kinematics: angles -> foot position (mm, body-relative)
inline FootPos leg_fk_mm(uint8_t leg, float hip_angle, float knee_angle);

// Inverse kinematics: foot position -> angles
// Returns false if unreachable
inline bool leg_ik(uint8_t leg, const FootPos& target, float& hip_out, float& knee_out);

// Convenience: foot position <-> pulse widths
inline bool foot_to_pulses(uint8_t leg, const FootPos& target, uint16_t& hip_us, uint16_t& knee_us);
inline FootPos pulses_to_foot(uint8_t leg, uint16_t hip_us, uint16_t knee_us);

// Standing foot position for each leg
inline FootPos standing_foot_pos(uint8_t leg);
```

Implementation details:
- `servo_cal()` builds a static table on first call. For each servo, polarity is +1 if `standing_us > 1500`, -1 if `standing_us < 1500`. `us_per_rad = abs(standing_us - 1500) / standing_angle` for hips, and `abs(standing_us - 1500) / abs(standing_angle)` for knees.
- `leg_fk_mm()` adapts `kinematics.h`'s `leg_fk()` to mm and the stock coordinate convention.
- `leg_ik()` is standard 2-link planar IK: `d = sqrt(x_local^2 + z_local^2)`, `cos_knee = (d^2 - L1^2 - L2^2) / (2*L1*L2)`, knee = acos, hip = atan2.
- `standing_foot_pos()` calls `leg_fk_mm(leg, IK_STAND_HIP, IK_STAND_KNEE)`.

Note: The stock firmware's asymmetric x values (59.25 vs -71.25) come from the hip offset (+85 for front, -85 for rear) combined with the leg geometry. The FK should reproduce this naturally.

- [ ] **Step 4: Run tests to verify they pass**

Run: `cd firmware/test && make test_ik && ./test_ik`
Expected: All 4 test groups pass. Key: standing round-trip error < 5us, stock ground truth error < 5mm.

- [ ] **Step 5: Add Makefile target**

Add to `firmware/test/Makefile`:
```makefile
test_ik: test_ik.cpp mock_arduino.h ../include/ik.h
	$(CXX) $(CXXFLAGS) -o $@ test_ik.cpp
```
Add `test_ik` to the `TARGETS` list.

- [ ] **Step 6: Commit**

```bash
git add firmware/include/ik.h firmware/test/test_ik.cpp firmware/test/Makefile
git commit -m "feat(firmware): add IK layer with stock firmware ground-truth calibration"
```

---

## Task 2: Body Transform (`body_transform.h`)

**Files:**
- Create: `firmware/include/body_transform.h`
- Create: `firmware/test/test_transform.cpp`
- Modify: `firmware/test/Makefile`

Depends on: Task 1 (ik.h)

- [ ] **Step 1: Write body transform tests**

```cpp
// firmware/test/test_transform.cpp
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

    // Test 2: Height change — z=+10 raises body
    {
        BodyPose up = {0, 0, 10, 0, 0, 0};
        BodyPose down = {0, 0, -10, 0, 0, 0};
        uint16_t p_up[8], p_down[8], p_stand[8];
        body_pose_to_pulses({0,0,0,0,0,0}, p_stand);
        body_pose_to_pulses(up, p_up);
        body_pose_to_pulses(down, p_down);
        // When body rises, knees must extend more -> pulse changes
        bool changed = false;
        for (int i = 0; i < 8; i++) {
            if (p_up[i] != p_stand[i]) changed = true;
        }
        printf("{\"test\":\"height_change\",\"changed\":%s,\"pass\":%s}\n",
               changed ? "true" : "false", changed ? "true" : "false");
        changed ? pass++ : fail++;
    }

    // Test 3: Roll — 5 degrees produces left/right asymmetry
    {
        BodyPose rolled = {0, 0, 0, 5, 0, 0};
        uint16_t p_roll[8], p_stand[8];
        body_pose_to_pulses({0,0,0,0,0,0}, p_stand);
        body_pose_to_pulses(rolled, p_roll);
        // Left knee (idx 1) and right knee (idx 3) should differ from each other
        int left_delta = abs((int)p_roll[1] - (int)p_stand[1]);
        int right_delta = abs((int)p_roll[3] - (int)p_stand[3]);
        bool asym = left_delta > 2 && right_delta > 2;
        printf("{\"test\":\"roll_asymmetry\",\"left_delta\":%d,\"right_delta\":%d,"
               "\"pass\":%s}\n", left_delta, right_delta,
               asym ? "true" : "false");
        asym ? pass++ : fail++;
    }

    // Test 4: Saturation — extreme rotation fails gracefully
    {
        BodyPose extreme = {0, 0, 0, 30, 0, 0};  // 30 degrees roll
        uint16_t p[8];
        bool ok = body_pose_to_pulses(extreme, p);
        printf("{\"test\":\"saturation\",\"reachable\":%s,\"pass\":true}\n",
               ok ? "true" : "false");
        pass++;  // either outcome is valid, just shouldn't crash
    }

    // Test 5: Lerp interpolation
    {
        BodyPose a = {0, 0, 0, 0, 0, 0};
        BodyPose b = {10, 0, 5, 5, 0, 0};
        BodyPose mid = lerp_pose(a, b, 0.5f);
        bool ok = fabsf(mid.dx - 5.0f) < 0.1f &&
                  fabsf(mid.dz - 2.5f) < 0.1f &&
                  fabsf(mid.roll - 2.5f) < 0.1f;
        printf("{\"test\":\"lerp\",\"pass\":%s}\n", ok ? "true" : "false");
        ok ? pass++ : fail++;
    }

    printf("{\"summary\":\"transform_tests\",\"pass\":%d,\"fail\":%d}\n", pass, fail);
    return fail > 0 ? 1 : 0;
}
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd firmware/test && make test_transform && ./test_transform`
Expected: Compilation error — `body_transform.h` does not exist.

- [ ] **Step 3: Implement `body_transform.h`**

Header-only at `firmware/include/body_transform.h`. Key contents:

```cpp
#pragma once
#include "ik.h"
#include <math.h>

struct BodyPose {
    float dx, dy, dz;         // translation mm (x=forward, y=lateral, z=up)
    float roll, pitch, yaw;   // rotation degrees
};

// Rotate a foot position by body roll/pitch/yaw (small angle rotation matrix)
inline FootPos rotate_foot(const FootPos& foot, float roll_deg, float pitch_deg, float yaw_deg);

// Compute all 8 servo pulses for a given body pose offset
// Starts from standing foot positions, applies inverse body transform, solves IK
inline bool body_pose_to_pulses(const BodyPose& pose, uint16_t pulses_out[8]);

// Linear interpolation between two poses
inline BodyPose lerp_pose(const BodyPose& a, const BodyPose& b, float t);
```

Algorithm for `body_pose_to_pulses`:
1. For each leg, get `standing_foot_pos(leg)`.
2. Apply inverse body transform: if body moves forward by `dx`, feet effectively move backward. Subtract body translation from foot positions.
3. Apply inverse body rotation: rotate each foot position by negative roll/pitch/yaw around body center.
4. Solve `leg_ik()` for each transformed foot position.
5. Convert angles to pulse widths via `angle_to_pulse()`.
6. Return false if any leg is unreachable.

- [ ] **Step 4: Run tests to verify they pass**

Run: `cd firmware/test && make test_transform && ./test_transform`
Expected: All tests pass.

- [ ] **Step 5: Update Makefile**

Add `test_transform` target and add to `TARGETS`.

- [ ] **Step 6: Commit**

```bash
git add firmware/include/body_transform.h firmware/test/test_transform.cpp firmware/test/Makefile
git commit -m "feat(firmware): add body transform with rotation/translation via IK"
```

---

## Task 3: Balance Controller

**Files:**
- Create: `firmware/include/balance.h`
- Create: `firmware/src/balance.cpp`
- Create: `firmware/test/test_balance.cpp`
- Modify: `firmware/test/Makefile`

Depends on: Task 2 (body_transform.h)

- [ ] **Step 1: Write balance convergence test**

```cpp
// firmware/test/test_balance.cpp
#include "mock_arduino.h"
#include "../include/ik.h"
#include "../include/body_transform.h"
#include "../include/balance.h"
#include "mock_imu.h"
#include "kinematics.h"
#include <cstdio>
#include <cmath>

int main() {
    int pass = 0, fail = 0;

    // Test 1: Convergence — 5-degree initial tilt corrects within 2 seconds
    {
        BalanceConfig cfg = {
            0.3f, 0.01f, 0.05f,  // kp, ki, kd for pitch
            0.3f, 0.01f, 0.05f,  // kp, ki, kd for roll
            8.0f,                 // max correction degrees
            0.3f                  // deadband degrees
        };
        balance_init(cfg);
        balance_enable(true);

        float pitch = 5.0f;  // initial disturbance
        float roll = 0.0f;
        float dt = 0.02f;    // 50 Hz
        int steps = 100;     // 2 seconds

        for (int i = 0; i < steps; i++) {
            BodyPose correction = balance_update(pitch, roll, dt);
            // Simulate: correction reduces error (simplified plant model)
            pitch -= correction.pitch * 0.5f;
            roll -= correction.roll * 0.5f;
        }

        bool converged = fabsf(pitch) < 1.0f;
        printf("{\"test\":\"convergence\",\"final_pitch\":%.2f,"
               "\"pass\":%s}\n", pitch, converged ? "true" : "false");
        converged ? pass++ : fail++;
    }

    // Test 2: Deadband — small errors produce zero output
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
        bool clamped = fabsf(out.pitch) <= 5.1f;  // within max + epsilon
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
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd firmware/test && make test_balance && ./test_balance`
Expected: Compilation error — `balance.h` does not exist.

- [ ] **Step 3: Implement `balance.h` and `balance.cpp`**

`balance.h` — interface:
```cpp
#pragma once
#include "body_transform.h"

struct BalanceConfig {
    float kp_pitch, ki_pitch, kd_pitch;
    float kp_roll,  ki_roll,  kd_roll;
    float max_correction_deg;
    float deadband_deg;
};

void balance_init(const BalanceConfig& config);
void balance_enable(bool enabled);
bool balance_is_enabled();
BodyPose balance_update(float pitch_deg, float roll_deg, float dt);
void balance_reset();
```

`balance.cpp` — PID with anti-windup:
- Stores pitch/roll error integral and previous error.
- Deadband: if `abs(error) < deadband`, treat as zero.
- Anti-windup: clamp integral so that `ki * integral` never exceeds `max_correction_deg`.
- Output: `BodyPose` with only `pitch` and `roll` fields set (corrections for the body transform layer).
- Returns zero-initialized `BodyPose` when disabled.
- `balance_reset()` clears integrator and previous error — call on mode transitions (walk<->stand).

Note: `balance.cpp` has NO Arduino dependencies. It's pure math. It compiles on host and firmware. Consider keeping it header-only, but it has mutable state (integrators), so a .cpp is cleaner.

- [ ] **Step 4: Run tests to verify they pass**

Run: `cd firmware/test && make test_balance && ./test_balance`
Expected: All 4 tests pass.

- [ ] **Step 5: Update Makefile and commit**

```bash
git add firmware/include/balance.h firmware/src/balance.cpp \
       firmware/test/test_balance.cpp firmware/test/Makefile
git commit -m "feat(firmware): add PID balance controller with deadband and anti-windup"
```

---

## Task 4: Servo Offset Persistence

**Files:**
- Create: `firmware/include/offsets.h`
- Create: `firmware/src/offsets.cpp`
- Create: `firmware/test/mock_preferences.h`
- Modify: `firmware/src/main.cpp`
- Modify: `firmware/include/protocol.h`

Independent — can run in parallel with Tasks 1-3.

- [ ] **Step 1: Create mock_preferences.h**

```cpp
// firmware/test/mock_preferences.h
#pragma once
#include <cstring>
#include <cstdint>

// Minimal Preferences stub for native tests
class Preferences {
    int16_t store[8] = {};
    bool opened = false;
public:
    bool begin(const char*, bool = false) { opened = true; return true; }
    void end() { opened = false; }
    int16_t getShort(const char* key, int16_t def = 0) {
        int idx = key[3] - '0';  // "off0"-"off7"
        return (idx >= 0 && idx < 8) ? store[idx] : def;
    }
    size_t putShort(const char* key, int16_t val) {
        int idx = key[3] - '0';
        if (idx >= 0 && idx < 8) store[idx] = val;
        return 2;
    }
    bool clear() { memset(store, 0, sizeof(store)); return true; }
};
```

- [ ] **Step 2: Implement `offsets.h` and `offsets.cpp`**

`offsets.h`:
```cpp
#pragma once
#include <stdint.h>

void offsets_init();
int16_t offset_get(uint8_t servo_idx);
void offset_set(uint8_t servo_idx, int16_t offset_us);
void offsets_save();
void offsets_reset();
uint16_t apply_offset(uint8_t servo_idx, uint16_t raw_us);
```

`offsets.cpp`:
- Uses `<Preferences.h>` on ESP32 or `mock_preferences.h` in tests.
- Namespace `"servo_cal"`, keys `"off0"` through `"off7"`.
- `offsets_init()` loads all 8 offsets from NVS into a static array.
- `apply_offset()` returns `raw_us + offset_get(idx)` clamped to 500-2500.

- [ ] **Step 3: Add `cmd_offset` handler to `main.cpp`**

Add to `protocol.h`:
```cpp
constexpr const char* MSG_CMD_OFFSET = "cmd_offset";
```

Add to `handle_message()` in `main.cpp`:
```cpp
else if (strcmp(type, MSG_CMD_OFFSET) == 0) {
    const char* action = doc["action"] | "read";
    if (strcmp(action, "set") == 0) {
        uint8_t idx = doc["index"] | 0;
        int16_t val = doc["value"] | 0;
        offset_set(idx, val);
    } else if (strcmp(action, "save") == 0) {
        offsets_save();
    } else if (strcmp(action, "reset") == 0) {
        offsets_reset();
    }
    // Always respond with current offsets
    JsonDocument resp;
    resp["type"] = MSG_ACK;
    resp["ref_type"] = MSG_CMD_OFFSET;
    resp["ok"] = true;
    JsonArray arr = resp["offsets"].to<JsonArray>();
    for (int i = 0; i < 8; i++) arr.add(offset_get(i));
    send_json(resp);
}
```

Call `offsets_init()` in `setup()` before `servos_init()`.

- [ ] **Step 4: Commit**

```bash
git add firmware/include/offsets.h firmware/src/offsets.cpp \
       firmware/test/mock_preferences.h \
       firmware/src/main.cpp firmware/include/protocol.h
git commit -m "feat(firmware): add servo offset persistence via ESP32 NVS"
```

---

## Task 5: Gait Refactor — IK-Based Pipeline

**Files:**
- Modify: `firmware/include/gait_math.h` — add `gait_tick_ik()`
- Modify: `firmware/include/gait.h` — add `GaitConfig`, `gait_set_config()`
- Modify: `firmware/src/gait.cpp` — rewrite `gait_update()` to use IK pipeline
- Modify: `firmware/include/config.h` — add stride defaults
- Modify: `firmware/include/protocol.h` — add new message types
- Modify: `firmware/src/main.cpp` — add `cmd_transform`, `cmd_gait_params` handlers, wire balance
- Modify: `firmware/test/Makefile`

Depends on: Tasks 1-3

This is the largest and riskiest task. It replaces the core servo pipeline.

- [ ] **Step 1: Add `GaitConfig` and `gait_tick_ik()` to gait_math.h**

Add after the existing `gait_tick()` (keep old one for reference):

```cpp
struct GaitConfig {
    float stride_length_mm;  // forward/back swing (default 20)
    float stride_height_mm;  // foot lift height (default 15)
    float frequency_hz;      // step frequency (default 1.5)
};

struct GaitFootPositions {
    FootPos feet[4];  // FL, FR, RL, RR — offsets from standing foot positions
};

// Foot-position gait tick: returns foot position OFFSETS from standing
// Caller adds these to standing_foot_pos() before IK
inline GaitFootPositions gait_tick_ik(float phase_rad, GaitDir dir,
                                       const GaitConfig& config, float speed);
```

Implementation:
- Each foot traces an elliptical trajectory in the sagittal plane.
- **Stance phase** (foot on ground, `sin(phase) < 0` for this leg): foot slides backward linearly. `dx = -stride_length * (phase_fraction) * speed * dir_sign`, `dz = 0`.
- **Swing phase** (foot in air, `sin(phase) > 0`): foot lifts in half-sine and swings forward. `dx = stride_length * (1 - phase_fraction) * speed * dir_sign`, `dz = stride_height * sin(swing_fraction * pi)`.
- Diagonal pairing preserved: FL+RR share phase, FR+RL share anti-phase.
- Turning: differential stride length (slow side 0.3x) instead of differential amplitude.

- [ ] **Step 2: Add `GaitConfig` to `gait.h`**

```cpp
#pragma once
#include "ik.h"
#include "body_transform.h"

enum class GaitState {
    STOP, STAND, WALK_FORWARD, WALK_BACKWARD, TURN_LEFT, TURN_RIGHT
};

void gait_init();
void gait_set_state(GaitState state, float speed = 1.0f);
void gait_set_config(const GaitConfig& config);
void gait_set_body_transform(const BodyPose& pose, uint16_t duration_ms);
GaitState gait_current_state();
void gait_update(unsigned long now_ms);
```

- [ ] **Step 3: Rewrite `gait_update()` in `gait.cpp`**

The new pipeline in `gait_update()`:

```cpp
void gait_update(unsigned long now_ms) {
    // ... idle timeout, dt computation (keep existing) ...

    if (state == STAND || state == STOP) {
        // Interpolate body transform to target, compute standing + transform via IK
        BodyPose combined = current_body_transform;
        if (balance_is_enabled()) {
            BodyPose bal = balance_update(current_pitch, current_roll, dt);
            combined.pitch += bal.pitch;
            combined.roll += bal.roll;
        }
        uint16_t pulses[8];
        if (body_pose_to_pulses(combined, pulses)) {
            for (int i = 0; i < 8; i++)
                servo_write_us(i, apply_offset(i, pulses[i]));
        }
        return;
    }

    // Advance phase
    phase += 2.0f * M_PI * config.frequency_hz * speed * dt;
    if (phase > 2.0f * M_PI) phase -= 2.0f * M_PI;

    // Get foot position offsets from gait
    GaitDir gdir = /* map state to dir */;
    GaitFootPositions gait_feet = gait_tick_ik(phase, gdir, config, speed);

    // Build foot targets: standing + gait offset + body transform
    BodyPose combined = current_body_transform;
    if (balance_is_enabled()) {
        BodyPose bal = balance_update(current_pitch, current_roll, dt);
        combined.pitch += bal.pitch;
        combined.roll += bal.roll;
    }

    for (int leg = 0; leg < 4; leg++) {
        FootPos foot = standing_foot_pos(leg);
        // Add gait offset
        foot.x += gait_feet.feet[leg].x;
        foot.y += gait_feet.feet[leg].y;
        foot.z += gait_feet.feet[leg].z;
        // Apply body transform (rotation + translation)
        foot = rotate_foot(foot, -combined.roll, -combined.pitch, -combined.yaw);
        foot.x -= combined.dx;
        foot.y -= combined.dy;
        foot.z -= combined.dz;
        // IK solve
        uint16_t hip_us, knee_us;
        if (foot_to_pulses(leg, foot, hip_us, knee_us)) {
            servo_write_us(leg * 2,     apply_offset(leg * 2,     hip_us));
            servo_write_us(leg * 2 + 1, apply_offset(leg * 2 + 1, knee_us));
        }
    }
}
```

Key: Remove `SERVO_POLARITY` array and `angle_to_us()` — polarity is now handled inside `ik.h`'s `angle_to_pulse()`.

- [ ] **Step 4: Add `cmd_transform` and `cmd_gait_params` to `main.cpp`**

Add to `protocol.h`:
```cpp
constexpr const char* MSG_CMD_TRANSFORM   = "cmd_transform";
constexpr const char* MSG_CMD_GAIT_PARAMS = "cmd_gait_params";
```

Add handlers in `handle_message()`:

```cpp
else if (strcmp(type, MSG_CMD_TRANSFORM) == 0) {
    BodyPose pose;
    pose.dx    = doc["x"]     | 0.0f;
    pose.dy    = doc["y"]     | 0.0f;
    pose.dz    = doc["z"]     | 0.0f;
    pose.roll  = doc["roll"]  | 0.0f;
    pose.pitch = doc["pitch"] | 0.0f;
    pose.yaw   = doc["yaw"]   | 0.0f;
    uint16_t ms = doc["ms"]   | 100;
    gait_set_body_transform(pose, ms);
    send_ack(MSG_CMD_TRANSFORM, true);
}
else if (strcmp(type, MSG_CMD_GAIT_PARAMS) == 0) {
    GaitConfig cfg;
    cfg.stride_height_mm = doc["stride_height"] | 15.0f;
    cfg.stride_length_mm = doc["stride_length"] | 20.0f;
    cfg.frequency_hz     = doc["frequency"]     | 1.5f;
    gait_set_config(cfg);
    send_ack(MSG_CMD_GAIT_PARAMS, true);
}
```

- [ ] **Step 5: Wire balance to IMU in main.cpp**

In the `handle_message` for `cmd_balance`:
```cpp
else if (strcmp(type, MSG_CMD_BALANCE) == 0) {
    balance_enabled = doc["enabled"] | true;
    balance_enable(balance_enabled);
    if (!balance_enabled) balance_reset();
    send_ack(MSG_CMD_BALANCE, true);
}
```

Pass IMU data to gait: add a global `current_pitch`/`current_roll` updated in the IMU streaming section, read by `gait_update()`.

- [ ] **Step 6: Add stride defaults to `config.h`**

```cpp
// --- Gait Parameters (IK-based) ---
#define GAIT_STRIDE_HEIGHT_MM   15.0f   // foot lift height (stock default)
#define GAIT_STRIDE_LENGTH_MM   20.0f   // forward/back swing
#define GAIT_FREQUENCY_HZ       1.5f    // steps per second
```

- [ ] **Step 7: Update existing gait tests**

Modify `firmware/test/main.cpp` to test `gait_tick_ik()`:
- Foot clearance test: verify `feet[leg].z >= stride_height - epsilon` during swing phase.
- Symmetry test: diagonal pairs (FL+RR, FR+RL) should have equal but anti-phase trajectories.
- Full pipeline test: gait + transform + balance -> all 4 legs produce valid IK solutions at every phase step.

Run: `cd firmware/test && make gait_tests && ./gait_tests`

- [ ] **Step 8: Verify PlatformIO builds**

Run: `cd firmware && pio run`
Expected: Clean compile. This validates that all new headers work on ESP32 toolchain (not just clang).

- [ ] **Step 9: Commit**

```bash
git add firmware/include/gait_math.h firmware/include/gait.h \
       firmware/src/gait.cpp firmware/src/main.cpp \
       firmware/include/protocol.h firmware/include/config.h \
       firmware/test/main.cpp firmware/test/Makefile
git commit -m "feat(firmware): replace angle-based gait with IK foot-position pipeline

Adds cmd_transform, cmd_gait_params commands. Wires balance PID to
IMU for active stabilization. Gait now works in foot-position space
with configurable stride height/length."
```

---

## Task 6: Integration Verification

- [ ] **Step 1: Run all native tests**

```bash
cd firmware/test
make clean && make all
./test_ik
./test_transform
./test_balance
./gait_tests all
```

All tests must pass.

- [ ] **Step 2: PlatformIO build**

```bash
cd firmware
pio run
```

Must compile cleanly.

- [ ] **Step 3: Flash and basic smoke test** (when RR servo replaced)

Flash via USB:
```bash
cd firmware && pio run -t upload
```

Verify:
1. Boot telemetry appears (lavender LEDs, `{"type":"boot",...}`)
2. `cmd_stand` produces stable standing pose
3. `cmd_balance` with enabled=true shows IMU-reactive servo adjustments
4. `cmd_transform` with small values (z=5, roll=3) visibly changes posture
5. `cmd_gait_params` with stride_height=20 accepted
6. `cmd_move` forward produces walking gait

- [ ] **Step 4: Final commit with any fixes**

---

## Risks and Mitigations

| Risk | Mitigation |
|---|---|
| Servo polarity wrong in IK calibration | Round-trip test (Task 1) catches this before any hardware is involved |
| Stock firmware coordinate system mismatch | Ground-truth test against stock `pose` values validates the mapping |
| Gait refactor breaks existing walking | Old `gait_tick()` stays in `gait_math.h` for reference; full native test suite validates new pipeline |
| Balance PID gains wrong for real hardware | Gains are configurable via `BalanceConfig`; start conservative (low kp, no ki), tune on hardware |
| IK unreachable during gait + transform combo | Pipeline checks `foot_to_pulses()` return value; on failure, holds last valid position |
| RR servo still blown | All development and testing is native (no hardware needed). Add a `SKIP_LEG_RR` flag if needed for 3-leg testing |
