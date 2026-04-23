# Gait Pipeline Failure Mode Reference

**Date**: 2026-04-22
**Purpose**: Comprehensive failure mode analysis for the Bark-Buddy quadruped gait pipeline, synthesized from structured per-layer analyses of all seven pipeline layers. Intended as a living troubleshooting and test-planning reference.

---

## How to Use This Document

**Troubleshooting workflow:**

1. Identify the observable symptom on the robot or in telemetry.
2. Look up the symptom in the [Symptom → Likely Layer Lookup Table](#4-symptom--likely-layer-lookup-table).
3. Note the candidate failure mode IDs and the suggested quick check.
4. Jump to the per-layer section for each candidate ID and follow the test strategy.
5. If the symptom spans multiple layers, consult the [Cross-Layer Cascade Chains](#3-cross-layer-cascade-chains) section.

---

## Scope

**Covered:**
- L1: `firmware/include/gait_math.h` — AEP/PEP trot kernel (header-only math)
- L2: `firmware/include/ik.h` / `firmware/include/body_transform.h` — inverse kinematics, body pose
- L3: `firmware/src/gait.cpp` — gait state machine, 50 Hz update loop
- L4: `firmware/include/balance.h` / `firmware/src/balance.cpp` — firmware PID balance controller
- L5: `host/behaviors/balance.py` — host-side fall detection monitor
- L6: `host/server.py` / `host/dog/` / `host/lock.py` — Python host transport layer
- L7: `web/` — browser UI (vanilla JS, ES modules)

**Out of scope:**
- Sonar, camera, and vision module subsystems
- OTA firmware update path (`host/ota.py`)
- LED and buzzer state machines
- WiFi provisioning and mDNS discovery internals
- SLAM/localization (planned, not yet implemented)

---

## Glossary

| Term | Definition |
|---|---|
| AEP | Anterior Extreme Position — the forward foot placement boundary in the trot kernel |
| PEP | Posterior Extreme Position — the rear foot placement boundary in the trot kernel |
| duty | Fraction of one gait cycle spent in stance phase (`stand_time_ms / (swing_time_ms + stand_time_ms)`) |
| IK | Inverse Kinematics — computing joint angles from a desired foot position |
| FK | Forward Kinematics — computing foot position from joint angles |
| `s_phase` | Gait phase accumulator in `gait.cpp`, range `[0, 2π)` |
| `telem_imu` | Firmware telemetry message carrying pitch/roll/yaw in degrees |
| `telem_status` | Firmware telemetry message carrying engage state, balance state, gait state |
| `tilt_fault` | Firmware event emitted when `|pitch|` or `|roll|` exceeds `BALANCE_TILT_CUTOFF_DEG` (50°) |
| `cmd_move` | Host → firmware command to set gait direction and speed |
| `cmd_engage` | Host → firmware command to engage or disengage servo power |
| `cmd_balance` | Host → firmware command to enable or disable the balance PID |
| `cmd_servo` | Host → firmware command to write a raw pulse width directly to one servo |
| `cmd_transform` | Host → firmware command to set a body pose offset |
| ControlLock | Server-side resource (`host/lock.py`) that serializes control from multiple browser tabs |
| STAND_RETURN_MS | 600 ms — duration of the return-to-standing taper on gait stop |
| SOFTSTART_DURATION_MS | 2000 ms — duration of the engage torque ramp |
| `gait_update_imu()` | Firmware function that updates `s_pitch`/`s_roll` used by tilt safety and balance PID |
| LP_TAU | 0.2 s — first-order low-pass filter time constant in mock physics model |
| us\_per\_rad | Servo calibration constant: microseconds per radian, derived from standing pulse deviation |

---

## 1. Layer 1: Gait Math (`gait_math.h`)

### L1-F01: Zero Total Time — Division Guard Returns Wrong Duty
**Severity**: degraded
**Mechanism**: In `gait_tick_ik`, `total_ms = config.swing_time_ms + config.stand_time_ms`. When both are zero the guard `(total_ms > 0)` fires and `duty` falls back to `0.5f`. A caller setting both to zero likely intends a stopped state, not a 50% duty trot. No assertion or log fires; the robot silently trots at 50% duty.
**Symptoms**: Robot walks when operator intends it to be stopped. Stride height is normal (not scaled), but all four feet cycle at 50% duty at whatever `frequency_hz` is set.
**Test strategy**: Call `gait_tick_ik(0.0f, GaitDir::FORWARD, {0, 0, 1.5f, 0, 0}, 1.0f)` with `swing_time_ms=0` and `stand_time_ms=0`. Verify that `duty=0.5` is used and that non-zero foot offsets are produced.
**Notes**: In practice `GaitConfig` defaults in `config.h` prevent this (`GAIT_SWING_TIME_MS=150`, `GAIT_STAND_TIME_MS=200`), but any path that zero-initializes a `GaitConfig` struct (e.g., a partially constructed command) can trigger it.
**Cross-references**: see L3-F11 (config blend can transiently produce zero values during ramp).

---

### L1-F02: All Legs Permanently in Stance — swing_time_ms = 0
**Severity**: degraded
**Mechanism**: With `swing_time_ms=0` and `stand_time_ms>0`, `duty = 1.0f`. Every per-leg phase `p ∈ [0,1)` satisfies `p < duty = 1.0`, so every leg stays in the stance branch forever. The swing branch `(p >= duty)` is unreachable.
**Symptoms**: All four feet drag along the ground with linear x-offsets; foot z = 0 always (never lifts). Robot shuffles without clearance — high scraping on any terrain, elevated servo current from dragging. Mock `foot_z` equals `standing_foot_pos(leg).z` constantly.
**Test strategy**: `gait_tick_ik(phase, GaitDir::FORWARD, {12.0f, 10.0f, 1.5f, 0, 200}, 1.0f)` — sweep phase 0→2π. Assert all `feet[i].z == 0.0f` throughout. Confirm `max_z == 0` (no swing).
**Notes**: The `config.h` default `GAIT_SWING_TIME_MS=150` prevents this at startup. Runtime parameter blending in `gait.cpp` could transiently pass through `swing_time_ms=0` if rounding truncates via `(uint32_t)(st + 0.5f)`.
**Cross-references**: see L1-F01, L3-F11.

---

### L1-F03: All Legs Simultaneously Airborne — stand_time_ms = 0
**Severity**: unsafe
**Mechanism**: With `stand_time_ms=0`, `duty = 0.0f`. Every phase `p ∈ [0,1)` satisfies `p >= duty = 0.0`, so all four legs are in the swing branch simultaneously. All four feet lift at the same time — no ground support.
**Symptoms**: Robot immediately collapses. All four foot z-offsets are positive (in the air). Mock `foot_z` rises above standing z for all four legs simultaneously. IK failures possible as feet reach full lift height plus standing offset, pushing toward workspace boundary.
**Test strategy**: `gait_tick_ik(0.5f, GaitDir::FORWARD, {12.0f, 10.0f, 1.5f, 150, 0}, 1.0f)`. Assert all four `feet[i].z > 0.0f` simultaneously. **Do not test on hardware without a support frame.**
**Notes**: Parameter ramp in `gait.cpp` blends `stand_time_ms` as `uint32_t`; if a command sends `stand_time_ms=0`, the blend will transit through it.
**Cross-references**: see L1-F01, L3-F11.

---

### L1-F04: Frozen Phase With Non-Zero Swing Lift (speed = 0)
**Severity**: degraded
**Mechanism**: `stride_length_mm` is scaled by `speed` (`sl = config.stride_length_mm * speed`), but `stride_height_mm` is NOT scaled: `sh = config.stride_height_mm` regardless of `speed`. When `speed=0`, `sl=0` so all x-offsets are zero, but if the current phase puts a leg in the swing branch, `swing_process` still computes `z = sh * sin²(pct*π)`. `gait.cpp` also stops advancing phase when `speed=0` (phase advance = `2π * freq * speed * dt = 0`), so legs freeze at their current phase-determined z.
**Symptoms**: Robot commanded to `speed=0` but one or two legs hover in mid-air at a fixed height proportional to `sin²` of the frozen phase angle. Servos hold constant non-standing positions. Mock `foot_z` shows asymmetric values: some legs at standing z, others elevated.
**Test strategy**: Let mock walk for ~125 ms (half period, phase ≈ π) then set `speed=0`. Observe that `feet[GAIT_FL].z` and `feet[GAIT_RR].z` remain non-zero for all subsequent ticks.
**Notes**: The `STAND_RETURN` taper in `gait.cpp` runs when transitioning to `STAND`/`STOP`; in pure `speed=0` without a state change, the frozen lift persists.
**Cross-references**: see L3-F07 (phase not reset on state transition).

---

### L1-F05: Yaw Trim Abandoned During Turns (Drift Compounds)
**Severity**: degraded
**Mechanism**: In `gait_tick_ik`, `left_mul` and `right_mul` are initialized from the turn direction before the yaw trim block: `left_mul = (dir == TURN_LEFT) ? 0.3f : 1.0f`. The trim block is then gated: `if (yaw_trim_mul != 0.0f && (dir == FORWARD || dir == BACKWARD))`. For `TURN_LEFT` or `TURN_RIGHT`, the gate is false — trim is never applied and the 0.3/1.0 differential is used as-is. Any stored persistent yaw drift correction is completely ignored during turning maneuvers.
**Symptoms**: During a sustained `TURN` command, yaw drift correction from `s_yaw_trim_mul` (saved in NVS) has no effect. Robot drifts if it has a physical drift tendency.
**Test strategy**: Set `yaw_trim_mul=0.4`, then call `gait_tick_ik(phase, GaitDir::TURN_RIGHT, cfg, 1.0f, 0.4f)`. Verify that `feet[GAIT_FL].x` amplitude equals `feet[GAIT_FR].x * 0.3` (the turn ratio), not modified by trim. Compare with a `FORWARD` direction call where trim is applied.
**Notes**: May be intentional design (turn = explicit override, trim = auto-correction only for straight motion). Should be explicitly documented either way.
**Cross-references**: see L3-F10 (NVS save stutter), L2-F02 (RL polarity can compound drift).

---

### L1-F06: yaw_trim Float Exact-Zero Comparison
**Severity**: cosmetic
**Mechanism**: The trim branch gates on `if (yaw_trim_mul != 0.0f && ...)`. Float equality to zero is unreliable if `yaw_trim_mul` acquires a denormal value through arithmetic upstream. In `gait.cpp`, `gait_set_yaw_trim()` assigns directly (no arithmetic), so in practice the value is representable. However, if a future code path computes trim via subtraction or accumulation, a near-zero denormal would cause the branch to fire when intended to be a no-op.
**Symptoms**: With a denormal trim value, the `FORWARD`/`BACKWARD` paths see an unintended 1.0/1.0 multiplier reset. Effectively: trim silently becomes zero. Robot drifts.
**Test strategy**: Pass `yaw_trim_mul = 5e-45f` (denormal, non-zero) to `gait_tick_ik` with `GaitDir::FORWARD`. Verify whether the trim block fires by checking effect on `feet[GAIT_FL].x` vs `feet[GAIT_FR].x`.
**Notes**: Not triggered by current NVS load path. Relevant if trim computation is ever moved to an incremental update loop.
**Cross-references**: see L1-F05.

---

### L1-F07: Negative or Out-of-Range GaitConfig Values
**Severity**: degraded
**Mechanism**: `GaitConfig::stride_length_mm` and `stride_height_mm` have no validation in `gait_tick_ik`. If `stride_height_mm < 0`, `swing_process` returns `z = height_mm * sin²(t) < 0` — feet are commanded below the standing plane. If `stride_length_mm < 0`, forward command produces backward foot sweep (inverted motion). Neither produces an error return.
**Symptoms**: Negative `stride_height_mm`: feet crouch during swing. Negative `stride_length_mm`: robot walks backward when commanded forward. Mock `foot_z` sinks below standing value during swing phases.
**Test strategy**: `gait_tick_ik(π, GaitDir::FORWARD, {12.0f, -5.0f, 1.5f, 150, 200}, 1.0f)` — assert swing-phase legs have `z < 0`. `gait_tick_ik(π/4, GaitDir::FORWARD, {-12.0f, 10.0f, 1.5f, 150, 200}, 1.0f)` — assert stance-phase foot x is positive (backward sweep during forward command).
**Notes**: Input validation is the responsibility of the JSON command handler. If host sends malformed `cmd_gait_config`, this propagates unchecked all the way to servo commands.
**Cross-references**: see L2-F03 (silent clamp), L3-F12 (leg freeze on IK failure).

---

### L1-F08: Diagonal Pair Asymmetry During TURN at Zero Speed
**Severity**: degraded
**Mechanism**: During `TURN_LEFT`/`TURN_RIGHT` with `speed=0`, `sl = stride_length_mm * 0 = 0`, so all x-offsets are zero. In stance: `stance_process({0,0,0}, {0,0,0}, pct)` returns `{0, 0, 0}`. In swing: `swing_process({0,0,0}, {0,0,0}, pct, sh)` returns `{0, 0, sh*sin²(pct*π)}` — legs still lift. At zero speed a `TURN` command causes all legs to lift in their gait pattern, producing stepping motion with zero net travel.
**Symptoms**: `TURN_LEFT` at `speed=0`: robot oscillates vertically in diagonal pairs without turning or translating. Telemetry: all foot x=0, z oscillates between 0 and `stride_height`.
**Test strategy**: `gait_tick_ik(phase, GaitDir::TURN_LEFT, cfg, 0.0f)` — sweep phase. Assert all `feet[i].x == 0.0f` but some `feet[i].z > 0`.
**Notes**: Commanding a turn at zero speed produces unexpected shaking in place.
**Cross-references**: see L1-F04.

---

### L1-F09: Phase Accumulation — Single-Subtract Wrap Insufficient at Maximum dt
**Severity**: cosmetic
**Mechanism**: `gait.cpp` line 326–327: `s_phase += 2π * freq * speed * dt; if (s_phase > 2π) s_phase -= 2π`. Only one subtraction. The dt guard (`if dt > 0.5s: skip`) limits single-tick increment to `2π * 1.5 * 1 * 0.5 = 4.71 rad < 2π = 6.28 rad`, so a single subtraction is sufficient for current parameters. However, `gait_tick_ik` itself uses `fmodf(phase_rad / 2π, 1.0f)` which handles arbitrarily large values correctly.
**Symptoms**: If the dt guard is ever relaxed or frequency increased significantly, phase would alias by one cycle, causing the wrong diagonal pair to be in stance. Robot stumbles.
**Test strategy**: Call `gait_tick_ik(100000.0f, GaitDir::FORWARD, cfg, 1.0f)` — very large phase. Verify output is consistent with expected phase. Then call `gait_tick_ik(-0.01f, ...)` — negative phase. Verify `phA += 1.0` correction fires.
**Notes**: The `fmodf` + negative correction in `gait_tick_ik` handles both large positive and negative `phase_rad` correctly. The vulnerability is in `gait.cpp`'s accumulation pattern, not in the kernel itself. Currently safe; becomes degraded if the dt guard is relaxed.
**Cross-references**: see L3-F09 (millis wrap).

---

## 2. Layer 2: Kinematics (`ik.h` / `body_transform.h`)

### L2-F01: IK_HIP_Z Unverified — Systematic Foot Position Error
**Severity**: degraded
**Mechanism**: `IK_HIP_Z = -25.0f` mm, explicitly flagged `// unverified; re-probe when convenient` in `ik.h`. Used in `hip_position(leg)` and indirectly in `standing_angles()` calibration. If the physical hip pivot is at -20 mm, `lz` becomes `-60 mm` instead of `-55 mm`, `us_per_rad` computes from wrong standing angles, and every IK command to a non-standing pose produces a systematic position error proportional to deviation from standing.
**Symptoms**: Robot appears to stand correctly (calibrated to observed standing-pose pulses), but all gait offsets produce wrong foot trajectories — stride is longer/shorter than commanded, swing height differs. Balance oscillates. Servos consistently reach soft-clamp limits sooner than expected in one direction.
**Test strategy**: In mock, run `standing_foot_pos(0)` and compare to stock firmware ground truth `[59.25, 46.0, -80.0]`. The test in `test_ik.cpp` (test 2: stock_ground_truth) checks this with a 2 mm gate. Tighten the gate to 0.5 mm to expose the error. Probe: temporarily set `IK_HIP_Z=-20.0f` in a local build and run `test_ik` — expect stock_ground_truth to fail.
**Notes**: This is the highest-confidence unverified assumption in the codebase. Potentially unsafe on terrain where position error compounds with obstacle clearance. Hardware test: measure distance from body center to hip pivot pin with calipers. See also the [Known-Unverified Items](#6-known-unverified-items) section.
**Cross-references**: see L2-F09 (us_per_rad accuracy), L3-F12 (leg freeze on IK failure), L4-F04 (balance compounds pose error).

---

### L2-F02: RL Servo Polarity UNVERIFIED Post-Swap
**Severity**: unsafe
**Mechanism**: `SERVO_POLARITY_OVERRIDE[4] = 0` (RL_hip) and `SERVO_POLARITY_OVERRIDE[5] = 0` (RL_knee), both marked `UNVERIFIED post-swap` in `config.h`. With override=0, auto-derivation fires: `su=2170>1500 → pol=+1` (RL_hip), `su=1611>1500 → pol=+1` (RL_knee). If either physical polarity is actually -1, `angle_to_pulse(idx, angle_rad)` sends the servo in the wrong direction for any deviation from standing.
**Symptoms**: Robot limps or circles; RL leg moves opposite to commanded direction or flips to wrong side. During gait: mock pitch/roll oscillates abnormally due to RL foot moving out-of-phase with the diagonal pair. Servo current on RL may exceed normal. Hardware: RL leg kicks outward/backward on first walk command.
**Test strategy**: Nudge test in mock: call `body_pose_to_pulses({5,0,0,0,0,0}, pulses)` (small forward body shift). Verify `pulses[4]` (RL_hip) changes in the expected direction. Compare with FL_hip (idx 0) response — they should be symmetric. On hardware: with dog in sling, slowly command small gait offset and observe RL leg direction vs FL.
**Notes**: The servo GPIO swap corrected `SERVO_PINS` but left polarity unverified. This is the highest-priority hardware verification item. See also the [Known-Unverified Items](#6-known-unverified-items) section.
**Cross-references**: see L3-F12 (leg freeze), L4-F06 (derivative amplifies asymmetry).

---

### L2-F03: leg_ik Sanity Check is Tautological — Silent Clamp Instead of Rejection
**Severity**: degraded
**Mechanism**: `leg_ik` (ik.h lines 224–229) computes `hp = angle_to_pulse(hi, hip_out)` then checks `if (hp < SERVO_JOINT_MIN_US[hi] || hp > SERVO_JOINT_MAX_US[hi]) return false`. But `angle_to_pulse` already clamps its output to `[SERVO_JOINT_MIN_US[idx], SERVO_JOINT_MAX_US[idx]]` before returning. The returned `hp` is always in range. The sanity-check condition is always false — it never rejects. When IK computes an angle that requires a pulse outside the hardware range, `leg_ik` returns `true` with a silently clamped angle, causing the foot to land at the wrong position.
**Symptoms**: Foot ends up at wrong position during maximum-extent gait offsets. The robot's stride appears truncated on one side. No IK failure is reported to the caller — `foot_to_pulses` returns `true`, so the hold-last-value fallback in `gait.cpp` does NOT fire. Servo reaches its soft-clamp limit silently.
**Test strategy**: Compute a foot position that requires a hip pulse > 2300 μs. Call `leg_ik(0, target, hip_a, knee_a)` — it returns `true`. Then call `angle_to_pulse(0, hip_a)` — returns 2300 (clamped). Call `leg_fk_mm(0, hip_a_clamped_angle, knee_a)` and verify foot x differs from target x.
**Notes**: Fix: check the raw un-clamped `us_f` in `angle_to_pulse` before clamping, and propagate the out-of-range flag.
**Cross-references**: see L1-F07 (out-of-range config drives boundary violation), L3-F12 (leg freeze on IK failure, which this bug prevents from firing).

---

### L2-F04: 4-Bar Knee Linkage Not Modeled — Systematic Shin Angle Error
**Severity**: degraded
**Mechanism**: `IK_L3=14.5f`, `IK_L4=14.0f`, `IK_L5=14.0f` are declared in `ik.h` but the comment explicitly states `knee_virtual_to_servo() is identity until that dimension is measured`. The servo drives the shin through a parallel 4-bar linkage; the IK computes a virtual knee angle and sends it directly to the servo as if `servo_angle = shin_angle`. The actual transmission ratio `ε = f(L3, L4, L5, ground_link_length)` is not computed. Systematic shin angle error of ~5–10% (estimated) propagates to every foot position command.
**Symptoms**: All legs land consistently short or long compared to IK prediction. Stride appears compressed or elongated. Balance oscillates because the body's actual height differs from the commanded height.
**Test strategy**: In mock, measure actual foot z via `pulses_to_foot` at peak swing and compare to expected swing height. On hardware: measure actual foot lift with a ruler and compare to `GAIT_STRIDE_HEIGHT_MM` setting.
**Notes**: Ground-link length is not in any header file; must be measured physically. See also the [Known-Unverified Items](#6-known-unverified-items) section.
**Cross-references**: see L2-F01 (compounds with IK_HIP_Z error), L4-F05 (mock physics fidelity).

---

### L2-F05: cal_table() Data Race on Multi-Core Hosts
**Severity**: cosmetic
**Mechanism**: `ik_detail::cal_table()` uses a static `bool built = false` guard (ik.h lines 105–144). On ESP32 (single-core `setup()` context with `ik_init()` called before FreeRTOS), this is safe. On the host mock build (clang++, multi-threaded), if two threads both call `cal_table()` before `built=true` is set, both enter the fill loop simultaneously. This is a data race on the static `bool built` and `table[]` — undefined behavior in C++. An ASAN/TSAN build would flag this.
**Symptoms**: In mock (multi-threaded): rare spurious corruption of one servo's calibration entry. IK produces a sudden wild pulse value for one servo until the process is restarted.
**Test strategy**: Build mock with `-fsanitize=thread` and run a test that starts the physics tick thread before calling any IK function without prior `ik_init()`. TSAN should report the race on `built`.
**Notes**: Not triggered by current usage pattern — `ik_init()` is always called before tasks start, making the race window effectively zero. Fix: use `std::once_flag / std::call_once` or `std::atomic<bool>`.
**Cross-references**: None.

---

### L2-F06: Lateral Body Translation (pose.dy) Is Silently Ignored
**Severity**: degraded
**Mechanism**: `body_pose_to_pulses` sets `foot.y -= pose.dy` for each leg. The foot then goes through `foot_to_pulses → leg_ik`. In `leg_ik`, only `lx = target.x - hip.x` and `lz = target.z - hip.z` are used; `target.y` is never read. The 2-DOF planar IK has no lateral degree of freedom. A pure lateral body translation `{0, 10, 0, 0, 0, 0}` produces identical servo pulses to the zero pose.
**Symptoms**: Commands with non-zero `dy` appear to succeed (IK returns `true`) but produce no servo movement. No error is reported.
**Test strategy**: `body_pose_to_pulses({0, 10, 0, 0, 0, 0}, pulses)` vs `body_pose_to_pulses({0, 0, 0, 0, 0, 0}, pulses_stand)`. Assert all 8 pulses are identical.
**Notes**: Inherent physical limitation of the 2-DOF leg — no lateral hip servo. The failure mode is the silent no-op rather than a reported error.
**Cross-references**: None.

---

### L2-F07: body_pose_to_pulses Partial Failure Leaves Asymmetric Pose
**Severity**: degraded
**Mechanism**: When one leg fails IK in `body_pose_to_pulses`, that leg's pulses fall back to `STANDING_POSE` while the other three legs receive computed (non-standing) pulses. The function returns `false` but still writes all eight pulses — three at the new pose, one at standing. Callers should hold the previous pose or refuse to move, but the walk branch in `gait.cpp` handles per-leg IK failures independently with hold-last-value semantics. In the `STAND` path: if `body_pose_to_pulses` returns false, the entire command is skipped (safe).
**Symptoms**: With one unreachable leg: that leg holds standing position, other three are at the commanded pose. Robot tilts asymmetrically. The balance layer sees this tilt and tries to correct it, potentially driving more legs out of range. Positive feedback instability if balance correction pushes the body further toward the unreachable region.
**Test strategy**: Call `body_pose_to_pulses({0, 0, 70, 0, 0, 0}, pulses)` — enough body raise to make some legs unreachable. Check that `all_ok = false` AND that `pulses[failed_leg*2]` equals `STANDING_POSE[failed_leg*2]`.
**Notes**: The test in `test_transform.cpp` (test 4) checks the all-legs-saturate case. The partial failure case (one unreachable leg only) is not covered.
**Cross-references**: see L4-F04 (balance compounds with user transform), L3-F12 (walk-branch IK failure).

---

### L2-F08: Elbow-Forward Convention Hardcoded — Alternate Branch Inaccessible
**Severity**: degraded
**Mechanism**: `leg_ik` (ik.h line 215) always sets `knee_out = -knee_mag` (negative = folded forward, elbow-forward convention). The elbow-back solution (`knee_out = +knee_mag`) is never considered. If a body pose or gait offset places the foot target in a region where the elbow-forward solution is at or near a servo limit but the elbow-back solution has margin, `leg_ik` either clips or fails, while a valid elbow-back solution exists.
**Symptoms**: IK failure or silent clamping for foot targets near the front of the workspace (large positive lx), even though an alternate knee configuration could reach.
**Test strategy**: Find a foot target where elbow-forward fails but elbow-back would succeed: `target = standing_foot_pos(0) + {20, 0, 5}` (far forward and slightly raised). Check both `knee_out = +knee_mag` and `knee_out = -knee_mag` manually against reachability and servo range.
**Notes**: For MechDog geometry, the physical leg likely only supports elbow-forward reliably. Unverified whether hardware can achieve elbow-back.
**Cross-references**: see L2-F03 (silent clamp masks this), L3-F12 (leg freeze on IK failure).

---

### L2-F09: standing_foot_pos Accuracy Depends on us_per_rad Derivation
**Severity**: degraded
**Mechanism**: `standing_foot_pos(leg)` calls `pulses_to_foot(leg, STANDING_POSE[leg*2], STANDING_POSE[leg*2+1])`, using `us_per_rad = fabsf(dev) / ang_abs` where `dev = (int)su - 1500`. For RL_knee (idx 5): `dev = 1611-1500 = 111 μs`, `ang = 2.22 rad` → `us_per_rad = 50 μs/rad` — very low sensitivity. A 5 μs error produces a 0.1 rad knee angle error → ~6 mm foot position error.
**Symptoms**: `standing_foot_pos` returns foot positions that don't exactly match the stock firmware ground truth (59.25, ±46, -80). The test uses a generous 2 mm gate that may mask calibration error.
**Test strategy**: Run `test_ik.cpp` test 2 (stock_ground_truth) with the 2 mm gate. Tighten the gate to 0.5 mm to expose accumulated errors. Check RL specifically: `standing_foot_pos(2)` should be within 0.5 mm of `(-71.25, 46, -80)`.
**Notes**: The test currently has a comment "corrected geometry: expect <0.5mm; 2mm is a generous gate." The generous gate may be masking real calibration error.
**Cross-references**: see L2-F01 (IK_HIP_Z error compounds this), L2-F04 (knee linkage error further distorts standing_foot_pos).

---

### L2-F10: lerp_pose Does Not Clamp t — Extrapolation Risk
**Severity**: cosmetic
**Mechanism**: `lerp_pose(a, b, float t)` performs unclamped linear interpolation. With `t < 0` or `t > 1`, it extrapolates beyond either endpoint. All callers in `gait.cpp` clamp `t` via `fminf(elapsed / duration, 1.0f)`, but the clamping only guards the upper bound. If `elapsed < 0` (possible if the clock wraps or `s_transform_start` is set in the future), `t` could go negative. Any future caller that omits the clamp would silently extrapolate to out-of-workspace poses.
**Symptoms**: Body pose extrapolates past the intended target, pushing legs out of IK workspace and triggering L2-F07 (partial failure fallback).
**Test strategy**: Call `lerp_pose(a={0,0,0,0,0,0}, b={0,0,10,0,0,0}, -0.5f)` — assert `dz = -5` (extrapolated below floor). Call with `t=2.0f` — assert `dz = 20`.
**Notes**: Currently safe due to caller-side clamp; becomes degraded if a new caller omits the clamp.
**Cross-references**: see L2-F07, L3-F09 (millis wrap could produce negative elapsed).

---

### L2-F11: Mock Physics IMU Pitch/Roll Reflects Servo Commands, Not Body Orientation
**Severity**: degraded
**Mechanism**: `physics::tick()` (physics.cpp) computes body pitch from FK foot positions: `target_pitch = atan2f(front_z - rear_z, wheelbase_mm)`. During gait, swing legs lift (foot_z rises), causing pitch/roll oscillation at gait frequency (~1–3° amplitude). The mock reports oscillating pitch/roll even when the body is perfectly level. Balance running in mock reacts to these oscillations as if the body is rocking, generating spurious correction commands.
**Symptoms**: In mock, telemetry shows `pitch_deg` and `roll_deg` oscillating at gait frequency (1.5 Hz). Balance corrections are applied even on flat synthetic ground. The 50° tilt fault threshold is unachievable through normal servo commands in mock — only direct `gait_update_imu()` injection in test harnesses can exercise the tilt fault path.
**Test strategy**: `bark mock` on flat ground, enable gait, observe telemetry `pitch_deg` field. Measure oscillation amplitude and frequency — should correlate with `GAIT_FREQUENCY_HZ=1.5`.
**Notes**: This is a fundamental limitation of a kinematics-only mock (no contact model, no dynamics). Hardware IMU measures actual body orientation independently of servo commands. Mock-based balance testing is unreliable as a proxy for hardware behavior. `LP_TAU=0.2s` is appropriate for the physics simulation but introduces non-physical lag.
**Cross-references**: see L4-F05 (mock physics PID fidelity limitation).

---

## 3. Layer 3: Gait State Machine (`gait.cpp`)

### L3-F01: Tilt Safety Bypassed While Paused
**Severity**: unsafe
**Mechanism**: `gait_update()` checks `s_paused` at line 202 before the tilt-safety block at line 210. `cmd_servo` calls `gait_pause()` before writing servos directly, setting `s_paused = true`. While paused, the tilt cutoff block — which calls `enter_stop()` and emits the `tilt_fault` event — is never reached for as long as no subsequent `gait_set_state()` clears the flag.
**Symptoms**: Dog is manually positioned by `cmd_servo` commands; it tips over past 50° pitch/roll. No `tilt_fault` telemetry event is emitted. Servos are not stopped. No `enter_stop` invocation. Robot continues to hold the commanded position while falling.
**Test strategy**: In the test harness: call `gait_pause()`, then `gait_update_imu(60, 0)`, then `gait_update(millis())` — assert `gait_current_state()` is still unchanged (not `STOP`). In mock: send `cmd_engage`, wait for `engage_complete`, send `cmd_servo {index:0, pulse_us:1800}`, inject 60° pitch via mock IMU — observe no `tilt_fault` event.
**Notes**: `s_paused` is cleared by the next `gait_set_state()`. If the dog falls before the operator sends another command, no recovery occurs. The heartbeat-detach watchdog (10 s) is the only remaining safety net. The tilt block should be refactored outside the paused-gate to run unconditionally.
**Cross-references**: see L4-F07 (stale IMU prevents tilt fault), L6-F02 (cmd_reset doesn't send stop).

---

### L3-F02: Tilt Safety Bypassed During Servo Engage Ramp
**Severity**: unsafe
**Mechanism**: `gait_update()` returns early at line 201 (`if (servos_is_ramping()) return;`) before the tilt-safety block. `servos_is_ramping()` returns `true` for the full 2-second `SOFTSTART_DURATION_MS` engage ramp. If the operator engages while the robot is on its side (tilted > 50°), the firmware performs a 2-second stiff-torque ramp to `STANDING_POSE` with no tilt abort path.
**Symptoms**: Robot engaged at high tilt angle. Servos fight gravity for 2 seconds at full standing-pose targets. No `tilt_fault` event. Possible mechanical stress on hip/knee joints during the ramp.
**Test strategy**: In mock: call `cmd_engage {enabled:true}`, before `engage_complete` set physics model to report 60° roll — observe no `tilt_fault` event fires during the 2 s ramp window; `engage_complete` fires normally.
**Notes**: The comment at line 201 says "never fight a ramp," which is correct for normal gait motion, but the tilt abort check should still run. Mitigation: split the `gait_update` return-on-ramping guard from the tilt block, ensuring tilt detection always executes.
**Cross-references**: see L3-F01, L4-F07.

---

### L3-F03: WALK↔TURN Immediate Foot-Position Jump
**Severity**: degraded
**Mechanism**: `dir_sign()` lambda returns `+1` for `WALK_FORWARD`, `TURN_LEFT`, and `TURN_RIGHT`; `-1` only for `WALK_BACKWARD`. Therefore `is_reversal` is `false` for any `TURN→TURN` or `WALK_FORWARD→TURN` transition. The state machine applies the new state immediately (`s_state = new_state`). On the very next `gait_tick_ik()` call, `left_mul` and `right_mul` jump from `1.0` (forward walk) to `0.3` (turn inner side) at whatever phase the gait is currently at.
**Symptoms**: Observable mid-stride snapping of inner-side leg positions. In mock: foot Z positions discontinuously change on the transition tick. On hardware: audible servo jump, visible body lurch on `FORWARD→TURN` or `TURN_LEFT→TURN_RIGHT` transitions at higher speeds.
**Test strategy**: In the test harness, after walking forward at full speed, call `gait_set_state(GaitState::TURN_LEFT, 1.0f)`. Record servo positions the tick before and tick after the transition — expect left-side servos (indices 0, 1, 4, 5) to jump.
**Notes**: Deferred reversal is only intended to protect `FORWARD↔BACKWARD`. A low-cost fix is to add a blend ramp on `left_mul`/`right_mul` inside `gait_tick_ik`, or to extend the deferred-reversal path to include `TURN_LEFT↔TURN_RIGHT`.
**Cross-references**: see L1-F05 (yaw trim interaction during TURN).

---

### L3-F04: New Walk Command Accepted During Tilt Hold, Immediately Stopped Again
**Severity**: degraded
**Mechanism**: After a tilt fault, `s_in_tilt_fault = true` for `BALANCE_TILT_HOLD_MS = 1000 ms`. The `blocked` check at line 242 evaluates `tilt || s_in_tilt_fault`. During the hold period (tilt resolved but `s_in_tilt_fault` still true), a `cmd_move` command calls `gait_set_state(WALK_FORWARD)`, which sets `s_state = WALK_FORWARD`. On the very next `gait_update` tick, the `blocked` check fires again and calls `enter_stop(now_ms)` again — this time with a redundant ramp capture with zeroed values.
**Symptoms**: Operator sends `cmd_move forward` after tilt fault — the command is ACK'd `true` (command_handlers.cpp does not consult tilt state), but on the next gait tick the state reverts to `STOP`. Move command silently fails for up to 1 second after tilt resolves.
**Test strategy**: Trigger a `tilt_fault` (inject 60° pitch). Wait 600 ms (tilt resolved, hold not elapsed). Send `{"type":"cmd_move","direction":"forward","speed":1.0}`. Observe: next gait telemetry still shows `state=STOP` or servos unchanged. After 1100 ms total from fault: `cmd_move` should now work.
**Notes**: The tilt hold blocking path should ideally be surfaced to the operator (e.g., with a telemetry event or a NACK from the motion command).
**Cross-references**: see L3-F05 (balance state interaction during hold).

---

### L3-F05: `s_balance_was_enabled` Silently Overrides `cmd_balance` During Fault Hold
**Severity**: degraded
**Mechanism**: On tilt fault entry (line 217), `s_balance_was_enabled = balance_is_enabled()` captures the pre-fault balance state. On re-arm (line 237), `if (s_balance_was_enabled) balance_enable(true)` restores it. If the operator sends `{"type":"cmd_balance","enabled":false}` during the 1000 ms hold period (to permanently disable balance), when the hold expires, `s_balance_was_enabled` (true, captured before the fault) re-enables balance, silently overriding the explicit disable command.
**Symptoms**: Operator disables balance during hold period. After 1 second, balance re-enables without any operator action or event emission. The status telemetry (`balance: true`) contradicts the operator's intent.
**Test strategy**: Enable balance, inject tilt fault. During hold: send `{"type":"cmd_balance","enabled":false}`. Wait for re-arm (>1 s after fault). Observe status telemetry: `balance=true` (overridden).
**Notes**: Fix is to also update `s_balance_was_enabled` if `balance_enable()` is called during the hold period.
**Cross-references**: see L5-F02 (host flag not updated on firmware-side balance change).

---

### L3-F06: Stale `s_target_transform` Persists Across Engage/Disengage Cycles
**Severity**: degraded
**Mechanism**: `gait_init()` resets `s_current_transform = {}` and `s_target_transform = {}` at lines 79–80. However, `gait_init()` is only called once in `setup()`. On re-engage (second `cmd_engage` after `cmd_disengage`), `gait_init` is NOT called again. If `cmd_transform` was used before disengage, on re-engage `s_current_transform` and `s_target_transform` still hold the old pose.
**Symptoms**: After disengage and re-engage, gait immediately applies the previous body transform. Servos settle at a non-neutral body pose on `STAND`. Observer sees tilted standing pose without any new transform command.
**Test strategy**: Engage, send `{"type":"cmd_transform","roll":10,"ms":100}`, disengage, re-engage, stand — observe body pose is roll=10, not neutral.
**Notes**: Fix is to call `gait_set_body_transform({}, 0)` at the start of engage.
**Cross-references**: see L4-F04 (stale transform compounds balance correction range).

---

### L3-F07: Phase Not Reset on State Transitions
**Severity**: cosmetic
**Mechanism**: `s_phase` is never reset in `gait_set_state()`, `enter_stop()`, or after the deferred-reversal apply. When the robot transitions `STOP→WALK` or finishes a reversal, the walk begins at whatever `s_phase` was when motion last stopped. Depending on phase, some legs may be mid-swing while others are in mid-stance, creating an asymmetric start.
**Symptoms**: Occasional "wrong-footed" starts when resuming walking after a stop. Two specific legs may start in swing simultaneously for a brief period (violating the diagonal-pair invariant until phase advances enough). Not dangerous at low speed; at high speed or with aggressive stride could cause a stumble.
**Test strategy**: Walk for 83 ms (intentionally stopping mid-swing), then stop and resume. On the first tick after speed ramp starts, check if foot z positions are symmetric.
**Notes**: Phase reset on restart would fix this but creates a visible "reset snap" if reset mid-swing. A phase-alignment step (snap to nearest stance transition) on restart would be more correct.
**Cross-references**: see L1-F04 (frozen mid-swing legs at speed=0).

---

### L3-F08: Heartbeat-Detach Disengage Race: Stand Ramp Capture Skipped
**Severity**: cosmetic
**Mechanism**: Heartbeat timeout in `main.cpp` calls `servos_detach_all()` before `gait_set_state(GaitState::STOP)`. `servos_detach_all()` sets `s_engaged = false`. When `gait_set_state(STOP)` then calls `enter_stop()`, the walking-check at line 107 evaluates `if (walking && servos_engaged())` — and `servos_engaged()` is now `false`. Therefore the stand ramp capture is skipped. `balance_reset()` runs with no effect when servos are already detached.
**Symptoms**: No visible servo issue. `balance_reset()` runs with no effect when servos are detached. On re-engage, stand taper begins from `REST→STANDING` (the engage ramp handles this separately anyway).
**Notes**: Documented for completeness as an interaction between the heartbeat path and `enter_stop`. Ordering is harmless but slightly unexpected.
**Cross-references**: see L7-F01 (heartbeat as safety net after UI disconnect).

---

### L3-F09: `millis()` Wrap at 49.7 Days
**Severity**: cosmetic
**Mechanism**: `millis()` wraps to 0 after ~49.7 days. The `dt` computation uses unsigned subtraction, which is correct for wraps. The guard at line 253 `dt > 0.5f` will skip the motion tick if the unsigned difference overflows into a value > 500 ms. The tilt-safety block still runs (correct). `s_last_update` is set to `now_ms` on line 205 even when the dt guard fires, so recovery on the next tick is correct. All other timestamp subtractions use unsigned subtraction — correct through wrap.
**Symptoms**: On the first tick after wrap, exactly one motion frame is skipped. Body transform and stand ramp continue correctly on the next tick. Effectively invisible.
**Test strategy**: Set mock clock to `UINT32_MAX - 5` and advance by 10 ms. Verify `gait_update` skips motion on the wrap tick but tilt safety still fires.
**Notes**: Unsigned wrap arithmetic is defined behavior in C++ and produces the correct elapsed time. One skipped motion tick every 49.7 days.
**Cross-references**: see L2-F10 (lerp_pose extrapolation from negative elapsed could occur at wrap).

---

### L3-F10: `gait_save_yaw_trim()` Calls NVS Write from Main Loop
**Severity**: degraded
**Mechanism**: `gait_save_yaw_trim()` calls `yaw_trim_save(s_yaw_trim_mul)` which calls into NVS (ESP32 `Preferences` API). `Preferences.end()` internally takes a flash mutex. On ESP32, NVS writes can block for 20–200 ms depending on sector erase. This runs inside `handle_cmd_yaw_trim` in `command_handlers.cpp`, which runs inside `loop()` on the main core — the same core as `gait_update()`.
**Symptoms**: On `{"type":"cmd_yaw_trim","op":"save"}`, the main loop stalls for up to 200 ms. During this window: zero `gait_update` ticks fire, one or more IMU telemetry frames are dropped, servo positions freeze at last commanded value. On a walking robot, this causes a ≤200 ms stride stutter.
**Test strategy**: While walking, send `{"type":"cmd_yaw_trim","op":"save"}`. Observe: brief servo motion pause in 3D viewer; gap in IMU telemetry timestamps > 20 ms.
**Notes**: `offsets_save()` has the same latency issue. Neither operation has a guard requiring `STOP` state before writing NVS. Operator should only save while standing/stopped.
**Cross-references**: None.

---

### L3-F11: Config Blend Converges Instantly When `dt` Is Large
**Severity**: cosmetic
**Mechanism**: The config blend at line 275: `float blend = fminf(dt / GAIT_PARAM_RAMP_S, 1.0f)` where `GAIT_PARAM_RAMP_S = 0.4f`. Applied each tick as a lerp multiplier. If `dt` is between 0.4 s and 0.5 s (e.g., after a long servo ramp that just finished), `blend = 1.0` and the new config is applied instantaneously in a single tick — bypassing the intended smooth 400 ms ramp.
**Symptoms**: Mid-walk gait-param change (e.g., `stride_length 12→40 mm`) could instantaneously snap foot targets if gait was paused or ramping for 400–500 ms. Visible foot jump on the first tick after the ramp.
**Notes**: The window of concern is 0.4–0.5 s gaps. Mitigated slightly by the `dt > 0.5` guard which skips the blend block entirely for very large gaps.
**Cross-references**: see L1-F01, L1-F02, L1-F03 (degenerate configs that blend can produce).

---

### L3-F12: Walk-Branch IK Failure Holds Last Servo Position (Leg Freezes)
**Severity**: degraded
**Mechanism**: In the walk branch (lines 339–356), when `foot_to_pulses()` returns `false` (leg unreachable), `servo_write_us()` is NOT called — the servo holds its last commanded position. By contrast, `body_pose_to_pulses()` in the `STAND`/`STOP` branch falls back to `STANDING_POSE[leg*2]` on failure. The two branches have inconsistent failure semantics. IK failure can occur when balance correction + body transform + gait offset compound to move the foot outside the reachable sphere, or when the solved pulse exceeds per-joint clamp limits (which the sanity check doesn't catch — see L2-F03).
**Symptoms**: One or more legs freeze mid-gait at the last valid pulse. The other legs continue their stride cycle. Asymmetric gait — one or two legs stationary while others step. In the 3D mock viewer, affected legs appear stuck. No error is emitted.
**Test strategy**: Apply an aggressive body transform that pushes one or more feet to the IK boundary while walking: `{"type":"cmd_transform","roll":15,"pitch":15,"z":10,"ms":100}`. Observe: 3D viewer legs freeze asymmetrically. Alternatively, set `stride_length_mm = 60` (5× default) via `cmd_gait_params` and walk at `speed=1.0`.
**Notes**: A consistent fallback (either always hold or always use standing pose on IK failure) would improve predictability. The walk-branch failure case should at least emit a debug telemetry event.
**Cross-references**: see L2-F03 (silent clamp prevents IK rejection), L2-F07 (partial pose failure), L1-F07 (out-of-range config), L4-F04 (balance+transform compound).

---

### L3-F13: Deferred Reversal Coast-to-Zero Floating-Point Threshold
**Severity**: cosmetic
**Mechanism**: Pending state is applied at line 265: `if (s_has_pending && s_speed < 0.02f)`. Speed decelerates at `GAIT_SPEED_ACCEL_PER_S = 3.0` units/s. The floor clamp at line 261 (`if (s_speed < 0.0f) s_speed = 0.0f`) ensures `s_speed` reaches exactly 0.0, which satisfies `< 0.02`. The threshold is always crossed within ~340 ms.
**Symptoms**: No failure under normal conditions. The threshold is always crossed.
**Notes**: The 0.02 threshold adds a marginal 6 ms of extra coast at the end before the direction flip. Intentional to ensure stable zero-speed before reversing.
**Cross-references**: None.

---

### L3-F14: Stand Ramp Capture With Stale `s_stand_ramp_from` on Mid-Taper Disengage
**Severity**: cosmetic
**Mechanism**: `s_stand_ramp_from[]` is only populated `if (walking && servos_engaged())`. If the robot disengages mid-taper (ramp still running, `s_stand_ramp_start > 0`, but `servos_engaged` goes false), on re-engage the old `s_stand_ramp_start` value persists. The ramp check `(now_ms - s_stand_ramp_start) < STAND_RETURN_MS` may evaluate as `true` initially (if re-engage happened within 600 ms), causing the taper to use the stale `s_stand_ramp_from[]` values from before disengage.
**Symptoms**: Visible brief position jump on re-engage after mid-taper detach.
**Notes**: Edge case; only occurs if heartbeat or explicit disengage fires during a stand taper.
**Cross-references**: see L3-F08, L3-F06.

---

## 4. Layer 4: Balance PID (`balance.h` / `balance.cpp`)

### L4-F01: Integral Accumulates at `ki=0` and Spikes on Runtime `ki` Enable
**Severity**: degraded
**Mechanism**: The default `balance_init` in `gait_init` sets `ki_pitch = 0.0f` and `ki_roll = 0.0f`. However, `pid_update` always runs `integral += error * dt` regardless of whether `ki` is zero. Anti-windup: when `ki = 0`, `max_integral = max_out = 8.0`. So `s_pitch_integral` accumulates up to `±8.0` over time. At `ki = 0`, `ki * integral = 0` so this has no output effect. But if the operator later calls `cmd_balance_config` to raise `ki` to, say, `0.1`, the first tick produces an output spike of `ki * integral = 0.1 * 8.0 = 0.8°` correction — a sudden, unintended body motion.
**Symptoms**: Enabling the integral term via `cmd_balance_config` causes a one-tick step in body position. Visible as a brief jerk of the standing body, or (during walk) a one-frame foot offset spike that may cause IK failure on an affected leg.
**Test strategy**:
```cpp
BalanceConfig cfg = {0.3f, 0.0f, 0.0f, 0.3f, 0.0f, 0.0f, 8.0f, 0.0f};
balance_init(cfg); balance_enable(true);
for (int i=0; i<100; i++) balance_update(5.0f, 0.0f, 0.02f);
balance_set_gains(0.3f, 0.1f, 0.0f, 0.3f, 0.1f, 0.0f);
BodyPose out = balance_update(5.0f, 0.0f, 0.02f);
// out.pitch should be distinctly higher than the pure-kp value (0.3*5=1.5°)
```
**Notes**: Fix: reset integrators in `balance_set_gains` when `ki` changes from 0 to non-zero. The existing `test_balance` Test 6 deliberately tests that `balance_set_gains` does NOT reset integral — which is correct for production gain-tuning, but the `ki=0→ki≠0` transition is an uncovered edge case.
**Cross-references**: see L4-F04 (integral spike compounds with body transform).

---

### L4-F02: Derivative Spike on PID Re-enable After Extended Disable
**Severity**: degraded
**Mechanism**: If `balance_enable(false)` + `balance_enable(true)` is called without a reset (not triggered by current callers, but possible from future callers), `s_pitch_prev_err` holds a stale value from before disable. If the IMU reads a different angle on re-enable (e.g., robot was moved while disabled), the derivative is computed as `(new_error - stale_prev_err) / dt`. For a 5° angle change while disabled and `dt = 0.02 s`, the derivative spike is `5.0 / 0.02 = 250°/s × kd = 0.05 → 12.5°` — clamped to `max_correction_deg = 8°`.
**Symptoms**: On re-enabling balance after a position change while disabled, the first tick produces maximum correction output (`±8°`). Visible as a sharp body rotation toward standing position, potentially tripping an IK boundary failure (L3-F12).
**Notes**: All current callers appear to call `balance_reset()` alongside `balance_enable(false)`. Risk is future callers not following the convention.
**Cross-references**: see L3-F12 (leg freeze on IK failure from spike), L4-F06 (derivative amplification).

---

### L4-F03: Deadband Flip-Flop Prevents Integral Buildup
**Severity**: degraded
**Mechanism**: When `|pitch_err| < deadband_deg (0.5°)`, `balance_update` zeroes `s_pitch_integral` AND resets `s_pitch_first = true`. If the system oscillates with IMU pitch alternating between 0.4° (inside deadband) and 0.6° (outside deadband), the integral is zeroed and `prev_err` is seeded on every transition through the deadband. A system oscillating at ±0.6° centered on a 0.4° mean never accumulates integral action and loses derivative continuity repeatedly.
**Symptoms**: On a slightly off-balance standing robot, balance correction oscillates at kp-only amplitude (never settling), even when `ki > 0`. Steady-state body tilt persists indefinitely at ~`deadband_deg / kp` from center. IMU telemetry shows pitch/roll oscillating ±0.5–1° without converging.
**Test strategy**:
```cpp
BalanceConfig cfg = {0.3f, 0.1f, 0.05f, 0.3f, 0.1f, 0.05f, 8.0f, 0.5f};
balance_init(cfg); balance_enable(true); balance_reset();
float angles[] = {0.6f, 0.4f, 0.6f, 0.4f, 0.6f, 0.4f};
for (float a : angles) {
    BodyPose out = balance_update(a, 0.0f, 0.02f);
    // observe output never grows from ki accumulation
}
```
**Notes**: Fix: only reset `s_pitch_first = true` in `balance_reset()`, not on each deadband entry. This allows the derivative term to compute correctly on deadband exit.
**Cross-references**: see L4-F06 (derivative noise at deadband boundary).

---

### L4-F04: Balance Output Compounds With User Body Transform Past IK Reach
**Severity**: degraded
**Mechanism**: In `gait_update` (lines 292–297), `combined.pitch = s_current_transform.pitch + bal.pitch`. The user can command up to any arbitrary pitch via `cmd_transform`. Balance correction is clamped to `±8.0°`. For a user transform of `pitch = 20°` plus a balance correction of `8°`, `combined.pitch = 28°`. At this extreme, at least one leg's IK solve fails, triggering L3-F12 (leg freezes at last position).
**Symptoms**: With large user body transforms active and balance enabled, one or more legs silently hold last position or snap to standing pose, causing visible body asymmetry. In mock: leg(s) in 3D view stop responding to gait. On hardware: asymmetric torque distribution.
**Test strategy**: Send `{"type":"cmd_transform","pitch":20,"ms":100}`, wait 200 ms. Enable balance. Inject IMU pitch of 10° into mock. Observe: combined pitch = 20+8 = 28° → IK fail (no telemetry currently emitted); 3D viewer shows frozen leg.
**Notes**: A pre-IK reachability check (or clamping combined pose before IK) would prevent this. `cmd_transform` is unconstrained — there are no server-side limits on pitch/roll values.
**Cross-references**: see L3-F12 (leg freeze), L2-F07 (partial failure leaves asymmetric pose), L3-F06 (stale transform from prior session).

---

### L4-F05: Mock Physics Pitch/Roll Derived From Foot Z Only — Limited Balance Feedback Fidelity
**Severity**: cosmetic
**Mechanism**: `physics::tick()` computes pitch and roll from foot Z positions through a first-order low-pass filter with `LP_TAU = 0.2 s`. During gait, swing legs lift (foot_z rises), causing pitch/roll oscillation at gait frequency. Balance corrections cause foot height changes, which feed back into pitch/roll with a 0.2 s LP delay. The effective PID loop has significant additional lag not present in real IMU hardware.
**Symptoms**: Balance appears extremely stable in mock even with high kd values (no noise to amplify). Balance may oscillate on hardware when mock testing showed convergence. 50° tilt faults are unachievable in normal mock operation — only achievable by direct `gait_update_imu()` injection.
**Test strategy**: To test tilt fault in mock, use the test harness: call `gait_update_imu(60, 0)` directly. This is the only way to exercise the tilt fault path in mock.
**Notes**: LP_TAU=0.2 s introduces non-physical lag. A higher-fidelity mock would add body inertia to the model. Mock is useful for state-machine behavior but not for PID stability tuning; hardware testing required for tuning kp/ki/kd.
**Cross-references**: see L2-F11 (mock physics limitation), L4-F06 (hardware noise not represented in mock).

---

### L4-F06: Derivative Term Amplifies IMU Noise on Hardware
**Severity**: degraded
**Mechanism**: In `pid_update`: `float raw_derivative = (error - prev_err) / dt`. At 50 Hz (`dt = 0.02 s`) with typical QMI8658 pitch noise of ±0.2°, the raw derivative is `±0.2 / 0.02 = ±10°/s`. Multiplied by `kd = 0.05`: `±0.5°` of noise contribution per tick. At `kd = 0.3` (6× default), noise contribution becomes `±3°`, causing visible servo jitter.
**Symptoms**: On hardware with balance enabled, servo positions exhibit ±5–30 μs jitter per tick at default kd. At higher kd, audible high-frequency buzzing from hip servos during `STAND`.
**Test strategy**: Hardware only. Enable balance with `kd = 0.3` via `cmd_balance_config`, monitor servo positions. Alternatively, log `balance_update` output at 50 Hz and compute RMS noise.
**Notes**: Cosmetic at default `kd = 0.05`; degraded at `kd > 0.1`. A low-pass filter on the derivative term (EMA) would significantly reduce noise sensitivity. No derivative filtering is currently implemented.
**Cross-references**: see L4-F03 (deadband interaction with noise), L4-F02 (derivative spike on re-enable).

---

### L4-F07: `gait_update_imu()` Only Called When Host Is Connected
**Severity**: unsafe
**Mechanism**: In `main.cpp`, `gait_update_imu()` is inside `if (connected && now - last_imu >= 1000 / TELEM_IMU_HZ)`. If the host disconnects (for any reason other than heartbeat timeout), `connected = false` and `gait_update_imu()` stops being called. `s_pitch` and `s_roll` in `gait.cpp` freeze at their last values. The tilt safety block continues to use stale pitch/roll. If the last IMU update before disconnect showed pitch/roll within safe limits (< 50°), the tilt block never fires even if the robot is subsequently tipped over.
**Symptoms**: After host disconnect (without heartbeat timeout), balance controller runs on stale IMU data. If the robot tips over, no `tilt_fault` event fires. Servos continue driving balance correction for the last-known angle indefinitely, until the heartbeat watchdog fires (10 s later).
**Test strategy**: `bark mock`. Connect host, engage, stand, enable balance. Disconnect host TCP without triggering heartbeat (stop bark server host-side). Tip mock robot (inject via physics) — observe no `tilt_fault`. Wait for heartbeat timeout (10 s) — observe `heartbeat_detach`.
**Notes**: The `gait_update_imu()` call should be decoupled from the `connected` gate. The IMU snapshot is already available unconditionally via `sensor_snapshot_get(snap)`. The fix is straightforward: move `gait_update_imu(snap.pitch, snap.roll)` outside the `if (connected && ...)` block.
**Cross-references**: see L3-F01 (paused gate also bypasses tilt), L3-F02 (ramp gate also bypasses tilt), L5-F04 (host-side frozen IMU cache).

---

## 5. Layer 5: Host Balance Monitor (`host/behaviors/balance.py`)

### L5-F01: State-Before-Send Desync on Transport Error
**Severity**: degraded
**Mechanism**: In `set_enabled()` (balance.py lines 31–33), `self._balance_enabled = enabled` is assigned before `await self._transport.send_json(...)`. If `send_json` raises `ConnectionError` (transport closed), the exception propagates but `_balance_enabled` is already flipped. The server then broadcasts `{"type":"balance_state","enabled":True}` based on the now-dirty flag.
**Symptoms**: UI shows "Balance: ON"; `_balance.enabled` returns `True`; firmware never received `cmd_balance` and remains in its previous state. Any subsequent re-enable attempt from UI is silently swallowed (flag already says enabled, UI shows ON, no re-send occurs).
**Test strategy**: Connect via mock (`bark mock`), enable balance in UI, then kill the mock TCP server. Observe that `DogIO._reader_loop` detects EOF and sets `_open=False`. Call `set_enabled` again from browser — the server-side `_balance_enabled` flag will be True but firmware was never commanded.
**Notes**: Same pattern exists for `cmd_engage` in `ButtonEngageBehavior.on_button_event` where `_set_engaged(False)` is called before the send completes, but that's an intentional optimistic update pattern. Here it's a bug because the flag is the source of truth for the broadcast.
**Cross-references**: see L6-F03 (transport swap resets balance state), L6-F07 (reconnect doesn't re-sync state).

---

### L5-F02: Firmware tilt_fault Doesn't Reset Host Balance Flag
**Severity**: degraded
**Mechanism**: The firmware can unilaterally disable balance when tilt is detected (>50°). When this happens, the firmware emits a `telem_event` or `telem_status` carrying the new balance state. However, `BalanceLayer._balance_enabled` is only set via `set_enabled()` — there is no code path in `server._on_firmware_telem()` or `BalanceLayer` that updates `_balance_enabled` from incoming `telem_status` or event messages. The `_on_firmware_telem` handler updates `self._engaged` from `telem_status` but ignores the balance field.
**Symptoms**: Firmware disables balance due to `tilt_fault`. UI continues showing "Balance: ON" (because `_balance.enabled` is still `True`). The `_telemetry_loop` broadcasts `telem_status` including `"balance": self._balance.enabled` every ~5 s, continuously asserting the wrong state to all clients.
**Test strategy**: With mock firmware, inject a `telem_status {"balance":false}` directly on port 9001. Observe that `_balance.enabled` remains `True` in the server, and the UI does not update the balance indicator.
**Notes**: Interacts with L5-F03 — if the host's `check()` subsequently detects recovery (<20°), it won't re-enable balance because there's no auto-re-enable logic; but the flag mismatch means the UI status is perpetually wrong until operator manually toggles.
**Cross-references**: see L3-F05 (firmware-side balance re-enable override), L5-F03 (recovery doesn't re-enable), L5-F01 (state-before-send pattern).

---

### L5-F03: Balance Re-enable Race After Tilt-Fault Recovery
**Severity**: degraded
**Mechanism**: Firmware fires `tilt_fault` at >50° and disables balance. Host `check()` fires at 20 Hz. The fall threshold is 35°; the recovery threshold is 20°. Between 35° and 50°, the host fires `event_fall` and sets `_fallen=True`. If the robot physically recovers (IMU drops below 20°), `check()` sets `_fallen=False` and emits `event_recovered` — but does NOT call `set_enabled(True)`. Nothing in `check()` or the telemetry loop re-enables balance after recovery.
**Symptoms**: Dog physically recovers from tilt. Host emits `event_recovered`; UI clears fall alert and shows dog upright. Balance remains disabled. Dog can then topple again without any balance correction.
**Test strategy**: Mock: simulate IMU climbing past 35°, then dropping below 20° via injected `telem_imu` messages at the mock TCP port. Monitor server logs for `event_fall`/`event_recovered` and verify `_balance.enabled` state does not change.
**Cross-references**: see L5-F02 (host flag not updated by firmware), L3-F04 (tilt hold blocks re-arm for 1 s).

---

### L5-F04: Frozen IMU Cache Prevents Fall Detection
**Severity**: degraded
**Mechanism**: `check()` reads `self._transport.get_imu()` which returns `dict(self._imu)` from `dog.py` — the last values pushed by `_handle_telem`. The `_imu` dict is only updated when a `telem_imu` message arrives. If the serial/TCP reader goes silent (firmware crash, cable disconnect detected late), `_imu` freezes at its last-known value. There is no timestamp or staleness guard in `check()`.
**Symptoms**: Dog falls physically. `telem_imu` stops arriving. Frozen cached values show, e.g., pitch=5°, roll=2° — so `check()` never fires `event_fall`. No fall alert. Odometry broadcast continues showing robot moving normally.
**Test strategy**: Run `bark mock`. Enable movement. Kill the mock firmware process mid-session (so the TCP connection drops). Observe: `DogIO._reader_loop` sets `_open=False` on EOF; `get_imu()` returns stale data; no fall event fires for the stale pitch values.
**Notes**: The reconnect loop polls `transport.is_open()` with 2 s sleep before each check. Fall detection gap can be up to ~3 s after transport EOF before reconnect attempt begins.
**Cross-references**: see L4-F07 (firmware-side stale IMU), L5-F05 (zero-initialized cache at startup).

---

### L5-F05: False Recovery from Zero-Initialized IMU Cache
**Severity**: cosmetic
**Mechanism**: `Dog.__init__` initializes `self._imu = {"pitch": 0.0, "roll": 0.0, "yaw": 0.0}`. If the robot is lying on its side at startup and the first `telem_imu` frame hasn't yet arrived, `check()` reads pitch=0, roll=0 and reports the dog as upright. The fall detection logic (`_fallen=False`, checking `> FALL_PITCH_DEG`) does not fire.
**Symptoms**: Robot boots while tipped over. Server starts; telemetry loop fires before first `telem_imu` arrives. `event_fall` is never emitted for the initial tipped state. UI shows dog upright.
**Notes**: Cosmetic for normal operation; degraded if someone tries to engage servos while dog is on its side during boot.
**Cross-references**: see L5-F04 (frozen IMU cache is conceptually related).

---

### L5-F06: IMU Oscillation Causing Balance Enable/Disable Flapping (Latent)
**Severity**: cosmetic
**Mechanism**: `check()` has 15° of hysteresis (fall at 35°, recover at 20°) but there is no rate limiting or debounce on `set_enabled()`. As currently written, fall detection only emits events; `set_enabled` is only called by the browser `cmd_balance` handler. The flapping risk is latent but becomes real if balance auto-management is added (e.g., auto-re-enable on recovery). Rapid toggling could send repeated `cmd_balance` to firmware faster than firmware can process, filling the ack_queue (maxsize=64).
**Symptoms**: If firmware receives many `cmd_balance` toggles in rapid succession: ack queue fills (dog.py: "dropping oldest"), some acks are lost, server-side state and firmware-side state diverge.
**Test strategy**: Script 100 rapid `cmd_balance` sends via WebSocket to the server. Monitor ack queue for "dropping oldest" log lines.
**Notes**: Cosmetic in current code; becomes degraded if auto-re-enable is added without rate limiting.
**Cross-references**: see L5-F01 (state desync), L5-F03 (re-enable path).

---

## 6. Layer 6: Host Transport (`server.py` / `host/dog/` / `host/lock.py`)

### L6-F01: Blocking `drain()` Can Stall the Telemetry Loop
**Severity**: unsafe
**Mechanism**: `DogIO.send()` (io.py) calls `self._writer.write(data)` then `await self._writer.drain()` with no timeout. If the TCP or serial write buffer fills (e.g., firmware is slow to consume bytes, or the serial cable has high latency), `drain()` blocks indefinitely inside the asyncio event loop. All other coroutines sharing that event loop — including `_telemetry_loop`, balance `check()`, and WebSocket broadcasts — are starved until `drain()` returns.
**Symptoms**: Server log goes silent. IMU updates stop reaching the browser. If the blockage persists >30 s, the ControlLock times out silently. Dog continues walking (firmware heartbeat keeps running); the host cannot send stop commands until the drain unblocks.
**Test strategy**: Use `bark mock` (TCP). Set up a mock that accepts connections but stops reading from the socket (simulates a blocked TCP window). Send a burst of large commands from the UI. Monitor server logs for freezes; measure time between telemetry broadcasts with timestamps.
**Notes**: Fix: wrap `drain()` with `asyncio.wait_for(..., timeout=2.0)` and handle `TimeoutError` as a transport error.
**Cross-references**: see L5-F04 (frozen IMU cache from stalled telemetry), L7-F01 (UI disconnect if no telemetry arrives).

---

### L6-F02: `cmd_reset` Doesn't Send Stop to Firmware
**Severity**: unsafe
**Mechanism**: In `_handle_ws_message`, `cmd_reset` sets `self._motion = "stop"` and resets server mode, but it does NOT call `await self._transport.send_json({"type":"cmd_move","direction":"stop"})`. It only broadcasts `{"type":"reset"}` to all WS clients. The firmware gait loop is never commanded to stop.
**Symptoms**: Operator presses "Reset" while dog is walking. Server-side `_motion` becomes "stop", so subsequent odometry broadcasts show `"motion":"stop"` and the 3D viz stops its walk animation. However, the firmware continues executing the last `cmd_move` indefinitely (or until heartbeat timeout). Physical robot keeps walking.
**Test strategy**: `bark mock`. Send `cmd_move forward` from UI, then click "Reset". Observe: mock firmware logs should show no `cmd_move stop` arriving after the reset. Mock's gait state continues as "forward".
**Cross-references**: see L7-F01 (UI disconnect also fails to stop firmware), L3-F01 (tilt bypass while paused).

---

### L6-F03: Transport Swap Resets Balance/Engaged State Without Firmware Re-Sync
**Severity**: degraded
**Mechanism**: `_replace_transport()` creates a fresh `BalanceLayer(new_transport)`, resetting `_balance_enabled=False` and `_fallen=False`. It does NOT re-send the current balance state to firmware, nor does it read firmware state before broadcasting the new status. `_broadcast_status()` advertises `"balance":False` even if the firmware was running with balance on before the transport swap.
**Symptoms**: USB is unplugged mid-session. mDNS auto-connects to WiFi. New transport opens. UI receives `telem_status` with `balance:false`. If the operator doesn't notice and re-enable it, the firmware may have balance enabled from before the swap but the host never sends the state.
**Cross-references**: see L5-F01 (state desync), L6-F07 (reconnect doesn't re-sync engaged state).

---

### L6-F04: ControlLock Auto-Acquires to First Sender, Multi-Tab Races
**Severity**: degraded
**Mechanism**: Any browser tab that sends a control command when `_lock.holder is None` immediately acquires the lock. With multiple tabs open, the first tab to send any control command silently grabs the lock — no challenge is sent to other tabs. The lock_challenge mechanism only triggers on explicit `cmd_lock` request, not on auto-acquire.
**Symptoms**: Two operator tabs open. Both show "Take Control". Whichever WS message arrives first wins the lock. The other operator sees `lock_denied` on the next command, not immediately.
**Test strategy**: Open two browser tabs, both connected to `bark mock`. In both, hold forward simultaneously. Observe server logs for which lock acquisition fires, and whether both tabs see consistent lock state simultaneously.
**Notes**: Compounded by the ControlLock's 30 s timeout: if the holder's browser crashes without disconnecting cleanly (no `close()` event), the stale `_holder` WS object lingers for 30 s before timeout clears it.
**Cross-references**: see L6-F05 (zombie WS holds lock), L7-F04 (balance toggle bypasses lock).

---

### L6-F05: WS Client Disconnect Not Detected Until Next Broadcast
**Severity**: degraded
**Mechanism**: `_broadcast()` catches `ConnectionError`/`ConnectionResetError` and removes dead clients from `_ws_clients`. But `_ws_handler`'s `async for raw_msg in ws` loop only exits when the WS stream closes. If a browser tab closes abnormally (process kill, network drop without TCP RST), aiohttp may not detect the close immediately — the dead socket lingers in `_ws_clients` until the next broadcast attempt raises.
**Symptoms**: Disconnected client is still nominally the lock holder. `_lock.release_if_holder(ws)` is called in `_ws_handler`'s `finally` block, but that only runs when the `async for` loop exits. If the TCP session lingers, the lock is held by a zombie WS for up to 30 s (lock timeout).
**Test strategy**: Hold the lock in one tab. Kill the OS process for that browser tab (not graceful close). Observe how long before `_ws_clients` removes the dead entry and lock is released.
**Cross-references**: see L6-F04 (lock contention), L7-F01 (no stop on disconnect).

---

### L6-F06: Web Hash Misses Subdirectory Changes — Stale Client Not Detected
**Severity**: degraded
**Mechanism**: `_compute_web_hash()` iterates `os.listdir(web_dir)` and only hashes top-level files in `web/`. It does not recurse into `web/modules/` or `web/dog3d/`. A change to `web/modules/controls.js`, `web/dog3d/gait.js`, or any subdirectory file is invisible to the hash. The version hash sent as `{"type":"version","hash":...}` will remain unchanged.
**Symptoms**: Developer updates `web/modules/controls.js`. Runs server restart. Connected browsers do not reload (stale client detection relies on hash mismatch). Old JS module code keeps running.
**Test strategy**: Modify `web/modules/controls.js` with a trivial change. Restart server. Observe that `msg.hash` from the new server matches the hash from the old server. Client does not reload.
**Cross-references**: see L7-F05 (stale 3D visualization after disconnect).

---

### L6-F07: Reconnect Loop Doesn't Re-Send Engaged/Balance State After Reconnect
**Severity**: degraded
**Mechanism**: `_reconnect_loop()` calls `await self._transport.open()` on reconnect and broadcasts status. However, it does not re-send `cmd_engage` or `cmd_balance` to restore the firmware's last-known state. After a transport reconnect, firmware boots fresh (or resumes with its own state); the server's `_engaged = True` / `_balance.enabled = True` may not match firmware reality.
**Symptoms**: Transport drops and reconnects. Server `_engaged` is True (never cleared). Dog was disengaged by firmware reset. Server broadcasts `engaged:True` to UI; UI shows "ENGAGED". But firmware is actually disengaged. Operator tries to move; commands go through but servos are still off.
**Cross-references**: see L6-F03 (transport swap), L5-F01 (state-before-send desync).

---

## 7. Layer 7: Web UI (`web/`)

### L7-F01: Walking Robot Not Stopped on WebSocket Disconnect
**Severity**: unsafe
**Mechanism**: `modules/ws.js` lines 37–39: `send()` only transmits if `ws.readyState === WebSocket.OPEN`. On disconnect, `ws.onclose` fires and schedules reconnect. No stop command is ever sent to the server. The browser has no mechanism to guarantee a stop is sent before the socket closes. On tab close, keyup does not fire.
**Symptoms**: Operator holds FORWARD key. Browser crashes or network drops. Firmware receives no stop. Robot walks indefinitely until firmware's own heartbeat watchdog fires (`HEARTBEAT_TIMEOUT_MS=10000`, 10 s).
**Test strategy**: `bark mock`. Hold FORWARD key. Use browser devtools to simulate offline. Observe mock firmware logs: no `cmd_move stop` arrives. Observe that gait continues in mock for up to 10 s.
**Notes**: Firmware's 10 s heartbeat is the only safeguard. The host server does not have its own client-activity watchdog that sends stop to firmware when all WS clients disconnect.
**Cross-references**: see L7-F02 (alt-tab drops keyup), L6-F02 (cmd_reset also doesn't stop firmware), L3-F08 (heartbeat detach).

---

### L7-F02: Window Blur / Alt-Tab Drops Key Release Events
**Severity**: unsafe
**Mechanism**: `controls.js:setupKeyboard()` listens on `document`. When the user switches applications (Cmd-Tab on macOS, Alt-Tab on Windows) while holding a direction key, the `keyup` event fires at the OS level but is not delivered to the browser tab because focus has left the window. `pressed[e.key]` remains `true`. On return to the tab, the key is already released physically, but `pressed` still tracks it as held — no `keyup` fires to trigger the stop command.
**Symptoms**: Operator holds "W" (forward). Switches to another app. Robot continues walking. Returns to tab. No stop is sent until operator presses and releases "W" explicitly.
**Test strategy**: `bark mock`. Hold "W" in browser. Switch to another app window. Wait 5 seconds. Return to browser. Observe mock: `cmd_move stop` was never sent during the alt-tab period.
**Notes**: Standard mitigation is listening for `window.blur` and sending stop + clearing `pressed`. Not implemented.
**Cross-references**: see L7-F01 (no stop on disconnect, same root), L6-F02 (cmd_reset doesn't fix this).

---

### L7-F03: Fall Alert Element Overwritten by Lock/NACK Messages
**Severity**: degraded
**Mechanism**: `app.module.js` reuses `#fall-alert` as a general notification element. `showFallAlert` sets its text to "FALL DETECTED". The lock_denied handler sets `el.textContent` to "Control held by X" with a `setTimeout` to hide after 3 s. The NACK handler sets `el.textContent` to `ref_type + " rejected"`. If a `lock_denied` or NACK arrives while the dog is fallen, the fall alert text is overwritten and auto-hidden after 3 s.
**Symptoms**: Dog is fallen. UI shows "FALL DETECTED" alert. Operator in another tab requests lock → `lock_denied` arrives in the current tab → fall alert changes to "Control held by X" and disappears after 3 s. Operator loses the fall indicator.
**Test strategy**: `bark mock`. Trigger a fall (inject `telem_imu` with pitch=40). Confirm fall alert appears. Have a second tab request the lock. Observe fall alert text change and auto-disappear.
**Cross-references**: see L5-F02 (host fall state not updated), L6-F04 (lock races).

---

### L7-F04: Balance Toggle Not Gated on Engaged State
**Severity**: degraded
**Mechanism**: `controls.js:setupActions()` sends `cmd_balance` on balance-toggle button click, gated only by `canControl()` (lock check). It does not check `_engaged`. Additionally, keyboard shortcut "b" also sends `cmd_balance` unconditionally — without even checking `canControl()`. The server-side handler is gated by `can_control()` but not by engaged state.
**Symptoms**: Dog is disengaged (servos off). Operator presses "b". `cmd_balance enabled:true` is sent to firmware. Firmware processes it (balance is now on). When servos engage later, balance kicks in immediately — potentially unexpected. Also, "b" key bypasses the lock check entirely, allowing non-lock-holders to toggle balance.
**Test strategy**: `bark mock`. Ensure dog is disengaged. Press "b". Observe `cmd_balance` arrives at mock firmware. Then engage: balance fires immediately.
**Cross-references**: see L6-F04 (lock bypass), L3-F05 (balance state override interaction).

---

### L7-F05: Stale 3D Visualization Continues Animating After Disconnect
**Severity**: cosmetic
**Mechanism**: `dog3d/index.js` `animate()` function runs `requestAnimationFrame` unconditionally. `animateGait(dt)` reads `state.currentMotion` and synthesizes leg animation from it, independent of real firmware telemetry. When the WS disconnects, no reset of `currentMotion` occurs in the `ws.js` `onclose` handler. The 3D model continues animating the last motion state.
**Symptoms**: WS drops while robot is walking. Browser shows "Offline" indicator. 3D dog model continues its animated walk cycle. Operator may believe the robot is still moving and delay corrective action, or believe the UI is updating and not notice the disconnect.
**Test strategy**: `bark mock`. Start walking. Kill server process. Observe that browser shows "Offline" but 3D dog animation continues walking.
**Cross-references**: see L7-F01 (stale state after disconnect).

---

### L7-F06: Rapid Direction Toggles Send Unbounded Command Stream
**Severity**: degraded
**Mechanism**: `controls.js:setupDpad()` sends `cmd_move direction` on every `mousedown` event and `cmd_move stop` on every `mouseup`. `setupKeyboard()` uses a `pressed` set to debounce keydown, but switching between two keys rapidly (e.g., alternating W/S presses) sends commands at keyboard repeat rate. There is no client-side rate limiter. Fast toggling (say, 10 msgs/sec) creates a command queue in the transport that the firmware's serial reader must drain. The ack queue (maxsize=64) fills and starts dropping acks.
**Symptoms**: Rapid FORWARD→BACKWARD keypresses generate a burst of `cmd_move` messages. Firmware's deferred direction change mechanism is overwhelmed by the velocity of UI messages. Gait state may lag behind user intent.
**Test strategy**: `bark mock`. Script a rapid alternation between W and S keys. Observe: number of `cmd_move` messages per second at mock, ack queue warnings in server log.
**Cross-references**: see L6-F01 (transport drain blocks under message burst), L5-F06 (ack queue saturation pattern).

---

### L7-F07: Battery Percent Calculation Uses Hardcoded Range
**Severity**: cosmetic
**Mechanism**: `app.module.js`: `batteryPercent(mv)` computes `((mv - 6000) / 2400) * 100`. If a `telem_battery` message arrives with a low voltage (e.g., 5000 mV from a partially discharged pack), `batteryPercent(5000)` = -41.7% → displayed as 0%. The battery graph records this as a critically low/dead battery, potentially alarming the operator unnecessarily. The `BATTERY_ABSENT_MV = 4500` constant in `config.h` defines the threshold below which a battery is considered absent, but this is not reflected in the UI calculation.
**Symptoms**: Battery absent state (USB-only, ~4500 mV via regulator leakage) or a very low pack produces 0% display and red line on battery graph, causing operator alarm when the pack may simply be absent.
**Test strategy**: Inject `telem_battery {"voltage_mv":5000,"present":true}` from mock. Observe `battery-val` shows "0%" and battery graph records red line at 0.
**Cross-references**: None.

---

## 3. Cross-Layer Cascade Chains

### Chain 1: IK Reachability Compounding — Body Transform + Balance + Gait Offset → Leg Freeze → False Tilt
**Trigger**: Operator sends a large `cmd_transform` (e.g., pitch=20°) while balance is enabled and the robot is walking.
**Propagation**: L2-F07 (body_pose_to_pulses partial failure when one leg exits workspace) → L4-F04 (balance output of up to 8° adds to the 20° transform, combined pitch = 28°) → L3-F12 (walk-branch IK failure holds that leg's last servo position) → L2-F11 (mock physics derives pitch from foot Z, now asymmetric due to frozen leg) → false pitch oscillation reported to firmware → L4-F03 (deadband flip-flop prevents integral from correcting the drift) → servo jitter at tilt boundary.
**How to distinguish from other causes**: Observe the 3D viewer for a leg that stops tracking the gait cycle. Check `cmd_transform` history — a large pitch or roll value is the trigger. Reduce transform to zero; if the leg resumes normal motion, the cause was IK compounding, not polarity (see Chain 2).

---

### Chain 2: RL Polarity Wrong → Asymmetric Gait → Balance Oscillation → Tilt Fault
**Trigger**: RL servo polarity unverified post-swap (`SERVO_POLARITY_OVERRIDE[4,5]` in `config.h`).
**Propagation**: L2-F02 (RL_hip or RL_knee moves in the wrong direction for any commanded deviation from standing) → L3-F12 (RL leg appears to freeze or move erratically during walk) → L2-F11 (mock physics reports asymmetric pitch/roll from RL foot moving out-of-phase) → L4-F06 (derivative term amplifies asymmetric oscillation) → L4-F04 (combined correction approaches workspace limit) → L3-F12 (second leg freezes) → eventually L3-F01/L3-F02 tilt fault or mechanical stop.
**How to distinguish from other causes**: Perform the nudge test: `body_pose_to_pulses({5,0,0,0,0,0}, pulses)` and verify `pulses[4]` (RL_hip) changes in the direction symmetric to FL_hip. If RL moves opposite, polarity is wrong. Distinct from Chain 1 in that the asymmetry begins immediately on the first walk command, not after a `cmd_transform`.

---

### Chain 3: gait_update_imu Gated on Connected → Host Disconnect → Stale IMU → Missed Tilt Fault → Servo Fight
**Trigger**: Host TCP connection drops (without triggering the 10 s heartbeat timeout — e.g., `bark server` process killed while firmware remains running over WiFi).
**Propagation**: L4-F07 (`gait_update_imu()` stops being called because `connected = false`) → `s_pitch`/`s_roll` freeze at last-known safe values → L3-F01 or L3-F02 tilt safety block reads stale values that never cross 50° → no `tilt_fault` event fires → if robot is tipped: L4-F05/L4-F06 balance PID continues outputting corrections based on frozen error → servos fight gravity at last-known correction angle → hardware damage risk over 10 s window → heartbeat fires → servos detach.
**How to distinguish from other causes**: Confirm the host is disconnected (server logs show no active client). Observe that `telem_imu` frames stop arriving in the browser. The key indicator is a 10 s window of motor noise without any `tilt_fault` event in the log before `heartbeat_detach`.

---

### Chain 4: cmd_servo Pauses Gait → Tilt Safety Bypass → No Fault Event → No Stop → Mechanical Damage
**Trigger**: Operator sends `cmd_servo` to manually position a leg (e.g., for calibration).
**Propagation**: L3-F01 (`cmd_servo` calls `gait_pause()`, setting `s_paused = true`) → tilt safety block in `gait_update()` is never reached while `s_paused` is true → robot tips past 50° → no `tilt_fault` emitted → L4-F07 balance PID continues outputting corrections based on the frozen pitch value → servos hold the commanded position while the robot falls → heartbeat watchdog (10 s) is the only recovery path → possible servo or linkage damage during fall.
**How to distinguish from other causes**: Check whether `cmd_servo` was recently sent (server logs show `cmd_servo` before the incident). Distinct from Chain 3 in that the host is still connected — telemetry is flowing, `gait_update_imu()` is updating, but the tilt block is gated out by `s_paused`, not by stale IMU data.

---

### Chain 5: UI FORWARD Held → WS Disconnect (Alt-Tab) → No Stop Sent → Firmware Heartbeat Expires (10 s) → Detach
**Trigger**: Operator holds "W" key (FORWARD) and switches to another application.
**Propagation**: L7-F02 (`window.blur` doesn't fire stop; `pressed["w"]` remains true) → L7-F01 (WS `onclose` fires but no stop command sent before socket closes) → firmware receives no `cmd_move stop` → `L6-F02` pattern: server may reset motion state but firmware doesn't receive a stop → firmware continues walking for up to `HEARTBEAT_TIMEOUT_MS = 10000 ms` → `heartbeat_detach` fires → servos detach → robot collapses from walking position.
**How to distinguish from other causes**: Check server log for `cmd_move stop` sent within 1 s of the WS disconnect. Absence of that log line confirms this chain. Distinct from Chain 3: the host server itself is running; only the browser tab disconnected.

---

### Chain 6: Transport Error Mid-Balance-Enable → Flag Desync → Host Thinks Balance ON → Firmware Has It OFF → Dog Falls Without Correction
**Trigger**: TCP/serial transport error occurs at the exact moment `set_enabled(True)` is called in `host/behaviors/balance.py`.
**Propagation**: L5-F01 (state-before-send desync: `_balance_enabled = True` set before `send_json` raises `ConnectionError`) → L6-F07 (reconnect loop opens new transport but doesn't re-send `cmd_balance`) → L6-F03 (transport swap creates fresh `BalanceLayer` with `_balance_enabled=False`, but if the reconnect doesn't use `_replace_transport`, the stale flag persists) → server broadcasts `"balance":True` to UI → UI shows "Balance: ON" → firmware's balance is actually disabled → robot loses active tilt correction → L5-F03 (recovery doesn't re-enable) → next physical disturbance tips the dog with no correction.
**How to distinguish from other causes**: Compare the server's `_balance.enabled` (shown in UI) against the firmware's last `telem_status` balance field. If they disagree, this chain is active. Distinct from Chain 3: IMU is updating correctly; balance indicator in UI is wrong.

---

### Chain 7: Firmware cmd_balance During Tilt Hold → `s_balance_was_enabled` Override → UI Indicator Permanently Wrong
**Trigger**: Operator disables balance during a tilt fault hold period (within 1 s after tilt resolves).
**Propagation**: L3-F05 (`cmd_balance {enabled:false}` during hold is silently overridden when the hold expires at 1 s, re-enabling balance) → L5-F02 (firmware-side balance enable via `s_balance_was_enabled` restore emits a `telem_status`, but `_balance_enabled` in the host is not updated from incoming firmware telemetry) → UI shows whatever the operator last toggled, which was "OFF" → UI shows "Balance: OFF"; firmware has balance "ON" → operator believes balance is disabled; corrective motions appear unintended.
**How to distinguish from other causes**: The trigger is a `telt_fault` in the log followed by a `cmd_balance {enabled:false}` within the 1 s hold window. Inject a `telem_status {"balance":true}` from firmware and verify the UI does not update.

---

### Chain 8: Malformed `cmd_gait_config` → Out-of-Range GaitConfig → Silent Clamp → Leg Freeze → No Telemetry
**Trigger**: Host sends `cmd_gait_config` with `stride_length_mm` or `stride_height_mm` outside the physical workspace (e.g., via a bug in a scripting client, not the standard UI).
**Propagation**: L1-F07 (no validation in `gait_tick_ik`; negative or very large values propagate directly to foot targets) → L2-F03 (silent clamp in `angle_to_pulse` masks the out-of-range IK solve; `leg_ik` returns `true` with wrong angles) → L3-F12 (walk-branch holds last servo position for the clamped leg; no error emitted) → balance continues correcting based on the asymmetric foot Z reported by mock physics → L4-F04 (balance output compounds the bad transform) → indefinite asymmetric gait with no telemetry event distinguishing this from a normal walk.
**How to distinguish from other causes**: Check `cmd_gait_config` history in server logs for non-default values. Reduce stride params to defaults; if gait normalizes, this chain is confirmed. Distinct from Chain 2 in that all four legs are affected symmetrically at first; the asymmetry emerges from clamping hitting the joint limits on one side first.

---

## 4. Symptom → Likely Layer Lookup Table

| Symptom | Most Likely Layer(s) | Failure Mode IDs | Quick Check |
|---|---|---|---|
| Dog limps or circles during walk | L2, L3 | L2-F02, L3-F12, L1-F07 | Nudge test: `body_pose_to_pulses` with small forward shift; verify RL_hip changes direction symmetrically with FL_hip |
| One leg freezes mid-gait | L3, L2 | L3-F12, L2-F03, L2-F08 | Observe 3D viewer for leg stuck at fixed angles; check if `foot_to_pulses` returned false by adding debug log; try reducing stride length |
| Robot drifts right/left during straight walk | L1, L2 | L1-F05, L2-F02, L2-F04 | Send `cmd_gait_params {stride_length:0}` — if drift stops, cause is in gait kernel; if it doesn't, likely hardware misalignment or polarity |
| Servos jitter rapidly while standing | L4, L2 | L4-F06, L4-F03, L2-F11 | Disable balance (`cmd_balance {enabled:false}`); if jitter stops, cause is balance PID noise. Check `kd` value — default 0.05 should be safe |
| Dog falls immediately on engage | L3, L1 | L3-F02, L1-F03, L2-F02 | Observe tilt angle before engage; check if `tilt_fault` fires during ramp; verify standing pose pulses are correct |
| Walk command silently has no effect | L3, L6 | L3-F04, L6-F07, L5-F01 | Check if tilt hold is active (within 1 s of last `tilt_fault`); check lock state; check `_engaged` server flag vs firmware `telem_status` |
| Balance indicator mismatch (UI says ON, behavior suggests OFF) | L5, L6 | L5-F01, L5-F02, L6-F03, L6-F07 | Compare `telem_status` balance field from firmware against `_balance.enabled` in server; re-enable from UI and observe whether firmware ACKs |
| Tilt fault fires spontaneously during normal walk | L4, L2 | L4-F04, L2-F11, L4-F06 | Reduce `cmd_transform` to zero; disable balance; observe if `tilt_fault` still fires. If mock: tilt fault via normal gait is nearly impossible — must be direct `gait_update_imu()` injection |
| Dog doesn't stop when UI stop command sent | L7, L6 | L7-F01, L7-F02, L6-F02 | Check server log for `cmd_move stop` sent after UI stop event; check whether WS was open at send time; verify `cmd_reset` path actually sends stop |
| 3D visualization shows wrong pose | L2, L7 | L2-F11, L2-F04, L7-F05 | Check `telem_servo` or `telem_imu` raw values against 3D pose; if post-disconnect, see L7-F05 |
| Walk animation continues after disconnecting | L7 | L7-F05 | Verify `ws.onclose` handler resets `state.currentMotion` to "stop"; currently this reset is not implemented |
| Robot stands at non-neutral pose after re-engage | L3 | L3-F06 | Check `s_target_transform` state on re-engage; send `cmd_transform {roll:0, pitch:0, ms:0}` to reset |
| Stride stutter on yaw trim save | L3 | L3-F10 | Observe IMU telemetry gap after `cmd_yaw_trim {op:"save"}`; only save while standing |
| All legs drag without lifting | L1 | L1-F02 | Verify `swing_time_ms > 0` in current `GaitConfig`; log `duty` value in `gait_tick_ik` |
| All legs airborne simultaneously | L1 | L1-F03 | Emergency: immediately send `cmd_move stop`. Verify `stand_time_ms > 0` in GaitConfig |

---

## 5. Test Matrix

| ID | Name | Severity | Mock test | Hardware test | Unit test | Currently covered? |
|---|---|---|---|---|---|---|
| L1-F01 | Zero total time — wrong duty fallback | degraded | `gait_tick_ik` with swing=0, stand=0 | Observe walk when stopped | — | No — `test_gait_ik.cpp` doesn't cover this case |
| L1-F02 | All legs in stance — swing_time_ms=0 | degraded | Sweep phase with swing=0 | Observe no foot lift | — | No |
| L1-F03 | All legs airborne — stand_time_ms=0 | unsafe | Verify all z>0 simultaneously | Do not test without frame | — | No |
| L1-F04 | Frozen phase at speed=0 | degraded | Walk ~125ms, set speed=0 | Observe hovering legs | — | No |
| L1-F05 | Yaw trim ignored during TURN | degraded | `gait_tick_ik` with TURN_RIGHT + trim | Walk circles with trim set | — | No |
| L1-F06 | Yaw trim denormal comparison | cosmetic | Pass `5e-45f` as trim | — | — | No |
| L1-F07 | Negative/OOR GaitConfig values | degraded | Negative stride_height + sweep | — | — | No |
| L1-F08 | Diagonal pair asymmetry at TURN+speed=0 | degraded | `gait_tick_ik` TURN_LEFT at speed=0 | Observe shaking in place | — | No |
| L1-F09 | Phase wrap — single subtract | cosmetic | `gait_tick_ik(100000.0f, ...)` | — | — | No |
| L2-F01 | IK_HIP_Z unverified | degraded | `test_ik` stock_ground_truth with 0.5mm gate | Caliper measurement | `test_ik.cpp` test 2 (2mm gate) | Partial — test exists but gate is too generous |
| L2-F02 | RL servo polarity unverified | unsafe | Nudge test via `body_pose_to_pulses` | Dog in sling, slow gait offset | — | No |
| L2-F03 | Tautological IK sanity check | degraded | Force hip pulse >2300, verify `leg_ik` returns true | — | — | No |
| L2-F04 | 4-bar knee linkage not modeled | degraded | Compare `pulses_to_foot` at swing peak | Ruler measure foot lift | — | No |
| L2-F05 | cal_table() data race | cosmetic | Build mock with `-fsanitize=thread` | — | — | No |
| L2-F06 | Lateral translation silently ignored | degraded | `body_pose_to_pulses({0,10,...})` == standing | — | `test_transform.cpp` | Partial — behavior present, test may not assert explicitly |
| L2-F07 | Partial IK failure — asymmetric pose | degraded | `body_pose_to_pulses({0,0,70,...})` | Aggressive body raise | `test_transform.cpp` test 4 (all-legs) | Partial — one-leg case not covered |
| L2-F08 | Elbow-forward hardcoded | degraded | Far-forward foot target + check elbow-back | — | — | No |
| L2-F09 | us_per_rad low sensitivity for RL | degraded | `test_ik` with 0.5mm gate on RL | — | `test_ik.cpp` test 2 | Partial — gate too generous |
| L2-F10 | lerp_pose unclamped t | cosmetic | `lerp_pose(a, b, -0.5f)` | — | `test_transform.cpp` | No — negative t not tested |
| L2-F11 | Mock IMU oscillates at gait frequency | degraded | `bark mock` + observe telemetry pitch oscillation | Compare to hardware IMU | — | No |
| L3-F01 | Tilt bypass while paused | unsafe | `gait_pause()` + `gait_update_imu(60,0)` + `gait_update()` | `cmd_servo` then tip | `test_engage.cpp` (may cover) | No — specific paused+tilt path not tested |
| L3-F02 | Tilt bypass during engage ramp | unsafe | `cmd_engage` + inject 60° before `engage_complete` | — | `test_engage.cpp` | No |
| L3-F03 | WALK↔TURN foot jump | degraded | Record servo positions tick before/after FORWARD→TURN | Listen for servo jump | `test_gait_ik.cpp` | No |
| L3-F04 | Walk command accepted during tilt hold | degraded | Trigger fault, send `cmd_move` at 600ms | — | — | No |
| L3-F05 | Balance re-enable overrides cmd_balance during hold | degraded | Enable balance, fault, disable during hold, wait re-arm | — | — | No |
| L3-F06 | Stale transform across engage cycles | degraded | Engage→transform→disengage→engage | — | — | No |
| L3-F07 | Phase not reset on state transition | cosmetic | Walk 83ms→stop→resume, check foot Z symmetry | — | `test_gait_taper.cpp` | No |
| L3-F08 | Heartbeat-detach stand ramp race | cosmetic | Trigger heartbeat during walk | — | — | No |
| L3-F09 | millis() wrap | cosmetic | Mock clock at UINT32_MAX-5, advance 10ms | — | — | No |
| L3-F10 | NVS write from main loop stutter | degraded | Send `cmd_yaw_trim {op:"save"}` while walking | Observe servo pause | — | No |
| L3-F11 | Config blend instant convergence at large dt | cosmetic | Pause gait 400-500ms, then change config | — | `test_gait_ik.cpp` | No |
| L3-F12 | Walk-branch IK failure — leg freeze | degraded | `cmd_transform {roll:15,pitch:15,z:10}` while walking | Aggressive body pose | — | No |
| L3-F13 | Deferred reversal threshold | cosmetic | FORWARD→BACKWARD at full speed | — | `test_gait_taper.cpp` | Partial |
| L3-F14 | Stale stand ramp capture on mid-taper disengage | cosmetic | Disengage during stand taper | — | `test_engage.cpp` | No |
| L4-F01 | Integral spike on ki enable | degraded | 100 ticks at ki=0 then set ki=0.1, observe output | — | `test_balance.cpp` | Partial — Test 6 covers set_gains, not ki=0→ki≠0 |
| L4-F02 | Derivative spike on re-enable | degraded | Disable, change angle, re-enable without reset | — | `test_balance.cpp` | No |
| L4-F03 | Deadband flip-flop | degraded | Oscillate pitch 0.4°/0.6°, observe integral | — | `test_balance.cpp` | No |
| L4-F04 | Balance + transform compounds past IK | degraded | `cmd_transform {pitch:20}` + balance + 10° IMU | `cmd_transform` + balance | — | No |
| L4-F05 | Mock physics PID fidelity | cosmetic | Compare mock vs hardware balance stability | Hardware kp/kd sweep | — | No |
| L4-F06 | Derivative amplifies IMU noise | degraded | Hardware only with `kd=0.3` | Hardware servo jitter | — | No (hardware only) |
| L4-F07 | gait_update_imu gated on connected | unsafe | Disconnect host, tip mock (inject via physics) | Disconnect server, tip dog | — | No |
| L5-F01 | State-before-send desync | degraded | Kill mock TCP mid-enable | — | — | No |
| L5-F02 | tilt_fault doesn't reset host flag | degraded | Inject `telem_status {balance:false}` at mock port | — | — | No |
| L5-F03 | No re-enable after recovery | degraded | Inject IMU >35° then <20° | — | — | No |
| L5-F04 | Frozen IMU cache | degraded | Kill mock firmware mid-session | — | — | No |
| L5-F05 | Zero-initialized IMU cache at boot | cosmetic | Start server while dog tilted, before telem | — | — | No |
| L5-F06 | Balance flapping latent | cosmetic | Script 100 rapid `cmd_balance` sends | — | — | No |
| L6-F01 | Blocking drain() stalls telemetry | unsafe | Mock that stops reading from socket | — | — | No |
| L6-F02 | cmd_reset doesn't stop firmware | unsafe | `cmd_move forward` then click Reset | — | — | No |
| L6-F03 | Transport swap resets balance state | degraded | Unplug USB mid-session with balance enabled | — | — | No |
| L6-F04 | ControlLock multi-tab race | degraded | Two tabs, simultaneous forward press | — | — | No |
| L6-F05 | WS zombie holds lock | degraded | Kill browser tab process mid-lock-hold | — | — | No |
| L6-F06 | Web hash misses subdirectories | degraded | Edit `web/modules/controls.js`, restart server | — | — | No |
| L6-F07 | Reconnect doesn't re-sync state | degraded | Disconnect and reconnect transport | — | — | No |
| L7-F01 | No stop on WS disconnect | unsafe | Hold FORWARD, simulate offline in devtools | Hold FORWARD, unplug cable | — | No |
| L7-F02 | Alt-tab drops keyup | unsafe | Hold W, alt-tab, wait, return | Physical test | — | No |
| L7-F03 | Fall alert overwritten | degraded | Trigger fall, send lock_denied from second tab | — | — | No |
| L7-F04 | Balance toggle not gated on engaged | degraded | Disengage, press "b" | — | — | No |
| L7-F05 | Stale 3D animation after disconnect | cosmetic | Start walking, kill server | — | — | No |
| L7-F06 | Rapid direction toggles | degraded | Script rapid W/S alternation | — | — | No |
| L7-F07 | Battery percent hardcoded range | cosmetic | Inject `telem_battery {voltage_mv:5000}` | — | — | No |

---

## 6. Known-Unverified Items

The following items are explicitly marked as unverified in the source code. Each represents an assumption that, if wrong, produces a specific set of failure modes. Hardware verification is required before these can be removed from this list.

---

### UV-1: IK_HIP_Z = -25.0 mm
**File**: `firmware/include/ik.h`
**Current assumption in code**: The hip pivot Z offset is -25.0 mm below the body center plane. Used in `hip_position(leg)` and `standing_angles()`.
**Why it matters**: Every gait offset, body transform, and balance correction is computed relative to this origin. If the real offset is, say, -20 mm or -30 mm, the IK model's prediction of foot position is systematically wrong by an amount proportional to the deviation from standing. Balance oscillates because the actual foot height does not match the commanded height. Stride length and height both differ from configured values.
**How to verify**: With calipers, measure the physical distance from the body center mounting surface to the hip pivot pin along the vertical axis, with the robot at rest. Compare to -25.0 mm. Alternatively, tighten the gate in `test_ik.cpp` test 2 (stock_ground_truth) from 2 mm to 0.5 mm and sweep IK_HIP_Z values until the test passes.
**Failure modes affected**: L2-F01 (systematic foot position error), L2-F09 (us_per_rad derivation), L4-F04 (balance compounds the error), L3-F12 (IK failure from compounded error).

---

### UV-2: RL Servo Polarity Post-Swap (`SERVO_POLARITY_OVERRIDE[4,5]`)
**File**: `firmware/include/config.h`
**Current assumption in code**: `SERVO_POLARITY_OVERRIDE[4] = 0` (auto-derive for RL_hip → +1, standing=2170) and `SERVO_POLARITY_OVERRIDE[5] = 0` (auto-derive for RL_knee → +1, standing=1611). Both marked `UNVERIFIED post-swap` in the config comment.
**Why it matters**: If either physical polarity is actually -1, every IK command to a non-standing RL position drives that servo in the wrong direction. The RL leg kicks backward or outward on the first walk command. The asymmetric gait drives balance oscillation and eventually a tilt fault or mechanical stop. This is the highest-priority verification item.
**How to verify**: With the robot in a sling (suspended so legs hang free): engage servos, then send `body_pose_to_pulses({5,0,0,0,0,0}, pulses)` (small forward body shift). Observe the physical direction RL_hip moves. Compare to FL_hip — they should be mirror-symmetric. If RL_hip moves the same direction as FL_hip (not mirrored), the polarity needs to be overridden to -1.
**Failure modes affected**: L2-F02 (polarity wrong → asymmetric gait), L3-F12 (IK freeze from asymmetry), L4-F06 (derivative amplification), L1-F05 (yaw trim doesn't help during turns).

---

### UV-3: 4-Bar Knee Linkage Transmission Ratio (IK_L3/IK_L4/IK_L5)
**File**: `firmware/include/ik.h`
**Current assumption in code**: `IK_L3 = 14.5f`, `IK_L4 = 14.0f`, `IK_L5 = 14.0f` mm are declared, but `knee_virtual_to_servo()` is identity (`return virtual_angle`) until the ground-link length is measured. The actual transmission ratio `ε = f(L3, L4, L5, ground_link_length)` is not computed.
**Why it matters**: The servo drives the shin through a parallel 4-bar linkage. The IK computes a virtual knee angle (the angle of the shin segment in the 2-link model) and sends it directly to the servo as if `servo_angle = shin_angle`. The actual ratio is ε ≠ 1. Estimated systematic shin angle error of 5–10% propagates to every foot position, making all gait heights and stride lengths differ from commanded values. Balance oscillates because the actual body height differs from the commanded height.
**How to verify**: Physical measurement: with the knee servo driven to a known pulse (e.g., `STANDING_POSE[1]`), measure the shin angle relative to the thigh using a protractor or inclinometer. Drive to a second known pulse. Compute the ratio `Δshaft_angle / Δshin_angle`. Compare to the theoretical ratio from L3/L4/L5 geometry. Alternatively: measure actual foot lift height with a ruler during normal gait and compare to `GAIT_STRIDE_HEIGHT_MM = 10.0`.
**Failure modes affected**: L2-F04 (identity transmission used), L2-F09 (standing_foot_pos accuracy), L4-F05 (mock physics fidelity limited by this error).

---

### UV-4: Heading vs Movement Coordinate System Mismatch
**Files**: `firmware/src/gait.cpp`, `firmware/include/gait_math.h`, related to IMU yaw axis convention
**Current assumption in code**: Gait direction commands (`WALK_FORWARD`, `TURN_LEFT`, etc.) are interpreted relative to the robot's body frame. IMU yaw is reported in a convention that may or may not align with the gait kernel's x-axis. This mismatch was suspected pre-IK pipeline landing and has not been re-verified after the AEP/PEP kernel replacement.
**Why it matters**: If the heading convention in the gait kernel (which foot leads on WALK_FORWARD) disagrees with the IMU yaw convention (which direction is "forward" for balance correction and turn commands), turns will drift in the wrong direction and yaw trim will be applied with the wrong sign. The failure is subtle because the robot still walks — it just doesn't go exactly where commanded.
**How to verify**: On flat ground: engage, walk FORWARD for 3 seconds. Measure IMU yaw change. It should be ~0° (no rotation). Then TURN_LEFT for 3 seconds. Measure IMU yaw change. It should be positive (left turn in the IMU convention) or negative, consistently. Log the heading change and verify it matches the operator's expectation for TURN_LEFT. If TURN_LEFT produces a right turn in yaw, the convention is inverted.
**Failure modes affected**: L1-F05 (yaw trim abandoned during turns — trim sign may also be wrong), L2-F02 (RL polarity interacts with heading convention), L4-F04 (balance correction direction depends on correct IMU axis mapping).

---

## 7. Unsafe Failure Modes Summary

The following failure modes are classified as **unsafe** — they can result in uncontrolled robot motion, servo damage, or physical injury to the robot or operator. All require mitigation before unattended or high-speed operation.

| ID | Name | Mitigation Status |
|---|---|---|
| L1-F03 | All legs simultaneously airborne (`stand_time_ms=0`) | Mitigated by `config.h` defaults and UI constraints; no runtime validation in firmware |
| L2-F02 | RL servo polarity unverified post-swap | No fix — hardware verification required (see UV-2) |
| L3-F01 | Tilt safety bypassed while paused (`cmd_servo`) | No fix — tilt block must be moved outside the `s_paused` gate |
| L3-F02 | Tilt safety bypassed during engage ramp | No fix — tilt block must be moved outside the `servos_is_ramping()` return |
| L4-F07 | `gait_update_imu()` gated on host connected | No fix — `gait_update_imu()` must be decoupled from the `connected` flag |
| L6-F01 | Blocking `drain()` stalls telemetry loop | No fix — `drain()` needs `asyncio.wait_for` with timeout |
| L6-F02 | `cmd_reset` doesn't send stop to firmware | No fix — `cmd_reset` handler must send `cmd_move stop` before resetting server state |
| L7-F01 | Walking robot not stopped on WS disconnect | Mitigated by firmware 10 s heartbeat; no software stop guarantee |
| L7-F02 | Alt-tab drops key release events | Mitigated by firmware 10 s heartbeat; no software stop guarantee |
