#!/usr/bin/env python3
"""
Servo identification and mapping validation for MechDog custom firmware.

Moves each servo index one at a time in tiny increments, reads IMU at each step,
and compares the observed pitch/roll response to the kinematic model's prediction
for each possible joint assignment. Outputs a confidence-ranked mapping.

This script is careful and slow:
- 5μs steps, 200ms dwell between steps
- IMU averaged over 3 readings per step
- Abort if tilt exceeds safety threshold
- Compare observed tilt axis/direction to FK predictions
"""

import json
import math
import serial
import sys
import time

# URDF dimensions from kinematics.h
UPPER_LEN = 0.055   # m
LOWER_LEN = 0.060   # m
HIP_OFFSET_X = 0.065  # m (front/back from center)
HIP_OFFSET_Z = 0.040  # m (left/right from center)
STAND_HIP_RAD = 0.3
STAND_KNEE_RAD = -0.6

STANDING = [2096, 1621, 2170, 1611, 904, 1379, 1389, 830]
PINS = [25, 26, 27, 14, 16, 17, 4, 2]
JOINT_LABELS = ['FL_hip', 'FL_knee', 'FR_hip', 'FR_knee',
                'RL_hip', 'RL_knee', 'RR_hip', 'RR_knee']

# Safety
TILT_LIMIT = 12.0   # degrees
STEP_US = 5          # microseconds per step
MAX_STEPS = 40       # max 200μs sweep
DWELL_S = 0.15       # seconds between steps
IMU_SAMPLES = 2      # readings to average


def leg_fk(leg_idx, hip_angle, knee_angle):
    """Forward kinematics for one leg. Returns foot position (x, y, z)."""
    # Hip offset from body center
    x_sign = 1 if leg_idx < 2 else -1  # front positive, rear negative
    z_sign = 1 if leg_idx % 2 == 0 else -1  # left positive, right negative
    hip_x = x_sign * HIP_OFFSET_X
    hip_z = z_sign * HIP_OFFSET_Z
    hip_y = -0.025  # below body center

    # Upper leg
    upper_y = -UPPER_LEN * math.cos(hip_angle)
    upper_x = UPPER_LEN * math.sin(hip_angle)
    knee_x = hip_x + upper_x
    knee_y = hip_y + upper_y

    # Lower leg
    total = hip_angle + knee_angle
    lower_y = -LOWER_LEN * math.cos(total)
    lower_x = LOWER_LEN * math.sin(total)
    foot_x = knee_x + lower_x
    foot_y = knee_y + lower_y

    return foot_x, foot_y, hip_z


def predict_tilt_for_joint(joint_idx, delta_rad):
    """
    Predict how much pitch and roll changes when one joint moves by delta_rad.
    Uses simplified statics: moving a foot shifts the support polygon,
    causing the body to tilt.
    """
    leg = joint_idx // 2
    is_hip = joint_idx % 2 == 0

    # Compute foot position at standing and with perturbation
    angles_stand = [STAND_HIP_RAD, STAND_KNEE_RAD] * 4
    hip_a = STAND_HIP_RAD
    knee_a = STAND_KNEE_RAD
    if is_hip:
        hip_a += delta_rad
    else:
        knee_a += delta_rad

    fx0, fy0, fz0 = leg_fk(leg, STAND_HIP_RAD, STAND_KNEE_RAD)
    fx1, fy1, fz1 = leg_fk(leg, hip_a, knee_a)

    # Foot displacement
    dx = fx1 - fx0  # forward/backward
    dy = fy1 - fy0  # up/down

    # Simplified tilt prediction:
    # - Hip movement shifts foot forward/backward → primarily affects PITCH
    # - Knee movement shifts foot up/down → affects both PITCH and ROLL
    # - Left-side legs: positive dz → positive ROLL
    # - Right-side legs: positive dz → negative ROLL
    # - Front legs: positive dx → negative PITCH (nose dips)
    # - Rear legs: positive dx → positive PITCH

    x_sign = 1 if leg < 2 else -1  # front vs rear
    z_sign = 1 if leg % 2 == 0 else -1  # left vs right

    # Rough: pitch changes with foot x-displacement, scaled by lever arm
    # roll changes with unilateral height change
    body_height = 0.09  # approximate standing height
    pitch_delta = -x_sign * math.degrees(math.atan2(dy, body_height)) * 0.5
    roll_delta = z_sign * math.degrees(math.atan2(dy, body_height)) * 0.3

    if is_hip:
        # Hip motion primarily creates pitch via forward/backward foot shift
        pitch_delta += -x_sign * dx * 100  # empirical scaling
    else:
        # Knee motion primarily creates roll via height change on one side
        roll_delta += z_sign * dy * 200

    return pitch_delta, roll_delta


class ServoTester:
    def __init__(self, port):
        self.s = serial.Serial(port, 115200, timeout=3)
        time.sleep(2)
        self._flush()

    def _flush(self):
        self.s.read(self.s.in_waiting)

    def send_json(self, msg):
        self.s.write((json.dumps(msg) + '\n').encode())
        time.sleep(0.05)

    def read_imu(self, samples=1):
        """Read IMU, average over multiple samples."""
        pitches, rolls = [], []
        for _ in range(samples):
            self._flush()
            time.sleep(0.15)
            data = self.s.read(self.s.in_waiting).decode(errors='replace')
            for line in data.split('\n'):
                if not line.strip().startswith('{'):
                    continue
                try:
                    d = json.loads(line.strip())
                    if d.get('type') == 'telem_imu':
                        pitches.append(d['pitch'])
                        rolls.append(d['roll'])
                        break
                except (json.JSONDecodeError, KeyError):
                    continue
        if not pitches:
            return None, None
        return sum(pitches) / len(pitches), sum(rolls) / len(rolls)

    def set_servo(self, idx, us):
        self.send_json({'type': 'cmd_servo', 'index': idx, 'pulse_us': us})

    def set_all_standing(self):
        for i, us in enumerate(STANDING):
            self.set_servo(i, us)
        time.sleep(1)

    def close(self):
        self.s.close()


def run_identification(port):
    t = ServoTester(port)

    # Verify firmware is responding
    t.send_json({"type": "ping"})
    time.sleep(1)
    data = t.s.read(t.s.in_waiting).decode(errors='replace')
    if 'pong' not in data:
        print("ERROR: No firmware response")
        t.close()
        return

    # Stand up, then stop gait engine so cmd_servo works
    print("Standing up...")
    t.send_json({"type": "cmd_stand"})
    time.sleep(3)
    # Stop gait engine — cmd_move stop sets STOP state which still runs gait_update
    # Use calibration mode instead which disables gait
    t.send_json({"type": "cmd_calibrate", "action": "stop"})
    time.sleep(1)
    t.set_all_standing()
    time.sleep(2)

    # Read baseline
    p0, r0 = t.read_imu(IMU_SAMPLES)
    if p0 is None:
        print("ERROR: Cannot read IMU")
        t.close()
        return

    print(f"Baseline IMU: pitch={p0:.2f}° roll={r0:.2f}°")
    print(f"Body orientation: {'upright' if abs(p0) < 15 and abs(r0) < 15 else 'TILTED'}")
    print()

    # Collect per-servo sweep data
    results = []

    for servo_idx in range(8):
        pos = STANDING[servo_idx]
        label = f"Servo {servo_idx} (GPIO {PINS[servo_idx]}, standing={pos}μs)"

        # Return to standing before each test
        t.set_all_standing()
        time.sleep(1)

        baseline_p, baseline_r = t.read_imu(IMU_SAMPLES)
        if baseline_p is None:
            print(f"{label}: IMU ERROR")
            results.append(None)
            continue

        # Sweep positive direction
        sweep_data = []
        aborted = False

        for step in range(1, MAX_STEPS + 1):
            delta = step * STEP_US
            t.set_servo(servo_idx, pos + delta)
            time.sleep(DWELL_S)

            p, r = t.read_imu(IMU_SAMPLES)
            if p is None:
                continue

            dp = p - baseline_p
            dr = r - baseline_r

            sweep_data.append({
                'delta_us': delta,
                'pitch': p, 'roll': r,
                'dp': dp, 'dr': dr,
            })

            # Safety check — limit CHANGE from baseline, not absolute value
            if abs(dp) > TILT_LIMIT or abs(dr) > TILT_LIMIT:
                t.set_servo(servo_idx, pos)
                time.sleep(0.3)
                aborted = True
                break

        # Return to standing
        t.set_servo(servo_idx, pos)
        time.sleep(0.5)

        # If positive didn't show much, try negative
        if not aborted and (not sweep_data or
                (max(abs(d['dp']) for d in sweep_data) < 0.3 and
                 max(abs(d['dr']) for d in sweep_data) < 0.3)):

            for step in range(1, MAX_STEPS + 1):
                delta = -(step * STEP_US)
                t.set_servo(servo_idx, pos + delta)
                time.sleep(DWELL_S)

                p, r = t.read_imu(IMU_SAMPLES)
                if p is None:
                    continue

                dp = p - baseline_p
                dr = r - baseline_r

                sweep_data.append({
                    'delta_us': delta,
                    'pitch': p, 'roll': r,
                    'dp': dp, 'dr': dr,
                })

                if abs(dp) > TILT_LIMIT or abs(dr) > TILT_LIMIT:
                    t.set_servo(servo_idx, pos)
                    time.sleep(0.3)
                    aborted = True
                    break

            t.set_servo(servo_idx, pos)
            time.sleep(0.5)

        # Analyze sweep
        if not sweep_data:
            print(f"{label}: NO DATA")
            results.append(None)
            continue

        max_dp = max(d['dp'] for d in sweep_data)
        min_dp = min(d['dp'] for d in sweep_data)
        max_dr = max(d['dr'] for d in sweep_data)
        min_dr = min(d['dr'] for d in sweep_data)
        peak_dp = max_dp if abs(max_dp) > abs(min_dp) else min_dp
        peak_dr = max_dr if abs(max_dr) > abs(min_dr) else min_dr
        primary = 'pitch' if abs(peak_dp) > abs(peak_dr) else 'roll'
        moved = abs(peak_dp) > 0.3 or abs(peak_dr) > 0.3

        result = {
            'servo_idx': servo_idx,
            'pin': PINS[servo_idx],
            'standing_us': pos,
            'sweep_points': len(sweep_data),
            'peak_pitch': round(peak_dp, 3),
            'peak_roll': round(peak_dr, 3),
            'primary_axis': primary,
            'pitch_sign': '+' if peak_dp > 0 else '-',
            'roll_sign': '+' if peak_dr > 0 else '-',
            'moved': moved,
            'aborted': aborted,
            'sweep': sweep_data,
        }
        results.append(result)

        status = 'OK' if moved else 'WEAK'
        if aborted:
            status += ' LIM'
        print(f"{label}")
        print(f"  {len(sweep_data)} points, peak: pitch={peak_dp:+.2f}° roll={peak_dr:+.2f}° "
              f"primary={primary} {status}")

    # Now compare to FK predictions
    print()
    print("=" * 70)
    print("JOINT IDENTIFICATION — Comparing observed IMU response to FK model")
    print("=" * 70)
    print()

    # For each servo, score it against each possible joint assignment
    print(f"{'Servo':8s} {'Best Match':12s} {'Confidence':11s} {'Observed':20s} {'Predicted':20s}")
    print("-" * 75)

    mapping = {}

    for servo_idx in range(8):
        r = results[servo_idx]
        if r is None or not r['moved']:
            print(f"{servo_idx:8d} {'???':12s} {'LOW':11s} {'no movement':20s}")
            continue

        obs_dp = r['peak_pitch']
        obs_dr = r['peak_roll']
        obs_mag = math.sqrt(obs_dp**2 + obs_dr**2)
        if obs_mag < 0.01:
            obs_mag = 0.01
        obs_unit = (obs_dp / obs_mag, obs_dr / obs_mag)

        best_joint = None
        best_score = -1

        for joint_idx in range(8):
            # Predict tilt for a 10-degree joint movement
            pred_dp, pred_dr = predict_tilt_for_joint(joint_idx, math.radians(10))
            pred_mag = math.sqrt(pred_dp**2 + pred_dr**2)
            if pred_mag < 0.01:
                continue
            pred_unit = (pred_dp / pred_mag, pred_dr / pred_mag)

            # Cosine similarity (direction match)
            cos_sim = obs_unit[0] * pred_unit[0] + obs_unit[1] * pred_unit[1]
            # Also check if flipping sign gives better match (servo polarity)
            cos_sim_neg = -(obs_unit[0] * pred_unit[0] + obs_unit[1] * pred_unit[1])

            score = max(cos_sim, cos_sim_neg)
            if score > best_score:
                best_score = score
                best_joint = joint_idx

        conf = 'HIGH' if best_score > 0.7 else 'MED' if best_score > 0.4 else 'LOW'
        joint_name = JOINT_LABELS[best_joint] if best_joint is not None else '???'
        pred_dp, pred_dr = predict_tilt_for_joint(best_joint, math.radians(10))

        print(f"{servo_idx:8d} {joint_name:12s} {conf:5s} ({best_score:.2f})  "
              f"p={obs_dp:+.2f}° r={obs_dr:+.2f}°   "
              f"p={pred_dp:+.2f}° r={pred_dr:+.2f}°")

        mapping[servo_idx] = {'joint': joint_name, 'score': best_score, 'conf': conf}

    # Save full results
    output = {
        'baseline': {'pitch': p0, 'roll': r0},
        'servos': results,
        'mapping': mapping,
        'pins': PINS,
        'standing': STANDING,
    }
    with open('servo_identification.json', 'w') as f:
        json.dump(output, f, indent=2, default=str)
    print(f"\nFull results saved to servo_identification.json")

    t.set_all_standing()
    t.close()


if __name__ == "__main__":
    port = sys.argv[1] if len(sys.argv) > 1 else '/dev/cu.usbserial-10'
    run_identification(port)
