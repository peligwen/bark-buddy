# Hardware TODO — Needs Real Dog

Tasks that are blocked until the MechDog is plugged in and powered on.
Ordered roughly by priority.

---

## 1. Hybrid Mode Smoke Test

First thing to run when the dog is plugged in.

- `python3 server.py --hybrid`
- Confirm boot message appears in console
- Open web UI, verify:
  - IMU pitch/roll updating in real time
  - Sonar distance updating
  - Battery voltage/percentage shown
- Try each D-pad direction for 1-2 seconds
- Confirm the dog actually walks (not just 3D view)
- Check if forward still drifts left (known issue from stock firmware reflash)

## 2. Validate Mock Physics Model Against Real Dog

Run sim alongside real hardware, compare telemetry side by side.

**Goal:** Quantify how far off the mock firmware's physics model is from reality, then tune it.

**Method:**
- Start hybrid transport (real dog)
- Log all `telem_imu` to a timestamped NDJSON file
- Execute a scripted motion sequence: stand → forward 2s → stop 1s → left 1s → stop 1s → right 1s → stop
- Replay the same motion command sequence through `MockFirmwareTransport`
- Compare traces:
  - Pitch/roll offset and noise profile
  - Response lag (how quickly IMU settles after stop)
  - Drift during forward walk
  - Turn rate vs commanded turn
- Output: MSE per axis, lag estimate, drift rate, recommended parameter adjustments

**Needs:** A `host/validate_model.py` script that drives both transports and produces comparison plots/stats.

## 3. Servo-to-Leg Mapping Validation

The servo-to-joint mapping is only partially confirmed via IMU response.
Back left leg was tucking in — polarity or assignment is wrong somewhere.

- With dog standing on flat surface, run `identify_servos.py` via hybrid mode's `cmd_servo`
- Move each servo one at a time in 5μs steps, record IMU deltas
- Match observed pitch/roll axis + direction to FK model predictions
- Produce a definitive mapping: servo index → joint name → polarity
- Update `config.h` SERVO_POLARITY and STANDING_POSE arrays

**Safety:** Servos get warm. Take breaks. Check delta from baseline not absolute tilt.

## 4. Movement Direction Calibration

After the stock firmware reflash, movement is off:
- Forward drifts left
- Right turn barely turns
- Direction signs may be wrong

**Method:**
- Use IMU yaw to measure actual turn rate for `move(20, -50)` and `move(20, 50)`
- Use dead reckoning + IMU to measure forward drift angle
- Adjust MOTION_CMDS speed/direction values in `handler.py` until:
  - Forward goes straight (< 5° drift over 2s)
  - Left/right turn at roughly equal rates
- Consider per-servo offset calibration via Hiwonder upper computer

## 5. Calibration Sweep for Physical Model Fitting

Use the calibration protocol to sweep each servo and build a response profile.

- For each servo (0-7): sweep ±100μs from standing in 10μs steps, 300ms dwell
- Record IMU pitch/roll at each step
- Output calibration NDJSON for `profile_analyzer`
- Fit: COM offset, body inertia, servo lag, damping
- Compare predicted vs actual IMU traces (target MSE < 1.0)

**Needs:** `host/calibrate_servos.py` (started but not tested on hardware)

## 6. WiFi TCP Testing

Custom firmware has WiFi TCP listener on port 9000 but hasn't been tested.

- Flash custom firmware with `WIFI_ENABLED` and credentials
- Verify TCP connection from host to dog at port 9000
- Test NDJSON command/telemetry flow over WiFi
- Measure latency vs serial (expect ~5-10ms additional)
- If WiFi works, hybrid mode could also work over WiFi (upload handler via WebREPL, then TCP for NDJSON)

## 7. Gait Engine Validation (Custom Firmware)

Once servo mapping and polarity are confirmed:

- Flash custom firmware with corrected SERVO_POLARITY
- Test standing pose — all 4 legs should be symmetrical
- Run gentle gait: 0.5s forward at 0.3 speed
- Compare real IMU to `physical_model.h` prediction
- Verify pitch/roll stays within ±5° during walk
- If good, gradually increase speed and duration

## 8. Battery Calibration

The ADC→voltage conversion uses an estimated 3.9x divider ratio.

- Measure actual battery voltage with multimeter
- Read ADC value at same time
- Calculate true divider ratio
- Update `_read_battery()` in handler.py and `imu.cpp` in custom firmware
- Verify low-battery threshold (currently 6.8V) is appropriate

---

## Notes

- Servos get warm during extended testing — take breaks between tasks
- Always return to standing pose before unplugging
- The `manual_servo_mode` flag in custom firmware must be cleared before gait works
- IMU returns [roll, pitch] not [pitch, roll] — already handled in handler.py
