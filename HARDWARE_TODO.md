# Hardware TODO — Needs Real Dog

Tasks that are blocked until the MechDog is plugged in and powered on.
Ordered roughly by priority.

---

## 1. Custom Firmware Smoke Test

First thing to run when the dog is plugged in.

- `bark` (auto-detects USB serial or mDNS WiFi)
- Confirm boot message appears in console
- Open web UI, verify:
  - IMU pitch/roll updating in real time
  - Sonar distance updating
  - Battery voltage/percentage shown
- Send `cmd_engage {enabled: true}` from UI
- Try each D-pad direction for 1-2 seconds
- Confirm the dog actually walks (not just 3D view)
- Check if forward still drifts left (known issue from prior testing)

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

The servo-to-joint mapping is **untrusted** — prior identification data was collected during a messy code state and should be discarded. `identify_servos.py` and `servo_test.py` also need a code review before running again.

**Pre-requisite:** Review `identify_servos.py` and `servo_test.py` before using on hardware.

- With dog standing on flat surface, run `identify_servos.py` via `cmd_servo`
- Move each servo one at a time in 5μs steps, record IMU deltas
- Match observed pitch/roll axis + direction to FK model predictions
- Produce a definitive mapping: servo index → joint name → polarity
- Update `config.h` SERVO_POLARITY and STANDING_POSE arrays

**Note:** RR hip servo was blown and has been replaced. All 8 servos should be functional.

**Safety:** Servos get warm. Take breaks. Check delta from baseline not absolute tilt.

## 4. Movement Direction Calibration

Movement direction may be off after hardware reassembly or servo replacement:
- Forward may drift left
- Turn rate may be asymmetric

**Method:**
- Use IMU yaw to measure actual turn rate for left/right move commands
- Use IMU to measure forward drift angle over 2s
- Adjust gait parameters via `cmd_gait_params` until:
  - Forward goes straight (< 5° drift over 2s)
  - Left/right turn at roughly equal rates
- Consider per-servo offset calibration via `cmd_offset`

## 5. Calibration Sweep for Physical Model Fitting

Use the calibration protocol to sweep each servo and build a response profile.

- For each servo (0-7): sweep ±100μs from standing in 10μs steps, 300ms dwell
- Record IMU pitch/roll at each step
- Output calibration NDJSON for `profile_analyzer`
- Fit: COM offset, body inertia, servo lag, damping
- Compare predicted vs actual IMU traces (target MSE < 1.0)

**Needs:** `host/calibrate_servos.py` (started but not tested on hardware)

## 6. WiFi TCP Validation

Custom firmware implements WiFi TCP on port 9000 (WIFI_ENABLED build flag, reconnect loop included). Needs end-to-end hardware test.

- Flash custom firmware with `WIFI_ENABLED=1` and credentials in `config_local.h`
- Verify TCP connection from host to dog at port 9000
- Test NDJSON command/telemetry flow over WiFi
- Measure latency vs serial (expect ~5-10ms additional)

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
- Update `BATTERY_DIVIDER` in `config.h` and verify `BATTERY_LOW_MV` threshold is appropriate (currently 6400mV)

---

## Notes

- Servos get warm during extended testing — take breaks between tasks
- Always send `cmd_engage {enabled: false}` before unplugging (servos ramp to rest pose)
- GPIO 2 (RR_knee) is a strapping pin — it is pulled low during boot by the servo PWM line. Boot diagnostic added in firmware; physical relocation to a non-strapping pin is pending.
- IMU returns [roll, pitch] not [pitch, roll] — already handled in firmware and host
