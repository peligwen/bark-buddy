#pragma once

// ============================================================
// Onboard Physics Model — design notes / TODO
//
// This file is a stub. It documents the planned onboard model
// for Bark-Buddy custom firmware ("Mutt"). Nothing here is
// implemented yet. The shared math kernels that this model
// would depend on are already in place:
//   firmware/include/gait_math.h   — trot gait kernel
//   firmware/include/cf_filter.h   — complementary IMU filter
//   firmware/test/kinematics.h     — forward kinematics
//
// All three are header-only with no Arduino dependencies and
// can run natively (in tests and on-chip).
// ============================================================

// ---- TODO: Leg Contact Estimation -------------------------
//
// Problem: MechDog has no contact sensors. The balance PID
// currently reacts to pitch/roll after the fact.
//
// Approach:
//   1. Call gait_tick() to get current hip/knee targets.
//   2. Feed targets into leg_fk() (kinematics.h) to estimate
//      foot positions in body frame.
//   3. Transform foot Y to world frame using cf_filter pitch/roll.
//   4. A foot with estimated world-Y < CONTACT_THRESHOLD and
//      gait phase in stance window (sinA < 0 for pair A) is
//      classified as grounded.
//
// Output: a 4-bit contact mask, updated each gait tick (~50 Hz).
// Useful for: balance correction gating, MPC (below), and
// streaming as a new telemetry field `telem_contact`.
//
// Prerequisite: kinematics.h moved from firmware/test/ into
// firmware/include/ so gait.cpp can include it.
// Estimated effort: 1–2 days.

// ---- TODO: Model Predictive Control (MPC) -----------------
//
// Problem: the host-side balance PID reacts to pitch/roll error.
// A predictive model could anticipate pitch disturbances one
// gait cycle ahead and pre-correct servo angles.
//
// Approach (lightweight, runs on core 1 at 50 Hz):
//   1. Maintain a minimal on-chip state: body pitch, roll,
//      pitch_dot, roll_dot (from CF filter), plus gait phase.
//   2. Predict pitch/roll one step ahead using a linear model
//      fit from logged sim-to-real data (see below).
//   3. Add a feedforward correction to the balance PID output
//      proportional to the predicted error.
//
// This doesn't replace the host PID — it runs underneath it,
// reducing the round-trip latency of the WiFi control loop.
//
// Prerequisite: leg contact estimation (above) for traction
// gating — don't apply MPC correction when airborne.
// Estimated effort: 1 week (model fitting + tuning).

// ---- TODO: Sim-to-Real Consistency Checking ---------------
//
// Problem: the physics sim and real hardware can diverge silently.
// Gait parameter changes that work in sim may behave differently
// on hardware; no current mechanism surfaces this.
//
// Approach:
//   1. Run the gait_tick() + lightweight kinematics on-chip to
//      produce a predicted pitch/roll each cycle.
//   2. Compare prediction to cf_filter output (measured pitch/roll).
//   3. Compute a rolling RMS divergence over a 1-second window.
//   4. Stream it as `model_err_deg` in telem_imu.
//
// On the host:
//   - server.py logs model_err_deg alongside telem_imu.
//   - If model_err_deg > threshold (e.g. 5°), emit a warning
//     and optionally pause autonomous behaviors (patrol/scan).
//   - A calibration tool can regress model error vs gait params
//     to auto-tune the sim's contact stiffness / damping.
//
// This makes hardware regression visible: if a firmware change
// increases model_err_deg, the gait model no longer fits reality.
// Estimated effort: 3–4 days (firmware side + host logging).
