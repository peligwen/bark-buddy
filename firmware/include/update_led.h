// firmware/include/update_led.h
//
// Eye-LED rainbow effect during firmware OTA. Owned by update_led.cpp;
// callers do not touch the active flag directly.
#pragma once

// Activate or deactivate the rainbow effect.
// While active, update_led_tick() drives the eye LEDs through a hue rotation
// at ~30 Hz (gated by COMMS_OUT_LINE_BYTES-style internal cadence).
void update_led_set_active(bool active);

bool update_led_is_active();

// Call from main loop. No-ops when not active.
void update_led_tick(unsigned long now_ms);
