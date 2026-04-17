#pragma once
#include <stdint.h>

// Returns true and sets *reason_out if pin is owned by a firmware subsystem.
// Returns false if pin is safe for external use (probe, gpio_aux, etc.).
// reason_out may be nullptr if the caller doesn't need the reason string.
// Possible reasons: "aux_servo", "buzzer", "led", "button",
//                   "i2c1", "i2c2", "battery", "imu_int", "adc_only"
bool pin_is_reserved(uint8_t pin, const char** reason_out = nullptr);
