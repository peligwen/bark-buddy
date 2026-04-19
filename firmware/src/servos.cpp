#include "servos.h"
#include "config.h"
#include "offsets.h"
#include "pin_registry.h"
#include <Arduino.h>

// Hardware PWM via ESP32 LEDC peripheral (old-style ledcSetup/ledcAttachPin API).
// Servo index i → LEDC channel i. Writes use ledcWrite(pin, duty) — pin-based.
// 50Hz, 14-bit resolution (~1.22us/tick). Zero CPU cost during pulse generation.
// Engage: attach at REST_POSE -> ramp to STANDING_POSE (non-blocking, millis-based).
// Disengage: ramp to REST_POSE -> detach.

uint8_t SERVO_PINS[8] = {25, 26, 27, 14, 16, 17, 15, 2};
static uint16_t current_us[8] = {0};
static bool s_engaged = false;
static bool s_ramping = false;
static bool s_battery_cutoff = false;

#if AUX_SERVOS_ENABLED
static uint16_t aux_current_us[AUX_SERVO_COUNT] = {0};
#endif

// Ramp state
static uint32_t s_ramp_start_ms = 0;
static uint32_t s_ramp_duration_ms = 0;
static uint16_t s_ramp_from[8] = {};
static uint16_t s_ramp_to_pose[8] = {};
static bool s_ramp_is_engage = false;  // true=engage, false=disengage

static uint16_t clamp_us(uint16_t us) {
    if (us < SERVO_MIN_US) return SERVO_MIN_US;
    if (us > SERVO_MAX_US) return SERVO_MAX_US;
    return us;
}

// Convert pulse width in microseconds to LEDC duty count.
// period_us = 1,000,000 / SERVO_FREQ_HZ = 20,000us
static uint32_t us_to_duty(uint16_t pulse_us) {
    return (uint32_t)((uint64_t)pulse_us * LEDC_MAX_DUTY / (1000000UL / SERVO_FREQ_HZ));
}

bool servos_engage_start() {
#if !PINS_VERIFIED
    Serial.println("{\"type\":\"error\",\"msg\":\"PINS_VERIFIED=0\"}");
    return false;
#else
    if (s_engaged || s_ramping || s_battery_cutoff) return false;
    for (int i = 0; i < 8; i++) {
        ledcSetup(i, SERVO_FREQ_HZ, LEDC_RESOLUTION);
        ledcAttachPin(SERVO_PINS[i], i);
        uint16_t pos = clamp_us(REST_POSE[i]);
        current_us[i] = pos;
        ledcWrite(SERVO_PINS[i], us_to_duty(pos));
    }
    s_engaged = true;
#if AUX_SERVOS_ENABLED
    aux_servo_init();
#endif
    for (int i = 0; i < 8; i++) {
        s_ramp_from[i] = REST_POSE[i];
        s_ramp_to_pose[i] = STANDING_POSE[i];
    }
    s_ramp_start_ms = millis();
    s_ramp_duration_ms = SOFTSTART_DURATION_MS;
    s_ramp_is_engage = true;
    s_ramping = true;
    return true;
#endif
}

void servos_disengage_start() {
    if (!s_engaged && !s_ramping) return;
    if (s_ramping && !s_ramp_is_engage) return;  // already disengaging — do not restart ramp
    // If mid-engage-ramp, freeze current position as start of disengage ramp
    for (int i = 0; i < 8; i++) {
        s_ramp_from[i] = (current_us[i] > 0) ? current_us[i] : STANDING_POSE[i];
        s_ramp_to_pose[i] = REST_POSE[i];
    }
    s_ramp_start_ms = millis();
    s_ramp_duration_ms = SHUTDOWN_RAMP_MS;
    s_ramp_is_engage = false;
    s_ramping = true;
}

RampResult servos_ramp_tick(uint32_t now_ms) {
    if (!s_ramping) return RampResult::NONE;
    uint32_t elapsed = now_ms - s_ramp_start_ms;
    if (elapsed >= s_ramp_duration_ms) {
        // Write final target
        for (int i = 0; i < 8; i++) {
            servo_write_us(i, s_ramp_to_pose[i]);
        }
        bool was_engage = s_ramp_is_engage;
        s_ramping = false;
        if (!was_engage) {
            // Disengage complete — detach
            servos_detach_all();
        }
        return was_engage ? RampResult::ENGAGE_COMPLETE : RampResult::DISENGAGE_COMPLETE;
    }
    float t = (float)elapsed / (float)s_ramp_duration_ms;
    for (int i = 0; i < 8; i++) {
        int16_t s = (int16_t)s_ramp_from[i];
        int16_t e = (int16_t)s_ramp_to_pose[i];
        uint16_t pos = (uint16_t)(s + (int16_t)((float)(e - s) * t));
        servo_write_us(i, pos);
    }
    return RampResult::NONE;
}

bool servos_is_ramping() {
    return s_ramping;
}

bool servos_engaged() {
    return s_engaged;
}

void servo_write_us(uint8_t index, uint16_t pulse_us) {
    if (!s_engaged || index >= 8) return;
    pulse_us = clamp_us(pulse_us);
    current_us[index] = pulse_us;
    ledcWrite(SERVO_PINS[index], us_to_duty(apply_offset(index, pulse_us)));
}

uint16_t servo_read_us(uint8_t index) {
    if (index >= 8) return 0;
    return current_us[index];
}

void servos_detach_all() {
    if (!s_engaged && !s_ramping) return;
    s_engaged = false;
    s_ramping = false;
    for (int i = 0; i < 8; i++) {
        ledcDetachPin(SERVO_PINS[i]);
        current_us[i] = 0;
    }
#if AUX_SERVOS_ENABLED
    for (int i = 0; i < AUX_SERVO_COUNT; i++) {
        ledcDetachPin(AUX_SERVO_PINS[i]);
        aux_current_us[i] = 0;
    }
#endif
}

void servos_set_battery_cutoff() {
    s_battery_cutoff = true;
    servos_detach_all();
}

bool servos_battery_cutoff() {
    return s_battery_cutoff;
}

bool servos_set_pin(uint8_t idx, uint8_t new_pin, String& err) {
    if (idx >= 8) { err = "index out of range"; return false; }
    if (new_pin > 39) { err = "pin out of range"; return false; }
    const char* reason = nullptr;
    if (pin_is_reserved(new_pin, &reason)) {
        err = String("pin reserved: ") + reason;
        return false;
    }
    int8_t swap_j = -1;
    for (uint8_t j = 0; j < 8; j++) {
        if (j != idx && SERVO_PINS[j] == new_pin) { swap_j = (int8_t)j; break; }
    }
    uint8_t old_pin = SERVO_PINS[idx];
    if (old_pin == new_pin) return true;
    if (s_engaged) {
        ledcDetachPin(old_pin);
        if (swap_j >= 0) ledcDetachPin(new_pin);
        SERVO_PINS[idx] = new_pin;
        ledcSetup(idx, SERVO_FREQ_HZ, LEDC_RESOLUTION);
        ledcAttachPin(new_pin, idx);
        servo_write_us(idx, current_us[idx] > 0 ? current_us[idx] : 1500);
        if (swap_j >= 0) {
            SERVO_PINS[swap_j] = old_pin;
            ledcSetup(swap_j, SERVO_FREQ_HZ, LEDC_RESOLUTION);
            ledcAttachPin(old_pin, swap_j);
            servo_write_us(swap_j, current_us[swap_j] > 0 ? current_us[swap_j] : 1500);
        }
    } else {
        SERVO_PINS[idx] = new_pin;
        if (swap_j >= 0) SERVO_PINS[swap_j] = old_pin;
    }
    return true;
}

// ============================================================
// Auxiliary servo ports (firmware indices 8-10)
// ============================================================
#if AUX_SERVOS_ENABLED

void aux_servo_init() {
    for (int i = 0; i < AUX_SERVO_COUNT; i++) {
        ledcSetup(AUX_SERVO_LEDC_CH[i], SERVO_FREQ_HZ, LEDC_RESOLUTION);
        ledcAttachPin(AUX_SERVO_PINS[i], AUX_SERVO_LEDC_CH[i]);
        uint16_t pos = clamp_us(1500);
        aux_current_us[i] = pos;
        ledcWrite(AUX_SERVO_PINS[i], us_to_duty(pos));
    }
}

void aux_servo_write_us(uint8_t idx, uint16_t us) {
    if (!s_engaged || idx >= AUX_SERVO_COUNT) return;
    us = clamp_us(us);
    aux_current_us[idx] = us;
    ledcWrite(AUX_SERVO_PINS[idx], us_to_duty(us));
}

uint16_t aux_servo_read_us(uint8_t idx) {
    if (idx >= AUX_SERVO_COUNT) return 0;
    return aux_current_us[idx];
}

#endif  // AUX_SERVOS_ENABLED
