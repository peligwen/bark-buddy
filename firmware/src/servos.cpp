#include "servos.h"
#include "config.h"
#include <Arduino.h>

// Hardware PWM via ESP32 LEDC peripheral.
// Each servo gets its own LEDC channel: servo index i → LEDC channel i.
// Uses old-style API (ledcSetup/ledcAttachPin/ledcWrite(channel,...)) compatible
// with espressif32 Arduino core 2.x.
// Zero CPU usage during pulse generation — no FreeRTOS task, no busy-wait.
// 14-bit resolution at 50Hz gives ~1.22us per tick (adequate for all servos).

static uint16_t current_us[8] = {0};
static bool attached = false;

// Frail mode state
static bool frail_mode = false;
static uint16_t frail_target_us[8] = {0};  // slew-rate filtered targets
static unsigned long duty_start = 0;       // when continuous movement started
static bool in_cooldown = false;
static unsigned long cooldown_start = 0;

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

bool servos_attach_at(const uint16_t pose[8]) {
#if !PINS_VERIFIED
    Serial.println("{\"type\":\"error\",\"msg\":\"PINS_VERIFIED=0, servos disabled\"}");
    return false;
#else
    if (attached) return false;
    for (int i = 0; i < 8; i++) {
        ledcSetup(i, SERVO_FREQ_HZ, LEDC_RESOLUTION);
        ledcAttachPin(SERVO_PINS[i], i);
    }
    attached = true;
    for (int i = 0; i < 8; i++) {
        uint16_t pos = clamp_us(pose[i]);
        current_us[i] = pos;
        ledcWrite(SERVO_PINS[i], us_to_duty(pos));
    }
    return true;
#endif
}

void servos_ramp_to(const uint16_t target[8], uint16_t duration_ms, uint8_t steps) {
    if (!attached) return;
    uint16_t start_us[8];
    for (int i = 0; i < 8; i++) {
        start_us[i] = current_us[i] > 0 ? current_us[i] : target[i];
    }
    for (int step = 0; step <= steps; step++) {
        float t = (float)step / (float)steps;
        for (int i = 0; i < 8; i++) {
            int16_t s = (int16_t)start_us[i];
            int16_t e = (int16_t)target[i];
            uint16_t pos = (uint16_t)(s + (int16_t)((float)(e - s) * t));
            pos = clamp_us(pos);
            current_us[i] = pos;
            ledcWrite(SERVO_PINS[i], us_to_duty(pos));
        }
        delay(duration_ms / steps);
    }
}

bool servos_shutdown_to_rest() {
    if (!attached) return false;
    servos_ramp_to(REST_POSE, SHUTDOWN_RAMP_MS, SHUTDOWN_RAMP_STEPS);
    delay(REST_SETTLE_MS);
    servos_detach_all();
    return true;
}

bool servos_init() {
    if (!servos_attach_at(REST_POSE)) return false;
    delay(BOOT_SETTLE_MS);
    servos_ramp_to(STANDING_POSE, SOFTSTART_DURATION_MS, SOFTSTART_STEPS);
    return true;
}

void servo_write_us(uint8_t index, uint16_t pulse_us) {
    if (!attached || index >= 8) return;
    if (in_cooldown) return;
    pulse_us = clamp_us(pulse_us);

    if (frail_mode) {
        // Range clamp: limit deviation from standing pose
        int16_t standing = (int16_t)STANDING_POSE[index];
        int16_t offset = (int16_t)pulse_us - standing;
        if (offset > FRAIL_MAX_OFFSET_US) offset = FRAIL_MAX_OFFSET_US;
        if (offset < -FRAIL_MAX_OFFSET_US) offset = -FRAIL_MAX_OFFSET_US;
        pulse_us = (uint16_t)(standing + offset);

        // Slew rate limit: max change per write
        int16_t current = (int16_t)frail_target_us[index];
        if (current > 0) {
            int16_t delta = (int16_t)pulse_us - current;
            if (delta > FRAIL_SLEW_RATE_US) delta = FRAIL_SLEW_RATE_US;
            if (delta < -FRAIL_SLEW_RATE_US) delta = -FRAIL_SLEW_RATE_US;
            pulse_us = (uint16_t)(current + delta);
        }
        frail_target_us[index] = pulse_us;
    }

    current_us[index] = pulse_us;
    ledcWrite(SERVO_PINS[index], us_to_duty(pulse_us));
}


uint16_t servo_read_us(uint8_t index) {
    if (index >= 8) return 0;
    return current_us[index];
}

void servos_detach_all() {
    if (!attached) return;
    attached = false;
    for (int i = 0; i < 8; i++) {
        ledcDetach(SERVO_PINS[i]);
        current_us[i] = 0;
    }
}

bool servos_active() {
    return attached;
}

bool servos_shutdown_to_lying_down() {
    if (!attached) return false;

    // Capture current positions as ramp start
    uint16_t start_us[8];
    for (int i = 0; i < 8; i++) {
        start_us[i] = current_us[i] > 0 ? current_us[i] : STANDING_POSE[i];
    }

    // Ramp to lying-down over SHUTDOWN_RAMP_MS
    for (int step = 0; step <= SHUTDOWN_RAMP_STEPS; step++) {
        float t = (float)step / (float)SHUTDOWN_RAMP_STEPS;
        for (int i = 0; i < 8; i++) {
            int16_t s    = (int16_t)start_us[i];
            int16_t e    = (int16_t)LYING_DOWN_POSE[i];
            uint16_t pos = (uint16_t)(s + (int16_t)((float)(e - s) * t));
            pos = clamp_us(pos);
            current_us[i] = pos;
            ledcWrite(SERVO_PINS[i], us_to_duty(pos));
        }
        delay(SHUTDOWN_RAMP_MS / SHUTDOWN_RAMP_STEPS);
    }

    delay(SHUTDOWN_SETTLE_MS);
    servos_detach_all();
    return true;
}

// --- Frail mode ---

void servos_set_frail(bool enabled) {
    frail_mode = enabled;
    in_cooldown = false;
    duty_start = 0;
    if (enabled) {
        for (int i = 0; i < 8; i++) {
            frail_target_us[i] = current_us[i] > 0 ? current_us[i] : STANDING_POSE[i];
        }
    }
}

bool servos_frail() {
    return frail_mode;
}

bool servos_update_duty(unsigned long now_ms) {
    if (!frail_mode || !attached) {
        in_cooldown = false;
        return false;
    }

    if (in_cooldown) {
        if (now_ms - cooldown_start >= FRAIL_COOLDOWN_MS) {
            in_cooldown = false;
            duty_start = now_ms;
        }
        return in_cooldown;
    }

    bool any_active = false;
    for (int i = 0; i < 8; i++) {
        if (current_us[i] > 0) {
            int16_t diff = (int16_t)current_us[i] - (int16_t)STANDING_POSE[i];
            if (diff < 0) diff = -diff;
            if (diff > 5) { any_active = true; break; }
        }
    }

    if (any_active) {
        if (duty_start == 0) duty_start = now_ms;
        if (now_ms - duty_start >= FRAIL_DUTY_ON_MS) {
            for (int i = 0; i < 8; i++) {
                uint16_t standing = STANDING_POSE[i];
                current_us[i] = standing;
                frail_target_us[i] = standing;
                ledcWrite(SERVO_PINS[i], us_to_duty(standing));
            }
            in_cooldown = true;
            cooldown_start = now_ms;
        }
    } else {
        duty_start = 0;
    }

    return in_cooldown;
}
