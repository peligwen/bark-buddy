#include "servos.h"
#include "config.h"
#include <Arduino.h>
#include "driver/gpio.h"
#include "soc/gpio_struct.h"

// Software PWM using a FreeRTOS task — works on ANY GPIO.
// Uses direct GPIO register writes to toggle pins, bypassing the GPIO
// driver which blocks SPI flash pins (7, 8).
//
// The task runs at high priority and handles the full 20ms servo period:
// 1. Set all servo pins HIGH
// 2. delayMicroseconds until each servo's pulse width, then set that pin LOW
// 3. Sleep until the next period

#define SERVO_PERIOD_US 20000

static uint16_t current_us[8] = {0};
static volatile uint16_t target_us[8] = {0};  // written by main, read by task
static bool attached = false;
static TaskHandle_t servo_task_handle = NULL;
static uint32_t servo_pin_mask[8] = {0};

// Sort servos by pulse width for efficient sequential pin-lowering
struct ServoOrder {
    uint8_t index;
    uint16_t us;
};

static void servo_pwm_task(void* arg) {
    ServoOrder order[8];

    while (attached) {
        // Snapshot target pulse widths and sort by ascending duration
        for (int i = 0; i < 8; i++) {
            order[i].index = i;
            order[i].us = target_us[i];
        }
        // Simple insertion sort (8 elements)
        for (int i = 1; i < 8; i++) {
            ServoOrder key = order[i];
            int j = i - 1;
            while (j >= 0 && order[j].us > key.us) {
                order[j + 1] = order[j];
                j--;
            }
            order[j + 1] = key;
        }

        // Set all active servo pins HIGH
        uint32_t all_mask = 0;
        for (int i = 0; i < 8; i++) {
            if (order[i].us > 0) all_mask |= servo_pin_mask[order[i].index];
        }
        if (all_mask) GPIO.out_w1ts = all_mask;

        // Walk through sorted list, delay until each threshold, set pin LOW
        uint16_t elapsed = 0;
        for (int i = 0; i < 8; i++) {
            if (order[i].us == 0) continue;
            uint16_t wait = order[i].us - elapsed;
            if (wait > 0) delayMicroseconds(wait);
            elapsed = order[i].us;
            GPIO.out_w1tc = servo_pin_mask[order[i].index];
        }

        // Wait for remainder of 20ms period
        uint16_t remaining = SERVO_PERIOD_US - elapsed;
        if (remaining > 1000) {
            vTaskDelay(pdMS_TO_TICKS(remaining / 1000));
        } else if (remaining > 0) {
            delayMicroseconds(remaining);
        }
    }
    vTaskDelete(NULL);
}

static uint16_t clamp_us(uint16_t us) {
    if (us < SERVO_MIN_US) return SERVO_MIN_US;
    if (us > SERVO_MAX_US) return SERVO_MAX_US;
    return us;
}

bool servos_init() {
#if !PINS_VERIFIED
    Serial.println("{\"type\":\"error\",\"msg\":\"PINS_VERIFIED=0, servos disabled\"}");
    return false;
#else
    // Configure all servo GPIOs as outputs
    for (int i = 0; i < 8; i++) {
        uint8_t pin = SERVO_PINS[i];
        servo_pin_mask[i] = (1UL << pin);
        if (pin == 7 || pin == 8) {
            // Direct register write — bypass GPIO driver SPI flash protection
            GPIO.enable_w1ts = (1UL << pin);
            PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[pin], PIN_FUNC_GPIO);
        } else {
            gpio_pad_select_gpio(pin);
            gpio_set_direction((gpio_num_t)pin, GPIO_MODE_OUTPUT);
        }
        GPIO.out_w1tc = (1UL << pin);
        current_us[i] = SERVO_CENTER_US;
        target_us[i] = SERVO_CENTER_US;
    }
    attached = true;

    // Start servo PWM task on core 1 (core 0 handles WiFi/BT)
    xTaskCreatePinnedToCore(servo_pwm_task, "servo_pwm", 2048, NULL,
                            configMAX_PRIORITIES - 1, &servo_task_handle, 1);

    // Soft-start: ramp from center to standing pose
    for (int step = 0; step <= SOFTSTART_STEPS; step++) {
        float t = (float)step / (float)SOFTSTART_STEPS;
        for (int i = 0; i < 8; i++) {
            uint16_t target = STANDING_POSE[i];
            uint16_t pos = SERVO_CENTER_US + (uint16_t)((float)(target - SERVO_CENTER_US) * t);
            pos = clamp_us(pos);
            current_us[i] = pos;
            target_us[i] = pos;
        }
        delay(SOFTSTART_DURATION_MS / SOFTSTART_STEPS);
    }

    return true;
#endif
}

void servo_write_us(uint8_t index, uint16_t pulse_us) {
    if (!attached || index >= 8) return;
    pulse_us = clamp_us(pulse_us);
    current_us[index] = pulse_us;
    target_us[index] = pulse_us;
}

void servo_write_angle(uint8_t index, float angle_deg) {
    if (angle_deg < 0) angle_deg = 0;
    if (angle_deg > 180) angle_deg = 180;
    uint16_t us = SERVO_MIN_US + (uint16_t)(angle_deg / 180.0f * (SERVO_MAX_US - SERVO_MIN_US));
    servo_write_us(index, us);
}

uint16_t servo_read_us(uint8_t index) {
    if (index >= 8) return 0;
    return current_us[index];
}

void servos_detach_all() {
    if (!attached) return;
    attached = false;
    if (servo_task_handle) {
        vTaskDelay(pdMS_TO_TICKS(50));
        servo_task_handle = NULL;
    }
    for (int i = 0; i < 8; i++) {
        GPIO.out_w1tc = servo_pin_mask[i];
        target_us[i] = 0;
        current_us[i] = 0;
    }
}

bool servos_active() {
    return attached;
}
