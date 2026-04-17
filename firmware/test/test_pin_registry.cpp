// test_pin_registry.cpp — Unit tests for pin_registry.
//
// Compiled twice:
//   test_pin_registry      — AUX_SERVOS_ENABLED=0 (default)
//   test_pin_registry_aux  — AUX_SERVOS_ENABLED=1, TEST_AUX_ENABLED defined

#include "mock_arduino.h"
#include "../include/pin_registry.h"
#include "../src/pin_registry.cpp"

#include <cstdio>
#include <cstdlib>

static int g_pass = 0;
static int g_fail = 0;

static void check(bool cond, const char* label) {
    if (cond) {
        printf("  PASS  %s\n", label);
        g_pass++;
    } else {
        printf("  FAIL  %s\n", label);
        g_fail++;
    }
}

// ------------------------------------------------------------------ //
// Helpers
// ------------------------------------------------------------------ //
static bool reserved(uint8_t pin) {
    return pin_is_reserved(pin, nullptr);
}

static bool reserved_with_reason(uint8_t pin, const char* expected_reason) {
    const char* reason = nullptr;
    bool r = pin_is_reserved(pin, &reason);
    if (!r) return false;
    if (!reason) return false;
    return (strcmp(reason, expected_reason) == 0);
}

// ------------------------------------------------------------------ //
// Individual subsystem tests
// ------------------------------------------------------------------ //
static void test_buzzer_pin_reserved() {
    printf("\nTest: test_buzzer_pin_reserved\n");
    check(reserved(21),                             "pin 21 is reserved");
    check(reserved_with_reason(21, "buzzer"),       "pin 21 reason is 'buzzer'");
}

static void test_led_pin_reserved() {
    printf("\nTest: test_led_pin_reserved\n");
    check(reserved(18),                             "pin 18 is reserved");
    check(reserved_with_reason(18, "led"),          "pin 18 reason is 'led'");
}

static void test_button_pin_reserved() {
    printf("\nTest: test_button_pin_reserved\n");
    check(reserved(5),                              "pin 5 is reserved");
    check(reserved_with_reason(5, "button"),        "pin 5 reason is 'button'");
}

static void test_i2c1_reserved() {
    printf("\nTest: test_i2c1_reserved\n");
    check(reserved(22),                             "pin 22 (I2C1 SDA) is reserved");
    check(reserved_with_reason(22, "i2c1"),         "pin 22 reason is 'i2c1'");
    check(reserved(23),                             "pin 23 (I2C1 SCL) is reserved");
    check(reserved_with_reason(23, "i2c1"),         "pin 23 reason is 'i2c1'");
}

static void test_i2c2_reserved() {
    printf("\nTest: test_i2c2_reserved\n");
    check(reserved(19),                             "pin 19 (I2C2 SDA) is reserved");
    check(reserved_with_reason(19, "i2c2"),         "pin 19 reason is 'i2c2'");
    check(reserved(13),                             "pin 13 (I2C2 SCL) is reserved");
    check(reserved_with_reason(13, "i2c2"),         "pin 13 reason is 'i2c2'");
}

static void test_battery_reserved() {
    printf("\nTest: test_battery_reserved\n");
    check(reserved(34),                             "pin 34 (battery ADC) is reserved");
    check(reserved_with_reason(34, "battery"),      "pin 34 reason is 'battery'");
}

static void test_imu_int_reserved() {
    printf("\nTest: test_imu_int_reserved\n");
    check(reserved(35),                             "pin 35 (IMU INT) is reserved");
    check(reserved_with_reason(35, "imu_int"),      "pin 35 reason is 'imu_int'");
}

static void test_adc_only_reserved() {
    printf("\nTest: test_adc_only_reserved\n");
    const uint8_t adc_only[] = {36, 37, 38, 39};
    for (uint8_t pin : adc_only) {
        char label[64];
        snprintf(label, sizeof(label), "pin %d is reserved", (int)pin);
        check(reserved(pin), label);
        snprintf(label, sizeof(label), "pin %d reason is 'adc_only'", (int)pin);
        check(reserved_with_reason(pin, "adc_only"), label);
    }
}

static void test_gpio_expansion_not_reserved() {
    printf("\nTest: test_gpio_expansion_not_reserved\n");
    const uint8_t expansion[] = {32, 33, 1, 3};
    for (uint8_t pin : expansion) {
        char label[64];
        snprintf(label, sizeof(label), "pin %d (gpio_aux expansion) is NOT reserved", (int)pin);
        check(!reserved(pin), label);
    }
}

static void test_servo_pins_not_in_registry() {
    printf("\nTest: test_servo_pins_not_in_registry\n");
    // SERVO_PINS = {25, 26, 27, 14, 16, 17, 4, 2}
    // Servo blocking is conditional on engage state — not in pin_registry.
    const uint8_t servo_pins[] = {25, 26, 27, 14, 16, 17, 4, 2};
    for (uint8_t pin : servo_pins) {
        char label[64];
        snprintf(label, sizeof(label), "servo pin %d is NOT in registry (engage-conditional)", (int)pin);
        check(!reserved(pin), label);
    }
}

#if !defined(TEST_AUX_ENABLED)
static void test_aux_servo_not_reserved_when_disabled() {
    printf("\nTest: test_aux_servo_not_reserved_when_disabled\n");
    // AUX_SERVOS_ENABLED=0 at compile time: pins 15, 0, 12 must not be reserved.
    const uint8_t aux_pins[] = {15, 0, 12};
    for (uint8_t pin : aux_pins) {
        char label[64];
        snprintf(label, sizeof(label), "aux servo pin %d is NOT reserved (AUX disabled)", (int)pin);
        check(!reserved(pin), label);
    }
}
#endif

#if defined(TEST_AUX_ENABLED)
static void test_aux_servo_reserved_when_enabled() {
    printf("\nTest: test_aux_servo_reserved_when_enabled\n");
    // AUX_SERVOS_ENABLED=1 at compile time: pins 15, 0, 12 must be reserved as "aux_servo".
    const uint8_t aux_pins[] = {15, 0, 12};
    for (uint8_t pin : aux_pins) {
        char label[64];
        snprintf(label, sizeof(label), "aux servo pin %d IS reserved (AUX enabled)", (int)pin);
        check(reserved(pin), label);
        snprintf(label, sizeof(label), "aux servo pin %d reason is 'aux_servo'", (int)pin);
        check(reserved_with_reason(pin, "aux_servo"), label);
    }
}
#endif

// ------------------------------------------------------------------ //
// Main
// ------------------------------------------------------------------ //
int main() {
#if defined(TEST_AUX_ENABLED)
    printf("=== test_pin_registry_aux (AUX_SERVOS_ENABLED=1) ===\n");
#else
    printf("=== test_pin_registry (AUX_SERVOS_ENABLED=0) ===\n");
#endif

    test_buzzer_pin_reserved();
    test_led_pin_reserved();
    test_button_pin_reserved();
    test_i2c1_reserved();
    test_i2c2_reserved();
    test_battery_reserved();
    test_imu_int_reserved();
    test_adc_only_reserved();
    test_gpio_expansion_not_reserved();
    test_servo_pins_not_in_registry();

#if defined(TEST_AUX_ENABLED)
    test_aux_servo_reserved_when_enabled();
#else
    test_aux_servo_not_reserved_when_disabled();
#endif

    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail > 0 ? 1 : 0;
}
