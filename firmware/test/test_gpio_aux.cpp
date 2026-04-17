// test_gpio_aux.cpp — Unit tests for the gpio_aux driver.
//
// Strategy: compile gpio_aux.cpp with MOCK_FIRMWARE=1 defined (passed via Makefile)
// so all Arduino calls are guarded out. Tests run on the host with no hardware.

#include "mock_arduino.h"
#include "../include/gpio_aux.h"

// Pull in the real implementation with MOCK_FIRMWARE active (defined via -D in Makefile).
// Arduino calls are #ifndef MOCK_FIRMWARE guarded in gpio_aux.cpp, so they compile away.
#include "../src/gpio_aux.cpp"

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
// Test: allowlist — known good pins
// ------------------------------------------------------------------ //
static void test_allowlisted_pins() {
    printf("\nTest: allowlisted_pins\n");
    check(gpio_aux_allowlisted(32), "pin 32 is allowlisted");
    check(gpio_aux_allowlisted(33), "pin 33 is allowlisted");
    check(gpio_aux_allowlisted(1),  "pin 1 is allowlisted");
    check(gpio_aux_allowlisted(3),  "pin 3 is allowlisted");
}

// ------------------------------------------------------------------ //
// Test: allowlist — pins that must NOT be allowed
// ------------------------------------------------------------------ //
static void test_not_allowlisted_pins() {
    printf("\nTest: not_allowlisted_pins\n");
    const uint8_t bad[] = {0, 2, 4, 5, 13, 14, 16, 17, 18, 19, 21, 22, 23, 25, 26, 27, 34, 35};
    for (uint8_t pin : bad) {
        char label[64];
        snprintf(label, sizeof(label), "pin %d is NOT allowlisted", (int)pin);
        check(!gpio_aux_allowlisted(pin), label);
    }
}

// ------------------------------------------------------------------ //
// Test: read_digital returns 0 in mock
// ------------------------------------------------------------------ //
static void test_read_digital_mock() {
    printf("\nTest: read_digital_mock\n");
    check(gpio_aux_read_digital(32) == 0, "read_digital(32) == 0 in mock");
    check(gpio_aux_read_digital(33) == 0, "read_digital(33) == 0 in mock");
    check(gpio_aux_read_digital(1)  == 0, "read_digital(1) == 0 in mock");
    check(gpio_aux_read_digital(3)  == 0, "read_digital(3) == 0 in mock");
}

// ------------------------------------------------------------------ //
// Test: read_digital returns -1 for non-allowlisted pin
// ------------------------------------------------------------------ //
static void test_read_digital_invalid() {
    printf("\nTest: read_digital_invalid\n");
    check(gpio_aux_read_digital(0)  == -1, "read_digital(0) == -1 (not allowed)");
    check(gpio_aux_read_digital(22) == -1, "read_digital(22) == -1 (not allowed)");
}

// ------------------------------------------------------------------ //
// Test: read_analog returns 0 for ADC-capable pins (32, 33 only)
//       and -1 for UART pins (1, 3) which are not ADC-safe.
// ------------------------------------------------------------------ //
static void test_read_analog_mock() {
    printf("\nTest: read_analog_mock\n");
    check(gpio_aux_read_analog(32) == 0,  "read_analog(32) == 0 in mock");
    check(gpio_aux_read_analog(33) == 0,  "read_analog(33) == 0 in mock");
    check(gpio_aux_read_analog(1)  == -1, "read_analog(1) == -1 (UART pin, not ADC-safe)");
    check(gpio_aux_read_analog(3)  == -1, "read_analog(3) == -1 (UART pin, not ADC-safe)");
}

// ------------------------------------------------------------------ //
// Test: read_analog returns -1 for non-allowlisted pin
// ------------------------------------------------------------------ //
static void test_read_analog_invalid() {
    printf("\nTest: read_analog_invalid\n");
    check(gpio_aux_read_analog(0)  == -1, "read_analog(0) == -1 (not allowed)");
    check(gpio_aux_read_analog(34) == -1, "read_analog(34) == -1 (not allowed)");
}

// ------------------------------------------------------------------ //
// Test: set_mode does not crash for any allowlisted pin + mode combo
// ------------------------------------------------------------------ //
static void test_set_mode_no_crash() {
    printf("\nTest: set_mode_no_crash\n");
    const uint8_t pins[] = {32, 33, 1, 3};
    const GpioMode modes[] = {
        GpioMode::FLOATING,
        GpioMode::PULLUP,
        GpioMode::PULLDOWN,
        GpioMode::GPIO_OUTPUT,
    };
    bool ok = true;
    for (uint8_t pin : pins) {
        for (GpioMode mode : modes) {
            gpio_aux_set_mode(pin, mode);  // must not crash
        }
    }
    check(ok, "set_mode does not crash for all allowlisted pins and modes");
}

// ------------------------------------------------------------------ //
// Test: set_mode on invalid pin does not crash
// ------------------------------------------------------------------ //
static void test_set_mode_invalid_no_crash() {
    printf("\nTest: set_mode_invalid_no_crash\n");
    gpio_aux_set_mode(0,   GpioMode::GPIO_OUTPUT);  // must not crash
    gpio_aux_set_mode(22,  GpioMode::PULLUP);
    gpio_aux_set_mode(255, GpioMode::FLOATING);
    check(true, "set_mode on invalid pins does not crash");
}

// ------------------------------------------------------------------ //
// Test: write does not crash
// ------------------------------------------------------------------ //
static void test_write_no_crash() {
    printf("\nTest: write_no_crash\n");
    gpio_aux_write(32, 1);
    gpio_aux_write(33, 0);
    gpio_aux_write(1,  1);
    gpio_aux_write(3,  0);
    gpio_aux_write(0,  1);  // invalid pin — must not crash
    check(true, "gpio_aux_write does not crash");
}

// ------------------------------------------------------------------ //
// Main
// ------------------------------------------------------------------ //
int main() {
    printf("=== test_gpio_aux ===\n");

    test_allowlisted_pins();
    test_not_allowlisted_pins();
    test_read_digital_mock();
    test_read_digital_invalid();
    test_read_analog_mock();
    test_read_analog_invalid();
    test_set_mode_no_crash();
    test_set_mode_invalid_no_crash();
    test_write_no_crash();

    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail > 0 ? 1 : 0;
}
