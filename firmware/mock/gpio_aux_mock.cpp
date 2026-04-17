// firmware/mock/gpio_aux_mock.cpp
// Mock gpio_aux — all reads return 0, writes are no-ops, allowlist checks pass
// for the four expansion pins. Link-time replacement for gpio_aux.cpp in mock builds.

#include "../include/gpio_aux.h"
#include <cstdio>

bool gpio_aux_allowlisted(uint8_t pin) {
    for (uint8_t i = 0; i < AUX_GPIO_COUNT; i++) {
        if (AUX_GPIO_PINS[i] == pin) return true;
    }
    return false;
}

void gpio_aux_set_mode(uint8_t pin, GpioMode mode) {
    (void)pin; (void)mode;
}

int gpio_aux_read_digital(uint8_t pin) {
    (void)pin;
    return 0;
}

int gpio_aux_read_analog(uint8_t pin) {
    if (!gpio_aux_allowlisted(pin)) return -1;
    (void)pin;
    return 0;
}

void gpio_aux_write(uint8_t pin, uint8_t val) {
    (void)pin; (void)val;
}
