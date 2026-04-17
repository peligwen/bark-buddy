// firmware/src/gpio_aux.cpp
// Stateless GPIO helpers for expansion pins (J5 header: IO32/IO33, UART0: TX/RX).
// All functions check the allowlist before calling Arduino GPIO APIs.
// In MOCK_FIRMWARE builds all reads return 0 and writes are no-ops.

#include "gpio_aux.h"

#ifndef MOCK_FIRMWARE
#include <Arduino.h>
#endif

bool gpio_aux_allowlisted(uint8_t pin) {
    for (uint8_t i = 0; i < AUX_GPIO_COUNT; i++) {
        if (AUX_GPIO_PINS[i] == pin) return true;
    }
    return false;
}

void gpio_aux_set_mode(uint8_t pin, GpioMode mode) {
    if (!gpio_aux_allowlisted(pin)) return;
#ifndef MOCK_FIRMWARE
    switch (mode) {
        case GpioMode::FLOATING:    pinMode(pin, INPUT);         break;
        case GpioMode::PULLUP:      pinMode(pin, INPUT_PULLUP);  break;
        case GpioMode::PULLDOWN:    pinMode(pin, INPUT_PULLDOWN);break;
        case GpioMode::GPIO_OUTPUT: pinMode(pin, OUTPUT);        break;
    }
#else
    (void)mode;
#endif
}

int gpio_aux_read_digital(uint8_t pin) {
    if (!gpio_aux_allowlisted(pin)) return -1;
#ifndef MOCK_FIRMWARE
    return digitalRead(pin);
#else
    return 0;
#endif
}

int gpio_aux_read_analog(uint8_t pin) {
    if (!gpio_aux_allowlisted(pin)) return -1;
#ifndef MOCK_FIRMWARE
    return analogRead(pin);
#else
    return 0;
#endif
}

void gpio_aux_write(uint8_t pin, uint8_t val) {
    if (!gpio_aux_allowlisted(pin)) return;
#ifndef MOCK_FIRMWARE
    digitalWrite(pin, val ? HIGH : LOW);
#else
    (void)val;
#endif
}
