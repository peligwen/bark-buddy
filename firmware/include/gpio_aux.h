#pragma once
#include <stdint.h>

// Allowlisted expansion GPIO pins. Only these may be used with gpio_aux_*.
// {32, 33} = J5 header; {1, 3} = UART0 header (TX/RX, USB-serial fallback risk).
static constexpr uint8_t AUX_GPIO_PINS[] = {32, 33, 1, 3};
static constexpr uint8_t AUX_GPIO_COUNT  = 4;

enum class GpioMode : uint8_t { FLOATING, GPIO_PULLUP, GPIO_PULLDOWN, GPIO_OUTPUT };
enum class GpioOp   : uint8_t { MODE, WRITE, READ, GPIO_ANALOG, SUBSCRIBE, UNSUBSCRIBE };

bool gpio_aux_allowlisted(uint8_t pin);
void gpio_aux_set_mode(uint8_t pin, GpioMode mode);
int  gpio_aux_read_digital(uint8_t pin);
int  gpio_aux_read_analog(uint8_t pin);  // ADC, 0-4095; only valid on GPIO 32/33
void gpio_aux_write(uint8_t pin, uint8_t val);
