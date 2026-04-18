#pragma once

enum class BatteryLedState { Ok, Low, Critical, Cutoff };

void battery_led_init(void);
void battery_led_update_voltage(int mv, bool cutoff);
void battery_led_tick(unsigned long now_ms);
BatteryLedState battery_led_state(void);
