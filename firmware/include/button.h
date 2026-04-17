#pragma once
#include <stdint.h>

enum class ButtonEvent : uint8_t { NONE, PRESS, RELEASE, LONG_PRESS };

void button_init();
// Call from sensor_task every ~10ms. Returns edge event or NONE.
ButtonEvent button_update(uint32_t now_ms);
// Returns ms since last press start (0 if not currently pressed / never pressed).
uint32_t button_held_ms();
