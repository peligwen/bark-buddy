// Mock button — returns NONE always; replaces firmware/src/button.cpp at link time.
#include "button.h"

void button_init() {}
ButtonEvent button_update(uint32_t) { return ButtonEvent::NONE; }
uint32_t button_held_ms() { return 0; }
