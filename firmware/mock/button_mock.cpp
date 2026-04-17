// Mock button — returns NONE always; replaces firmware/src/button.cpp at link time.
#include "button.h"

void button_init() {}
ButtonResult button_update(uint32_t) { return ButtonResult{ButtonEvent::NONE, 0}; }
uint32_t button_held_ms() { return 0; }
