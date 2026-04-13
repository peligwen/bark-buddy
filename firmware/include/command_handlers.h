// firmware/include/command_handlers.h
#pragma once
#include <ArduinoJson.h>

// Call once from setup() — initialises handler-owned state.
void handlers_init();

// Dispatch a received JSON message to the appropriate handler.
// Called from process_rx() in main.cpp.
void handle_message(const JsonDocument& doc);

// Read-only accessors for state that main loop needs.
// State is owned here; main.cpp never writes these directly.
bool          handlers_manual_servo_mode();
bool          handlers_test_mode();
unsigned long handlers_last_test_cmd();
