// firmware/include/command_handlers.h
#pragma once
#include <ArduinoJson.h>

// Call once from setup() — initialises handler-owned state.
void handlers_init();

// Dispatch a received JSON message to the appropriate handler.
// Called from process_rx() in main.cpp.
void handle_message(const JsonDocument& doc);
