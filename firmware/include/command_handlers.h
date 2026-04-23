// firmware/include/command_handlers.h
#pragma once
#include <ArduinoJson.h>

// Call once from setup() — initialises handler-owned state.
void handlers_init();

// Dispatch a received JSON message to the appropriate handler.
// Called from process_rx() in main.cpp.
void handle_message(const JsonDocument& doc);

// Broadcast current servo pin mapping to the connected client.
// Call from setup() / send_boot_to_new_client() so the host always
// receives the current mapping on (re-)connect.
void broadcast_servo_pins();

// Set before processing each transport's input so handle_cmd_ota_update
// knows whether to enforce signature verification.
void set_msg_source_serial(bool is_serial);
