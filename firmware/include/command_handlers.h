// firmware/include/command_handlers.h
#pragma once
#include <ArduinoJson.h>
#include "protocol.h"   // MsgSource

// Call once from setup() — initialises handler-owned state.
void handlers_init();

// Dispatch a received JSON message to the appropriate handler.
// Called from process_rx() in main.cpp. The source is forwarded to handlers
// that need it (currently only handle_cmd_ota_update gates auth on it).
void handle_message(const JsonDocument& doc, MsgSource source);

// Broadcast current servo pin mapping to the connected client.
// Call from setup() / send_boot_to_new_client() so the host always
// receives the current mapping on (re-)connect.
void broadcast_servo_pins();
