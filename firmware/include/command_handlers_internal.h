// firmware/include/command_handlers_internal.h
//
// Subhandler entry points exposed to the dispatch table in command_handlers.cpp.
// These live in sibling translation units (command_handlers_ota.cpp,
// command_handlers_probe.cpp) so command_handlers.cpp can stay focused on
// motion / IO commands.
//
// Not part of any public surface — never include from anything other than
// command_handlers.cpp and the matching subhandler .cpp file.
#pragma once

#include <ArduinoJson.h>
#include "protocol.h"

// Defined in command_handlers_ota.cpp
void handle_cmd_ota_request_nonce(const JsonDocument& doc, MsgSource source);
void handle_cmd_ota_update(const JsonDocument& doc, MsgSource source);

// Defined in command_handlers_probe.cpp
void handle_cmd_probe_pin(const JsonDocument& doc);
