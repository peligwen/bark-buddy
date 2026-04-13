// firmware/include/comms.h
#pragma once
#include <ArduinoJson.h>

// Defined in main.cpp. Writes NDJSON to serial and (if connected) TCP client.
void send_json(const JsonDocument& doc);

// Convenience wrapper: sends {"type":"ack","ref_type":ref_type,"ok":ok[,"error":error]}
void send_ack(const char* ref_type, bool ok, const char* error = nullptr);
