#pragma once
// HTTPClient stub for mock build — OTA is not supported in mock mode.
#include <string>
#include "WiFi.h"

#define HTTP_CODE_OK 200

class HTTPClient {
public:
    void begin(const std::string&) {}
    int GET() { return -1; }
    int getSize() { return -1; }
    WiFiClient* getStreamPtr() { return nullptr; }
    bool connected() { return false; }
    void end() {}
};
