#pragma once
// OTA Update stub for mock build — not supported.
#include <cstddef>
#include <cstdint>

#define UPDATE_SIZE_UNKNOWN 0xFFFFFFFF

class UpdateClass {
public:
    bool begin(size_t = 0) { return false; }
    size_t write(const uint8_t*, size_t) { return 0; }
    bool end(bool = false) { return false; }
    bool isFinished() { return false; }
    void abort() {}
    bool hasError() { return true; }
    const char* errorString() { return "mock-skipped"; }
};
extern UpdateClass Update;
