#pragma once
#include <cstdlib>
#include <cstdint>
struct ESPClass {
    uint32_t getChipId() { return 0xDEADBEEFu; }
    void restart() { exit(0); }
    uint32_t getFreeHeap() { return 200000; }
};
extern ESPClass ESP;
