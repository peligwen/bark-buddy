// Stub Arduino.h for host-side tests. Real stubs are in mock_arduino.h.
// This file exists so that #include <Arduino.h> in firmware sources resolves
// when compiled with -I. (test directory) via the Makefile.
#pragma once
// Under MOCK_FIRMWARE (bark-mock binary), use real-time millis/delay.
#ifdef MOCK_FIRMWARE
#ifndef MOCK_REAL_TIME
#define MOCK_REAL_TIME 1
#endif
#endif
#include "mock_arduino.h"
