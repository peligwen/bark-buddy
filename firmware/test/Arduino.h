// Stub Arduino.h for host-side tests. Real stubs are in mock_arduino.h.
// This file exists so that #include <Arduino.h> in firmware sources resolves
// when compiled with -I. (test directory) via the Makefile.
#pragma once
#include "mock_arduino.h"
