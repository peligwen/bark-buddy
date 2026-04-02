#pragma once
#include <Wire.h>

bool sonar_init(TwoWire& wire);
uint16_t sonar_read_mm();
void sonar_set_rgb_mode(uint8_t mode);  // 0=manual, 1=breathing, 2=cycle
void sonar_set_rgb(uint8_t led, uint8_t r, uint8_t g, uint8_t b);  // led: 1 or 2
