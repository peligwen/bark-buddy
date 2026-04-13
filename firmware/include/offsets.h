#pragma once
#include <stdint.h>

// Servo trim offsets — stored in NVS, applied to IK-computed pulse widths
// before writing to hardware. Range: ±500us. Clamped to [500, 2500] when applied.

void offsets_init();                                        // load from NVS
int16_t offset_get(uint8_t servo_idx);                      // get offset for one servo (0-7)
void offset_set(uint8_t servo_idx, int16_t offset_us);      // set in RAM (not persisted)
void offsets_save();                                        // persist all to NVS
void offsets_reset();                                       // zero all in RAM and NVS
uint16_t apply_offset(uint8_t servo_idx, uint16_t raw_us);  // raw_us + offset, clamped
