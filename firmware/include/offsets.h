#pragma once
#include <stdint.h>

// Servo trim offsets — stored in NVS namespace "mechdog", applied to IK-computed
// pulse widths before writing to hardware. Range: ±500us. Clamped to [500, 2500].
// Keys: "offset_s1".."offset_s8" (matches Hiwonder stock NVS so factory cal applies).
//
// Also stores yaw trim (key "move_angle") in the same namespace.

void offsets_init();                                        // load from NVS (migrates old ns if needed)
int16_t offset_get(uint8_t servo_idx);                      // get offset for one servo (0-7)
void offset_set(uint8_t servo_idx, int16_t offset_us);      // set in RAM (not persisted)
void offsets_save();                                        // persist all to NVS
void offsets_reset();                                       // zero all in RAM and NVS
uint16_t apply_offset(uint8_t servo_idx, uint16_t raw_us);  // raw_us + offset, clamped

// Yaw trim — persistent left/right stride bias for drift correction.
// Stored in "mechdog"/"move_angle" (float). Range [-0.7, 0.7].
float yaw_trim_load();           // read from NVS (or 0.0 if absent)
void  yaw_trim_save(float mul);  // persist to NVS
