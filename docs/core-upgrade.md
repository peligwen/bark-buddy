# arduino-esp32 Core Upgrade Checklist

Short runnable checklist for future core version bumps.

## Before you start
- [ ] Run `bark test` on current main to establish a green baseline
- [ ] Note the current platform version in `firmware/platformio.ini`
- [ ] Check arduino-esp32 release notes for breaking API changes

## Step 1 — Pin the new platform version
- [ ] Find the latest stable pioarduino/platform-espressif32 release: https://github.com/pioarduino/platform-espressif32/releases
- [ ] Update `firmware/platformio.ini` `platform =` line with the ZIP URL
- [ ] Update the comment with the new core version and date
- [ ] Update `PIO_PLATFORM_VER` build flag to match
- [ ] Update `ESP_ARDUINO_VERSION_*` stubs in `firmware/test/mock_arduino.h`

## Step 2 — Audit breaking API changes
Run: `git grep -E 'ledc|Wire|Serial|analogRead|mbedtls|WiFiClient|esp_timer|freertos' firmware/src/ firmware/include/`

Known breaking changes by version:

**2.x → 3.x (discovered 2026-04-20):**
- LEDC: `ledcSetup+ledcAttachPin+ledcWrite(ch)` → `ledcAttach(pin)+ledcWrite(pin)+ledcDetach(pin)`
- mbedtls: `mbedtls_sha256_starts_ret/update_ret/finish_ret` → `mbedtls_sha256_starts/update/finish` (ESP-IDF 5.x dropped the `_ret` suffix)
- WiFi: `WiFiClient` no longer transitively included — add explicit `#include <WiFi.h>` in any TU that uses it
- WiFi: `WiFiServer::available()` deprecated → `WiFiServer::accept()`
- analogRead: default ADC attenuation may shift — verify battery voltage against multimeter after flash
- Wire: master-only usage unchanged; no slave-mode API on this project

## Step 3 — Make changes atomically
- [ ] All LEDC call sites: `servos.cpp`, `buzzer.cpp`, `command_handlers.cpp` (probe handler)
- [ ] Mock: `firmware/test/mock_arduino.h` — sync with whichever API is now canonical
- [ ] Mock shims: check `firmware/mock/WiFi.h`, `firmware/mock/mbedtls/sha256.h` — any renamed functions need aliases in the mock too so both host and device builds stay green
- [ ] Test stubs: any file that declares `servo_write_us`, `aux_servo_write_us`, etc.
- [ ] Run `bark test` — must pass before proceeding

## Step 4 — Delete dead constants
- [ ] Any channel-number constants made obsolete by the new API
- [ ] Stale comments referencing the old API
- [ ] `git grep 'LEDC_CH\|ledcSetup\|ledcAttachPin\|ledcDetachPin' firmware/` should return empty

## Step 5 — Hardware verification
- [ ] `bark flash` — firmware uploads cleanly
- [ ] `python scripts/servo_bringup.py` — all 8 servos move, all acks show `written:true`
- [ ] Battery ADC sanity: compare `analogRead(BATTERY_ADC_PIN)` to multimeter reading (±2% tolerance)
  - If out of range: add `analogSetPinAttenuation(BATTERY_ADC_PIN, ADC_11db)` to battery init
- [ ] Buzzer: send `cmd_buzzer`, confirm tone, verify no servo movement during tone
- [ ] `bark ping` (or equivalent) — confirm `arduino_esp32_core` and `pio_platform` fields are correct
