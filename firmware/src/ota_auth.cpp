#include "ota_auth.h"
#include <string.h>
#include <monocypher-ed25519.h>

// OWNER_PUBKEY is provided by owner_pubkey.generated.cpp (PlatformIO builds)
// or firmware/test/test_owner_pubkey.cpp (test/mock builds).
extern const uint8_t OWNER_PUBKEY[32];

#if defined(HOST_BUILD) || defined(MOCK_FIRMWARE)
#  include "mock_arduino.h"
#else
#  include <Arduino.h>
#  include <esp_system.h>
#endif

static uint8_t  s_nonce[32]       = {};
static bool     s_nonce_valid     = false;
static uint32_t s_nonce_issued_ms = 0;

bool ota_nonce_issue(uint8_t out_nonce[32]) {
    for (int i = 0; i < 8; i++) {
        uint32_t r = esp_random();
        memcpy(s_nonce + i * 4, &r, 4);
    }
    memcpy(out_nonce, s_nonce, 32);
    s_nonce_issued_ms = (uint32_t)millis();
    s_nonce_valid = true;
    return true;
}

bool ota_nonce_verify(const uint8_t nonce[32], const uint8_t sha256[32], const uint8_t sig[64]) {
    bool was_valid     = s_nonce_valid;
    uint32_t issued_at = s_nonce_issued_ms;
    s_nonce_valid = false;  // consume unconditionally

    if (!was_valid) return false;
    if ((uint32_t)millis() - issued_at > OTA_NONCE_TTL_MS) return false;
    if (memcmp(nonce, s_nonce, 32) != 0) return false;

    uint8_t message[64];
    memcpy(message,      nonce,  32);
    memcpy(message + 32, sha256, 32);
    return crypto_ed25519_check(sig, OWNER_PUBKEY, message, 64) == 0;
}

#ifdef OTA_AUTH_TEST_HOOKS
void ota_auth_test_inject_nonce(const uint8_t nonce[32]) {
    memcpy(s_nonce, nonce, 32);
    s_nonce_issued_ms = (uint32_t)millis();
    s_nonce_valid = true;
}
#endif
