#pragma once
#include <stdint.h>
#include <stdbool.h>

#define OTA_NONCE_TTL_MS 30000U

// Issue a fresh nonce. Fills out_nonce[32]. Replaces any pending nonce.
bool ota_nonce_issue(uint8_t out_nonce[32]);

// Verify nonce+signature. Consumes the nonce unconditionally.
// nonce[32]  — the nonce the host claims was issued
// sha256[32] — the firmware binary SHA-256
// sig[64]    — Ed25519 signature over (nonce_bytes || sha256_bytes)
// Returns true only if fresh, matching, and signature verifies.
bool ota_nonce_verify(const uint8_t nonce[32], const uint8_t sha256[32], const uint8_t sig[64]);

#ifdef OTA_AUTH_TEST_HOOKS
void ota_auth_test_inject_nonce(const uint8_t nonce[32]);
#endif
