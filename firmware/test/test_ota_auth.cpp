// test_ota_auth.cpp — unit tests for firmware/src/ota_auth.cpp
// Test keypair: seed = bytes(range(32)); vectors from Task 2 Step 1.
#include "mock_arduino.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>

// Provide the test pubkey (seed=bytes(range(32)))
#include "test_owner_pubkey.cpp"

// Pull in monocypher (standard Ed25519 with SHA-512) before ota_auth.cpp
#include "../lib/monocypher/monocypher-ed25519.h"

// Include the real implementation under test
#define OTA_AUTH_TEST_HOOKS
#include "../src/ota_auth.cpp"

static int g_pass = 0;
static int g_fail = 0;
static void check(bool cond, const char* label) {
    if (cond) { printf("  PASS  %s\n", label); g_pass++; }
    else       { printf("  FAIL  %s\n", label); g_fail++; }
}

static void hex_to_bytes(const char* hex, uint8_t* out, size_t len) {
    for (size_t i = 0; i < len; i++) {
        char b[3] = { hex[i*2], hex[i*2+1], 0 };
        out[i] = (uint8_t)strtol(b, nullptr, 16);
    }
}

// Test vectors from Task 2 Step 1 (nonce=bytes(32), sha256=bytes(range(32,64)))
static const char NONCE_HEX[]  = "0000000000000000000000000000000000000000000000000000000000000000";
static const char SHA256_HEX[] = "202122232425262728292a2b2c2d2e2f303132333435363738393a3b3c3d3e3f";
// SIG_HEX output from /tmp/bark_test_vectors.txt:
static const char SIG_HEX[]    = "af353a7184b58e2045d7a27d2d945d49344414de60f03b60f84235209f8da9d9334ff539c7627dad5cb94e4a051ccf1fc4cc37f4e7c449169eb3e5b40f960202";

int main() {
    uint8_t nonce[32], sha256[32], sig[64];
    hex_to_bytes(NONCE_HEX,  nonce,  32);
    hex_to_bytes(SHA256_HEX, sha256, 32);
    hex_to_bytes(SIG_HEX,    sig,    64);

    printf("=== test_ota_auth ===\n");

    // T1: valid sig accepted
    { mock_reset_clock(); ota_auth_test_inject_nonce(nonce);
      check(ota_nonce_verify(nonce, sha256, sig), "T1: valid sig accepted"); }

    // T2: bad sig rejected
    { mock_reset_clock();
      uint8_t bad[64]; memcpy(bad, sig, 64); bad[0] ^= 0xFF;
      ota_auth_test_inject_nonce(nonce);
      check(!ota_nonce_verify(nonce, sha256, bad), "T2: bad sig rejected"); }

    // T3: wrong sha256 rejected
    { mock_reset_clock();
      uint8_t bad[32]; memcpy(bad, sha256, 32); bad[0] ^= 0x01;
      ota_auth_test_inject_nonce(nonce);
      check(!ota_nonce_verify(nonce, bad, sig), "T3: wrong sha256 rejected"); }

    // T4: wrong nonce rejected (caller claims different nonce than what was stored)
    { mock_reset_clock();
      uint8_t other[32] = {}; other[0] = 0x01;
      ota_auth_test_inject_nonce(nonce);   // stored: all-zeros
      check(!ota_nonce_verify(other, sha256, sig), "T4: wrong nonce rejected"); }

    // T5: replay rejected (nonce consumed on first use)
    { mock_reset_clock();
      ota_auth_test_inject_nonce(nonce);
      ota_nonce_verify(nonce, sha256, sig);   // consumes
      check(!ota_nonce_verify(nonce, sha256, sig), "T5: replay rejected"); }

    // T6: expired nonce rejected
    { mock_reset_clock();
      ota_auth_test_inject_nonce(nonce);
      mock_advance_ms(OTA_NONCE_TTL_MS + 1);
      check(!ota_nonce_verify(nonce, sha256, sig), "T6: expired nonce rejected"); }

    // T7: ota_nonce_issue returns a non-zero value
    { uint8_t out[32] = {};
      bool issued = ota_nonce_issue(out);
      check(issued, "T7: nonce issue returns true");
      bool nonzero = false;
      for (int i = 0; i < 32; i++) if (out[i]) { nonzero = true; break; }
      check(nonzero, "T7: issued nonce non-zero"); }

    printf("=== %d passed, %d failed ===\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
