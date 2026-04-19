# OTA Owner-Key Authentication Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Gate WiFi OTA firmware updates behind an Ed25519 owner key that is compiled into the firmware at USB-flash time, so only the host that last USB-flashed the device can push OTA updates.

**Architecture:** A per-host Ed25519 keypair lives in `~/.bark/`. A PlatformIO pre-build script reads the public key and writes it into a generated C++ source file compiled into every firmware build. WiFi OTA uses a two-message nonce+signature handshake; serial OTA bypasses it. A new `host/owner_key.py` module owns key generation and signing.

**Tech Stack:** monocypher 4.0.2 (Ed25519 verify, vendored under `firmware/lib/monocypher/`), Python `cryptography` package (host signing), mbedtls/sha256 (already in firmware for OTA hash), ArduinoJson 7.

---

## File Map

**New files:**
- `firmware/lib/monocypher/monocypher.c` — vendored Ed25519 verify-only C library
- `firmware/lib/monocypher/monocypher.h` — vendored header
- `firmware/lib/monocypher/library.json` — PlatformIO library descriptor
- `firmware/include/ota_auth.h` — nonce issue + verify API
- `firmware/src/ota_auth.cpp` — nonce state machine + signature check
- `firmware/test/test_ota_auth.cpp` — unit tests for ota_auth
- `firmware/test/test_owner_pubkey.cpp` — known test pubkey for mock+unit builds
- `firmware/pre_build.py` — PlatformIO extra_script: writes `owner_pubkey.generated.cpp`
- `firmware/src/owner_pubkey.generated.cpp` — generated per build, gitignored
- `host/owner_key.py` — Ed25519 keypair load/generate/sign

**Modified files:**
- `firmware/platformio.ini` — add `pre:pre_build.py` to extra_scripts
- `firmware/include/protocol.h` — add `MSG_CMD_OTA_REQUEST_NONCE`, `MSG_TELEM_OTA_NONCE`
- `firmware/include/command_handlers.h` — add `set_msg_source_serial(bool)`
- `firmware/src/command_handlers.cpp` — s_from_serial flag; new nonce handler; sig verify in handle_cmd_ota_update
- `firmware/src/main.cpp` — call set_msg_source_serial before serial/TCP loops
- `firmware/test/mock_arduino.h` — add esp_random() stub
- `firmware/mock/esp_compat.h` — add esp_random() stub for mock build
- `firmware/test/Makefile` — add test_ota_auth target; add ota_auth.cpp + monocypher.c to MOCK_SRC
- `host/ota_flash.py` — nonce request, sign, extended cmd_ota_update; friendly error on reason:sig
- `bark_cli.py` — call ensure_owner_key() before USB flash
- `.gitignore` — add `firmware/src/owner_pubkey.generated.cpp`

---

## Task 1: Vendor monocypher + add esp_random stub

**Files:**
- Create: `firmware/lib/monocypher/monocypher.c`
- Create: `firmware/lib/monocypher/monocypher.h`
- Create: `firmware/lib/monocypher/library.json`
- Modify: `firmware/test/mock_arduino.h`
- Modify: `firmware/mock/esp_compat.h`

- [ ] **Step 1: Clone monocypher 4.0.2 and copy the two source files**

```bash
cd /tmp
git clone --depth 1 --branch 4.0.2 https://github.com/LoupVaillant/Monocypher.git monocypher-src
mkdir -p /Users/gwen/workspace/bark-buddy/firmware/lib/monocypher
cp /tmp/monocypher-src/src/monocypher.c /Users/gwen/workspace/bark-buddy/firmware/lib/monocypher/
cp /tmp/monocypher-src/src/monocypher.h /Users/gwen/workspace/bark-buddy/firmware/lib/monocypher/
```

- [ ] **Step 2: Write library.json for PlatformIO auto-discovery**

Create `firmware/lib/monocypher/library.json`:
```json
{
  "name": "monocypher",
  "version": "4.0.2",
  "description": "Verify-only Ed25519 (RFC 8032) for OTA signature checking",
  "build": {
    "srcDirs": ["."]
  }
}
```

- [ ] **Step 3: Add esp_random() to firmware/test/mock_arduino.h**

In `firmware/test/mock_arduino.h`, add after the `delay` stub (after line ~29 `inline void delay(unsigned long ms) { _mock_millis += ms; }`):
```cpp
#include <cstdlib>
inline uint32_t esp_random() { return (uint32_t)rand(); }
```

- [ ] **Step 4: Add esp_random() to firmware/mock/esp_compat.h**

In `firmware/mock/esp_compat.h`, add after the existing content:
```cpp
#include <cstdlib>
inline uint32_t esp_random() { return (uint32_t)rand(); }
```

- [ ] **Step 5: Quick compile check**

```bash
cd /Users/gwen/workspace/bark-buddy/firmware/lib/monocypher
clang++ -std=c++17 -x c++ monocypher.c -c -o /dev/null && echo "OK"
```
Expected: `OK`

- [ ] **Step 6: Commit**

```bash
cd /Users/gwen/workspace/bark-buddy
git add firmware/lib/monocypher/ firmware/test/mock_arduino.h firmware/mock/esp_compat.h
git commit -m "feat(firmware): vendor monocypher 4.0.2; add esp_random stub to test/mock"
```

---

## Task 2: Generate test keypair constants

**Files:**
- Create: `firmware/test/test_owner_pubkey.cpp`

We need a deterministic test pubkey so unit tests and mock builds can verify signatures without the real `~/.bark/` key. Generate the bytes in Step 1, embed them in Step 2.

- [ ] **Step 1: Generate test keypair bytes and save to file**

```bash
python3 -c "
from cryptography.hazmat.primitives.asymmetric.ed25519 import Ed25519PrivateKey
from cryptography.hazmat.primitives.serialization import Encoding, PublicFormat, PrivateFormat, NoEncryption
seed = bytes(range(32))
priv = Ed25519PrivateKey.from_private_bytes(seed)
pub = priv.public_key().public_bytes(Encoding.Raw, PublicFormat.Raw)
nonce_bytes = bytes(32)
sha256_bytes = bytes(range(32, 64))
sig = priv.sign(nonce_bytes + sha256_bytes)
print('PUBKEY_HEX:', pub.hex())
print('SIG_HEX:', sig.hex())
print('NONCE_HEX:', nonce_bytes.hex())
print('SHA256_HEX:', sha256_bytes.hex())
" | tee /tmp/bark_test_vectors.txt
```

Expected output: four lines with PUBKEY_HEX (64 chars), SIG_HEX (128 chars), NONCE_HEX (64 chars), SHA256_HEX (64 chars). Save `/tmp/bark_test_vectors.txt` — you'll need these values in the next two steps.

- [ ] **Step 2: Write test_owner_pubkey.cpp**

Read the PUBKEY_HEX from `/tmp/bark_test_vectors.txt` and create `firmware/test/test_owner_pubkey.cpp`. Replace each `0xXX` with the actual byte values from the 64-char hex string (32 bytes = 64 hex chars):

```bash
python3 -c "
import pathlib
pub_hex = open('/tmp/bark_test_vectors.txt').readlines()[0].split(': ')[1].strip()
bytes_list = ', '.join(f'0x{pub_hex[i:i+2]}' for i in range(0, 64, 2))
content = f'''// Known test pubkey matching seed=bytes(range(32)).
// NEVER commit real owner keys here — test use only.
#include <stdint.h>
const uint8_t OWNER_PUBKEY[32] = {{{bytes_list}}};
'''
pathlib.Path('firmware/test/test_owner_pubkey.cpp').write_text(content)
print('Written firmware/test/test_owner_pubkey.cpp')
" 
```

Verify the file was written:
```bash
head -4 /Users/gwen/workspace/bark-buddy/firmware/test/test_owner_pubkey.cpp
```
Expected: starts with comment + `const uint8_t OWNER_PUBKEY[32] = {0xXX, ...};`

- [ ] **Step 3: Commit**

```bash
cd /Users/gwen/workspace/bark-buddy
git add firmware/test/test_owner_pubkey.cpp
git commit -m "test(firmware): add deterministic test Ed25519 pubkey for OTA auth tests"
```

---

## Task 3: host/owner_key.py

**Files:**
- Create: `host/owner_key.py`

- [ ] **Step 1: Write the failing test**

Create `/tmp/test_owner_key.py`:
```python
import sys, os, tempfile
sys.path.insert(0, "/Users/gwen/workspace/bark-buddy/host")
os.environ["BARK_KEY_DIR"] = tempfile.mkdtemp()
import owner_key  # should fail: module doesn't exist
```

```bash
python3 /tmp/test_owner_key.py 2>&1 | head -3
```
Expected: `ModuleNotFoundError: No module named 'owner_key'`

- [ ] **Step 2: Write owner_key.py**

Create `host/owner_key.py`:
```python
"""
Ed25519 owner keypair management for OTA authentication.

Keys live in BARK_KEY_DIR env var override, or ~/.bark/ by default.
  owner.ed25519      — 32-byte Ed25519 seed (private key), mode 0600
  owner.ed25519.pub  — 32-byte Ed25519 public key
"""
import os
import stat
from pathlib import Path

from cryptography.hazmat.primitives.asymmetric.ed25519 import Ed25519PrivateKey
from cryptography.hazmat.primitives.serialization import (
    Encoding,
    PrivateFormat,
    PublicFormat,
    NoEncryption,
)


def _key_dir() -> Path:
    override = os.environ.get("BARK_KEY_DIR")
    return Path(override) if override else Path.home() / ".bark"


def _private_key_path() -> Path:
    return _key_dir() / "owner.ed25519"


def _public_key_path() -> Path:
    return _key_dir() / "owner.ed25519.pub"


def ensure_owner_key() -> None:
    """Generate the owner keypair if it doesn't exist. Prints one line on first gen."""
    d = _key_dir()
    d.mkdir(parents=True, exist_ok=True)
    priv_path = _private_key_path()
    if not priv_path.exists():
        privkey = Ed25519PrivateKey.generate()
        priv_bytes = privkey.private_bytes(Encoding.Raw, PrivateFormat.Raw, NoEncryption())
        pub_bytes = privkey.public_key().public_bytes(Encoding.Raw, PublicFormat.Raw)
        priv_path.write_bytes(priv_bytes)
        os.chmod(priv_path, stat.S_IRUSR | stat.S_IWUSR)
        _public_key_path().write_bytes(pub_bytes)
        print(f"[owner] Generated owner key, saved to {priv_path}")


def load_private_key() -> Ed25519PrivateKey:
    return Ed25519PrivateKey.from_private_bytes(_private_key_path().read_bytes())


def sign_ota(nonce_hex: str, sha256_hex: str) -> str:
    """Return 128-hex Ed25519 signature over (nonce_bytes || sha256_bytes)."""
    privkey = load_private_key()
    message = bytes.fromhex(nonce_hex) + bytes.fromhex(sha256_hex)
    return privkey.sign(message).hex()


def public_key_bytes() -> bytes:
    """Return raw 32-byte public key."""
    return _public_key_path().read_bytes()
```

- [ ] **Step 3: Write full test and verify it passes**

Overwrite `/tmp/test_owner_key.py`:
```python
import sys, os, tempfile, stat
sys.path.insert(0, "/Users/gwen/workspace/bark-buddy/host")
os.environ["BARK_KEY_DIR"] = tempfile.mkdtemp()

import owner_key

owner_key.ensure_owner_key()
priv_path = owner_key._private_key_path()
pub_path  = owner_key._public_key_path()
assert priv_path.exists(), "private key file missing"
assert pub_path.exists(),  "public key file missing"
assert oct(stat.S_IMODE(os.stat(priv_path).st_mode)) == '0o600', "private key not 0600"
assert len(pub_path.read_bytes()) == 32, "pubkey not 32 bytes"

old_pub = pub_path.read_bytes()
owner_key.ensure_owner_key()
assert pub_path.read_bytes() == old_pub, "second call changed the key"

sig_hex = owner_key.sign_ota("00" * 32, "ab" * 32)
assert len(sig_hex) == 128, f"sig wrong length: {len(sig_hex)}"

from cryptography.hazmat.primitives.asymmetric.ed25519 import Ed25519PublicKey
pubkey = Ed25519PublicKey.from_public_bytes(pub_path.read_bytes())
pubkey.verify(bytes.fromhex(sig_hex), bytes.fromhex("00" * 32 + "ab" * 32))
print("ALL PASS")
```

```bash
python3 /tmp/test_owner_key.py
```
Expected: `ALL PASS`

- [ ] **Step 4: Commit**

```bash
cd /Users/gwen/workspace/bark-buddy
git add host/owner_key.py
git commit -m "feat(host): add Ed25519 owner key management for OTA auth"
```

---

## Task 4: firmware/pre_build.py

**Files:**
- Create: `firmware/pre_build.py`
- Modify: `firmware/platformio.ini`
- Modify: `.gitignore`

- [ ] **Step 1: Write pre_build.py**

Create `firmware/pre_build.py`:
```python
"""
pre_build.py — PlatformIO extra_script (pre-build phase).

Reads ~/.bark/owner.ed25519.pub (or BARK_OWNER_PUBKEY_FILE env override)
and writes firmware/src/owner_pubkey.generated.cpp with the 32-byte
public key as a C++ constant.

If absent, writes a zero key and prints a warning.
"""
import os
from pathlib import Path

Import("env")  # noqa: F821  (PlatformIO injects this)


def generate_owner_pubkey(source, target, env):  # noqa: ANN001
    pubkey_env = os.environ.get("BARK_OWNER_PUBKEY_FILE")
    if pubkey_env:
        pub_path = Path(pubkey_env)
    else:
        pub_path = Path.home() / ".bark" / "owner.ed25519.pub"

    if not pub_path.exists():
        print(f"[pre_build] WARNING: Owner pubkey not found at {pub_path}")
        print(f"[pre_build]          Run 'bark flash --usb' to generate your owner key.")
        print(f"[pre_build]          Using zero pubkey — OTA auth will reject all requests.")
        pub_bytes = bytes(32)
    else:
        pub_bytes = pub_path.read_bytes()
        if len(pub_bytes) != 32:
            raise RuntimeError(
                f"[pre_build] Owner pubkey at {pub_path} must be 32 bytes, got {len(pub_bytes)}"
            )
        print(f"[pre_build] Injecting owner pubkey from {pub_path}")

    byte_list = ", ".join(f"0x{b:02x}" for b in pub_bytes)
    generated_path = Path(env["PROJECT_SRC_DIR"]) / "owner_pubkey.generated.cpp"
    generated_path.write_text(
        '#include <stdint.h>\n'
        f'const uint8_t OWNER_PUBKEY[32] = {{{byte_list}}};\n'
    )
    print(f"[pre_build] Written {generated_path}")


env.AddPreAction("buildprog", generate_owner_pubkey)  # noqa: F821
```

- [ ] **Step 2: Update platformio.ini**

In `firmware/platformio.ini`, change `extra_scripts = pre:pre_upload.py` to:
```ini
extra_scripts =
    pre:pre_build.py
    pre:pre_upload.py
```

- [ ] **Step 3: Update .gitignore**

Add to the end of `.gitignore`:
```
firmware/src/owner_pubkey.generated.cpp
```

- [ ] **Step 4: Smoke-test the pre_build logic in isolation**

Create `/tmp/verify_prebuild.py` and run it:
```python
import sys, os, pathlib, tempfile
sys.path.insert(0, '/Users/gwen/workspace/bark-buddy/firmware')
tmp = tempfile.mkdtemp()
# Write a fake 32-byte pubkey
pub = pathlib.Path(tmp) / 'owner.ed25519.pub'
pub.write_bytes(bytes(range(32)))
os.environ['BARK_OWNER_PUBKEY_FILE'] = str(pub)

# Parse pre_build.py, extract generate_owner_pubkey, call it with a fake env
src = pathlib.Path('/Users/gwen/workspace/bark-buddy/firmware/pre_build.py').read_text()
src = src.replace('Import("env")', '')  # strip PIO-specific import
src = src.replace('env.AddPreAction("buildprog", generate_owner_pubkey)', '')
import types
mod = types.ModuleType('pre_build')
src_compiled = compile(src, 'pre_build.py', 'exec')
fake_env = {'PROJECT_SRC_DIR': tmp}
globs = {'__name__': 'pre_build'}
globs.update({'env': fake_env})

# Load function definition
for stmt in src.split('\n'):
    pass  # just need the function loaded

# Use importlib alternative: write to file and import
pathlib.Path(tmp + '/pre_build_test.py').write_text(
    src + f'\ngenerate_owner_pubkey(None, None, {{"PROJECT_SRC_DIR": {repr(tmp)}}})\n'
)
import subprocess, sys
result = subprocess.run([sys.executable, tmp + '/pre_build_test.py'], capture_output=True, text=True)
print(result.stdout, result.stderr)
out = pathlib.Path(tmp) / 'owner_pubkey.generated.cpp'
assert out.exists(), 'generated file missing'
content = out.read_text()
assert 'OWNER_PUBKEY' in content and '0x00' in content
print('pre_build OK')
```

```bash
python3 /tmp/verify_prebuild.py
```
Expected: `pre_build OK`

- [ ] **Step 5: Commit**

```bash
cd /Users/gwen/workspace/bark-buddy
git add firmware/pre_build.py firmware/platformio.ini .gitignore
git commit -m "feat(firmware): inject owner pubkey into firmware binary at build time"
```

---

## Task 5: firmware/include/ota_auth.h + firmware/src/ota_auth.cpp (TDD)

**Files:**
- Create: `firmware/include/ota_auth.h`
- Create: `firmware/src/ota_auth.cpp`
- Create: `firmware/test/test_ota_auth.cpp`
- Modify: `firmware/test/Makefile`

- [ ] **Step 1: Write the failing test**

Create `firmware/test/test_ota_auth.cpp`. In Step 2 of Task 2, you saved `/tmp/bark_test_vectors.txt`. Read the SIG_HEX value from that file and replace `PASTE_SIG_HEX_HERE` below.

```cpp
// test_ota_auth.cpp — unit tests for firmware/src/ota_auth.cpp
// Test keypair: seed = bytes(range(32)); vectors from Task 2 Step 1.
#include "mock_arduino.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>

// Provide the test pubkey (seed=bytes(range(32)))
#include "test_owner_pubkey.cpp"

// Pull in monocypher before ota_auth.cpp so it compiles
#include "../lib/monocypher/monocypher.h"

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
// Paste the SIG_HEX output from /tmp/bark_test_vectors.txt here:
static const char SIG_HEX[]    = "PASTE_SIG_HEX_HERE";

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
```

- [ ] **Step 2: Add test_ota_auth to Makefile and verify it fails to build**

In `firmware/test/Makefile`:

**2a.** Append `test_ota_auth` to the `TARGETS` line (line 3).

**2b.** Add the build rule (use a real tab character for indentation — Makefile requires tabs):

```makefile
test_ota_auth: test_ota_auth.cpp mock_arduino.h test_owner_pubkey.cpp \
    ../include/ota_auth.h ../src/ota_auth.cpp \
    ../lib/monocypher/monocypher.h ../lib/monocypher/monocypher.c
	$(CXX) $(CXXFLAGS) -DMOCK_FIRMWARE=1 -DOTA_AUTH_TEST_HOOKS \
	    -I../lib/monocypher \
	    -I../.pio/libdeps/mechdog/ArduinoJson/src \
	    -o $@ test_ota_auth.cpp ../src/ota_auth.cpp \
	    ../lib/monocypher/monocypher.c
```

```bash
cd /Users/gwen/workspace/bark-buddy/firmware/test
make test_ota_auth 2>&1 | head -5
```
Expected: compile error `ota_auth.h` not found (confirms test is wired up before impl).

- [ ] **Step 3: Write firmware/include/ota_auth.h**

Create `firmware/include/ota_auth.h`:
```cpp
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
```

- [ ] **Step 4: Write firmware/src/ota_auth.cpp**

Create `firmware/src/ota_auth.cpp`:
```cpp
#include "ota_auth.h"
#include <string.h>
#include <monocypher.h>

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
    return crypto_eddsa_check(sig, OWNER_PUBKEY, message, 64) == 0;
}

#ifdef OTA_AUTH_TEST_HOOKS
void ota_auth_test_inject_nonce(const uint8_t nonce[32]) {
    memcpy(s_nonce, nonce, 32);
    s_nonce_issued_ms = (uint32_t)millis();
    s_nonce_valid = true;
}
#endif
```

- [ ] **Step 5: Build and run test_ota_auth**

```bash
cd /Users/gwen/workspace/bark-buddy/firmware/test
make test_ota_auth && ./test_ota_auth
```
Expected:
```
=== test_ota_auth ===
  PASS  T1: valid sig accepted
  PASS  T2: bad sig rejected
  PASS  T3: wrong sha256 rejected
  PASS  T4: wrong nonce rejected
  PASS  T5: replay rejected
  PASS  T6: expired nonce rejected
  PASS  T7: nonce issue returns true
  PASS  T7: issued nonce non-zero
=== 8 passed, 0 failed ===
```

If T1 fails: the SIG_HEX in the test file doesn't match the OWNER_PUBKEY in `test_owner_pubkey.cpp`. Regenerate both from the same seed by re-running Task 2 Step 1, then re-paste the values.

- [ ] **Step 6: Run all tests**

```bash
cd /Users/gwen/workspace/bark-buddy/firmware/test
make test
```
Expected: all existing tests pass.

- [ ] **Step 7: Commit**

```bash
cd /Users/gwen/workspace/bark-buddy
git add firmware/include/ota_auth.h firmware/src/ota_auth.cpp \
    firmware/test/test_ota_auth.cpp firmware/test/Makefile
git commit -m "feat(firmware): add OTA nonce+Ed25519 auth module; test_ota_auth passes"
```

---

## Task 6: Firmware protocol constants + handlers

**Files:**
- Modify: `firmware/include/protocol.h`
- Modify: `firmware/include/command_handlers.h`
- Modify: `firmware/src/command_handlers.cpp`
- Modify: `firmware/src/main.cpp`
- Modify: `firmware/test/Makefile` (MOCK_SRC)

- [ ] **Step 1: Add protocol constants to protocol.h**

In `firmware/include/protocol.h`, add two lines after `MSG_CMD_OTA_UPDATE` (after line 19):
```cpp
constexpr const char* MSG_CMD_OTA_REQUEST_NONCE = "cmd_ota_request_nonce";
constexpr const char* MSG_TELEM_OTA_NONCE       = "telem_ota_nonce";
```

- [ ] **Step 2: Expose set_msg_source_serial in command_handlers.h**

In `firmware/include/command_handlers.h`, add after `broadcast_servo_pins()`:
```cpp
// Set before processing each transport's input so handle_cmd_ota_update
// knows whether to enforce signature verification.
void set_msg_source_serial(bool is_serial);
```

- [ ] **Step 3: Add includes + s_from_serial + helpers + new handler to command_handlers.cpp**

**3a.** In `firmware/src/command_handlers.cpp`, after the closing `#endif` of the existing `#if WIFI_ENABLED` include block (after line 22), add:
```cpp
#include "ota_auth.h"
#include <monocypher.h>

static bool s_from_serial = false;

void set_msg_source_serial(bool is_serial) {
    s_from_serial = is_serial;
}
```

**3b.** Add hex-decode helpers just before `handle_cmd_ota_update` (before line 304):
```cpp
static bool hex_decode32(const char* hex, uint8_t out[32]) {
    if (!hex || strlen(hex) != 64) return false;
    for (int i = 0; i < 32; i++) {
        char b[3] = { hex[i*2], hex[i*2+1], 0 };
        char* end = nullptr;
        out[i] = (uint8_t)strtoul(b, &end, 16);
        if (end != b + 2) return false;
    }
    return true;
}

static bool hex_decode64(const char* hex, uint8_t out[64]) {
    if (!hex || strlen(hex) != 128) return false;
    for (int i = 0; i < 64; i++) {
        char b[3] = { hex[i*2], hex[i*2+1], 0 };
        char* end = nullptr;
        out[i] = (uint8_t)strtoul(b, &end, 16);
        if (end != b + 2) return false;
    }
    return true;
}
```

**3c.** Add `handle_cmd_ota_request_nonce` just before `handle_cmd_ota_update`:
```cpp
static void handle_cmd_ota_request_nonce(const JsonDocument&) {
#if !WIFI_ENABLED
    send_ack(MSG_CMD_OTA_REQUEST_NONCE, false, "wifi_disabled");
#else
    uint8_t nonce[32];
    ota_nonce_issue(nonce);
    char hex[65];
    for (int i = 0; i < 32; i++) snprintf(hex + i * 2, 3, "%02x", nonce[i]);
    hex[64] = '\0';
    JsonDocument resp;
    resp["type"]  = MSG_TELEM_OTA_NONCE;
    resp["nonce"] = hex;
    send_json(resp);
#endif
}
```

**3d.** In `handle_cmd_ota_update`, add the signature verification block after the missing-URL early return (after the `return;` on line ~313, before the TCP client IP check block at line 315). The new block goes between the `return;` for missing URL and the `// Only allow OTA downloads` comment:
```cpp
    // --- Signature check (WiFi only — serial is trusted via physical access) ---
#if WIFI_ENABLED
    if (!s_from_serial) {
        const char* nonce_hex = doc["nonce"] | "";
        const char* sig_hex   = doc["sig"]   | "";
        if (!nonce_hex[0] || !sig_hex[0]) {
            send_ack(MSG_CMD_OTA_UPDATE, false, "missing_auth");
            return;
        }
        uint8_t nonce_bytes[32], sha256_bytes[32], sig_bytes[64];
        if (!hex_decode32(nonce_hex, nonce_bytes) ||
            !hex_decode32(expected_sha256, sha256_bytes) ||
            !hex_decode64(sig_hex, sig_bytes)) {
            send_ack(MSG_CMD_OTA_UPDATE, false, "bad_auth_encoding");
            return;
        }
        if (!ota_nonce_verify(nonce_bytes, sha256_bytes, sig_bytes)) {
            send_ack(MSG_CMD_OTA_UPDATE, false, "sig");
            return;
        }
    }
#endif
```

**3e.** Add the new handler to `k_handlers[]`. After `{ MSG_CMD_OTA_UPDATE, handle_cmd_ota_update },` add:
```cpp
    { MSG_CMD_OTA_REQUEST_NONCE, handle_cmd_ota_request_nonce },
```

- [ ] **Step 4: Add set_msg_source_serial calls in main.cpp**

In `firmware/src/main.cpp`, in `loop()`:

Change the serial read block (around line 245):
```cpp
    // Read serial
    set_msg_source_serial(true);
    while (Serial.available()) {
        process_rx(serial_rx, serial_rx_pos, Serial.read(), now);
    }
```

Change the TCP read block (around line 257):
```cpp
        if (tcp_client && tcp_client.connected()) {
            set_msg_source_serial(false);
            while (tcp_client.available()) {
                process_rx(tcp_rx, tcp_rx_pos, tcp_client.read(), now);
            }
        }
```

- [ ] **Step 5: Add ota_auth.cpp + monocypher to MOCK_SRC in Makefile**

In `firmware/test/Makefile`:

**5a.** Add `-I../lib/monocypher \` to `MOCK_FLAGS` (after line 11, before `-include Arduino.h`).

**5b.** Add three lines to the end of `MOCK_SRC` (after `../src/servos.cpp`):
```makefile
	../src/ota_auth.cpp \
	../lib/monocypher/monocypher.c \
	test_owner_pubkey.cpp
```

- [ ] **Step 6: Build mock firmware**

```bash
cd /Users/gwen/workspace/bark-buddy/firmware/test
make bark-mock
```
Expected: compiles without errors.

- [ ] **Step 7: Run test_command_handlers**

```bash
cd /Users/gwen/workspace/bark-buddy/firmware/test
make test_command_handlers && ./test_command_handlers
```
Expected: all existing tests pass.

- [ ] **Step 8: Commit**

```bash
cd /Users/gwen/workspace/bark-buddy
git add firmware/include/protocol.h firmware/include/command_handlers.h \
    firmware/src/command_handlers.cpp firmware/src/main.cpp \
    firmware/test/Makefile
git commit -m "feat(firmware): wire cmd_ota_request_nonce + owner-sig gating into OTA handler"
```

---

## Task 7: host/ota_flash.py — nonce handshake + signing

**Files:**
- Modify: `host/ota_flash.py`

- [ ] **Step 1: Add import at top of ota_flash.py**

After the existing imports in `host/ota_flash.py`, add:
```python
from owner_key import sign_ota
```

- [ ] **Step 2: Replace step 7 with nonce-request + sign + send**

Find the step 7 block (around line 229-234) that currently reads:
```python
        # ------------------------------------------------------------------
        # 7. Send OTA command
        # ------------------------------------------------------------------
        print("[ota] Sending cmd_ota_update ...")
        await transport.send_json({
            "type": "cmd_ota_update",
            "url": firmware_url,
            "sha256": sha256_hex,
        })
```

Replace it with:
```python
        # ------------------------------------------------------------------
        # 7. Request nonce, sign, send authenticated OTA command
        # ------------------------------------------------------------------
        print("[ota] Requesting OTA nonce ...")
        nonce_event: asyncio.Event = asyncio.Event()
        nonce_holder: dict = {}

        def _nonce_cb(msg: dict) -> None:
            if msg.get("type") == "telem_ota_nonce":
                nonce_holder["nonce"] = msg.get("nonce", "")
                nonce_event.set()
            else:
                _telem_cb(msg)

        transport.set_telem_callback(_nonce_cb)
        await transport.send_json({"type": "cmd_ota_request_nonce"})
        try:
            await asyncio.wait_for(nonce_event.wait(), timeout=10.0)
        except asyncio.TimeoutError:
            print("[ota] ERROR: timed out waiting for nonce from device.")
            return 1
        finally:
            transport.set_telem_callback(_telem_cb)

        nonce_hex = nonce_holder.get("nonce", "")
        if len(nonce_hex) != 64:
            print(f"[ota] ERROR: invalid nonce from device: {nonce_hex!r}")
            return 1

        print("[ota] Signing OTA command ...")
        sig_hex = sign_ota(nonce_hex, sha256_hex)

        print("[ota] Sending cmd_ota_update ...")
        await transport.send_json({
            "type":   "cmd_ota_update",
            "url":    firmware_url,
            "sha256": sha256_hex,
            "nonce":  nonce_hex,
            "sig":    sig_hex,
        })
```

- [ ] **Step 3: Update the ack handler to print helpful ownership error**

Find in `_telem_cb`:
```python
        elif msg_type == "ack" and msg.get("ref_type") == "cmd_ota_update":
            if not msg.get("ok"):
                error = msg.get("error", "unknown")
                print(f"[ota] ERROR: firmware rejected cmd_ota_update: {error}")
                ota_result["ok"] = False
                done_event.set()
```

Change to:
```python
        elif msg_type == "ack" and msg.get("ref_type") == "cmd_ota_update":
            if not msg.get("ok"):
                error = msg.get("error", "unknown")
                if error == "sig":
                    print("[ota] ERROR: firmware rejected OTA — this device is owned by a different host.")
                    print("[ota]        Re-flash via USB to take ownership:  bark flash --usb")
                else:
                    print(f"[ota] ERROR: firmware rejected cmd_ota_update: {error}")
                ota_result["ok"] = False
                done_event.set()
```

- [ ] **Step 4: Commit**

```bash
cd /Users/gwen/workspace/bark-buddy
git add host/ota_flash.py
git commit -m "feat(host): add nonce+Ed25519 signing to WiFi OTA flow"
```

---

## Task 8: bark_cli.py — ensure key before USB flash

**Files:**
- Modify: `bark_cli.py`

- [ ] **Step 1: Add ensure_owner_key() to _do_usb_flash()**

In `bark_cli.py`, find `_do_usb_flash()` (around line 78). Change it from:
```python
def _do_usb_flash():
    result = subprocess.run(
        ["pio", "run", "-t", "upload"],
        cwd=FIRMWARE_DIR,
    )
    sys.exit(result.returncode)
```
to:
```python
def _do_usb_flash():
    _ensure_host_importable()
    from owner_key import ensure_owner_key
    ensure_owner_key()
    result = subprocess.run(
        ["pio", "run", "-t", "upload"],
        cwd=FIRMWARE_DIR,
    )
    sys.exit(result.returncode)
```

- [ ] **Step 2: Smoke-test CLI**

```bash
cd /Users/gwen/workspace/bark-buddy
python3 bark_cli.py flash --help
python3 bark_cli.py flash --wifi --help
```
Expected: both print help text without errors.

- [ ] **Step 3: Commit**

```bash
cd /Users/gwen/workspace/bark-buddy
git add bark_cli.py
git commit -m "feat(cli): generate owner key before USB flash"
```

---

## Task 9: CLAUDE.md update

- [ ] **Step 1: Update protocol section in CLAUDE.md**

In `CLAUDE.md`, in the "Custom Firmware Protocol" section:

Update the `cmd_ota_update` line:
```markdown
- `{"type": "cmd_ota_update", "url": "...", "sha256": "...", "nonce": "...", "sig": "..."}` — WiFi OTA (nonce+sig required on WiFi; serial path omits auth)
```

Add after it:
```markdown
- `{"type": "cmd_ota_request_nonce"}` — request a one-time nonce before WiFi OTA (WiFi only)
```

In the Telemetry section, add:
```markdown
- `{"type": "telem_ota_nonce", "nonce": "<64-hex>"}` — one-time nonce for signing OTA command
```

- [ ] **Step 2: Commit**

```bash
cd /Users/gwen/workspace/bark-buddy
git add CLAUDE.md
git commit -m "docs: update protocol spec with OTA owner-auth messages"
```

---

## Task 10: End-to-end verification

- [ ] **Step 1: Run full test suite**

```bash
cd /Users/gwen/workspace/bark-buddy/firmware/test
make test
```
Expected: all tests pass including `test_ota_auth`.

- [ ] **Step 2: Build mock firmware**

```bash
cd /Users/gwen/workspace/bark-buddy/firmware/test
make bark-mock
```
Expected: builds without errors.

- [ ] **Step 3: Mock end-to-end — setup test key environment**

```bash
python3 -c "
import os, stat
from pathlib import Path
from cryptography.hazmat.primitives.asymmetric.ed25519 import Ed25519PrivateKey
from cryptography.hazmat.primitives.serialization import Encoding, PrivateFormat, PublicFormat, NoEncryption
d = Path('/tmp/test-bark-keys'); d.mkdir(exist_ok=True)
priv = Ed25519PrivateKey.from_private_bytes(bytes(range(32)))
priv_path = d / 'owner.ed25519'
priv_path.write_bytes(priv.private_bytes(Encoding.Raw, PrivateFormat.Raw, NoEncryption()))
os.chmod(priv_path, 0o600)
(d / 'owner.ed25519.pub').write_bytes(priv.public_key().public_bytes(Encoding.Raw, PublicFormat.Raw))
print('Test keys written to', d)
"
```

- [ ] **Step 4: Mock end-to-end — OTA with correct owner key succeeds**

In terminal 1:
```bash
cd /Users/gwen/workspace/bark-buddy/firmware/test
./bark-mock --tcp-port 9001
```

In terminal 2:
```bash
cd /Users/gwen/workspace/bark-buddy
BARK_KEY_DIR=/tmp/test-bark-keys python3 -c "
import asyncio, sys
sys.path.insert(0, 'host')
from ota_flash import flash_wifi
rc = asyncio.run(flash_wifi(host='127.0.0.1', tcp_port=9001))
print('exit code:', rc)
"
```
Expected: nonce request → signing → flashing → `exit code: 0`

- [ ] **Step 5: Mock end-to-end — OTA with wrong key is rejected**

In terminal 2 (with mock still running or restarted):
```bash
BARK_KEY_DIR=$(mktemp -d) python3 -c "
import asyncio, sys, os
sys.path.insert(0, 'host')
import owner_key; owner_key.ensure_owner_key()  # generates a fresh (wrong) key
from ota_flash import flash_wifi
rc = asyncio.run(flash_wifi(host='127.0.0.1', tcp_port=9001))
print('exit code:', rc)
"
```
Expected:
```
[ota] ERROR: firmware rejected OTA — this device is owned by a different host.
[ota]        Re-flash via USB to take ownership:  bark flash --usb
exit code: 1
```
