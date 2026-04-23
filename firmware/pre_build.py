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
        f'extern const uint8_t OWNER_PUBKEY[32] = {{{byte_list}}};\n'
    )
    print(f"[pre_build] Written {generated_path}")


generate_owner_pubkey(None, None, env)  # noqa: F821
