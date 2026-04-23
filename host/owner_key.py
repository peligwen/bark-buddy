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
