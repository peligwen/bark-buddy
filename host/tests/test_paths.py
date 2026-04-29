"""Tests for _paths helpers."""

import os
import tempfile
import pytest

from _paths import compute_sha256, parse_fw_tcp, firmware_dir, firmware_binary_path


def test_compute_sha256_known_value():
    with tempfile.NamedTemporaryFile(delete=False) as f:
        f.write(b"hello world")
        path = f.name
    try:
        h = compute_sha256(path)
        # echo -n "hello world" | shasum -a 256
        assert h == "b94d27b9934d3e08a52e52d7da7dabfac484efe37a5380ee9088f7ace2efcde9"
    finally:
        os.unlink(path)


def test_compute_sha256_empty_file():
    with tempfile.NamedTemporaryFile(delete=False) as f:
        path = f.name
    try:
        h = compute_sha256(path)
        assert h == "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855"
    finally:
        os.unlink(path)


@pytest.mark.parametrize("input_str,expected", [
    ("192.168.1.1",          ("192.168.1.1", 9000)),
    ("192.168.1.1:9001",     ("192.168.1.1", 9001)),
    ("127.0.0.1:9000",       ("127.0.0.1",   9000)),
    ("localhost",            ("localhost",   9000)),
    ("localhost:8456",       ("localhost",   8456)),
])
def test_parse_fw_tcp(input_str, expected):
    assert parse_fw_tcp(input_str) == expected


def test_parse_fw_tcp_default_port_override():
    assert parse_fw_tcp("host.example", default_port=1234) == ("host.example", 1234)


def test_parse_fw_tcp_invalid_port_raises():
    with pytest.raises(ValueError):
        parse_fw_tcp("192.168.1.1:abc")


def test_firmware_paths_are_absolute():
    assert os.path.isabs(firmware_dir())
    assert os.path.isabs(firmware_binary_path())
    # binary path lives under firmware dir
    assert firmware_binary_path().startswith(firmware_dir() + os.sep)
