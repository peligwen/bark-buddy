"""Tests for transport_priority — pure function over label strings."""

import pytest

from supervisor import transport_priority


@pytest.mark.parametrize("label,expected", [
    ("mock",                    0),
    ("fw-wifi:192.168.1.42",    1),
    ("fw-usb:/dev/cu.usbserial-10", 2),
    ("fw-usb:COM3",             2),
    ("fw:/dev/cu.usbserial-10", 2),  # legacy detector format, USB
    ("fw:tty.usbserial-10",     2),  # legacy detector format, USB
    ("fw:192.168.1.42",         1),  # legacy detector format, WiFi by content
    ("fw",                      0),  # no rest, fall through
    ("unknown",                 0),
    ("",                        0),
])
def test_transport_priority_classification(label, expected):
    assert transport_priority(label) == expected


def test_usb_outranks_wifi_outranks_mock():
    assert transport_priority("fw-usb:/dev/cu.usbserial-10") > \
           transport_priority("fw-wifi:192.168.1.42") > \
           transport_priority("mock")
