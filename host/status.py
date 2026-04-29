"""
Session status assembly.

Single source for the augmented `telem_status` the host sends to browsers.
Pulls from transport, balance, and OTA so all callers (initial WS connect,
1 Hz battery broadcast, transport replace) emit identical shape.
"""

from typing import Any


def build_telem_status(
    *,
    transport,
    balance,
    transport_label: str,
    available_fw_version: str,
    mode: str,
    battery_mv: int | None = None,
) -> dict[str, Any]:
    open_ = transport.is_open() if transport else False
    engaged = transport.get_engaged() if transport else False
    ramping = transport.get_ramping() if transport else False
    cutoff = transport.get_battery_cutoff() if transport else False
    present = transport.get_battery_present() if transport else True
    fw_version = transport.get_fw_version() if transport else ""
    fw_info = transport.firmware_info if transport else {}

    if transport is None:
        lifecycle = "unknown"
    elif ramping:
        lifecycle = "ramping"
    elif engaged:
        lifecycle = "active"
    else:
        lifecycle = "disengaged"

    msg: dict[str, Any] = {
        "type": "telem_status",
        "mode": mode,
        "balance": balance.enabled,
        "fallen": balance.is_fallen,
        "connected": open_,
        "engaged": engaged,
        "ramping": ramping,
        "battery_cutoff": cutoff,
        "battery_present": present,
        "fw_version": fw_version,
        "available_fw_version": available_fw_version,
        "transport": transport_label,
        "lifecycle": lifecycle,
    }
    if battery_mv is not None:
        msg["battery_mv"] = battery_mv
    ota_status = fw_info.get("ota_status")
    if ota_status:
        msg["ota_status"] = ota_status
    return msg
