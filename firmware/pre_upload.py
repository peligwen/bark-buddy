"""
pre_upload.py — PlatformIO pre-upload extra script.

Before esptool resets the ESP32 into bootloader mode, this script sends
cmd_update_begin over serial so the firmware can start the rainbow LEDs and
detach servos. The rainbow plays during the build/transfer phase and stops
once the bootloader takes over.

If no firmware is running (fresh board, already in bootloader, etc.) the
serial write will simply time out harmlessly.
"""
import serial
import time

Import("env")  # noqa: F821  (PlatformIO injects this)


def before_upload(source, target, env):  # noqa: ANN001
    upload_port = env.get("UPLOAD_PORT") or env.GetProjectOption("upload_port", "")
    baud = int(env.GetProjectOption("monitor_speed", 115200))

    if not upload_port:
        # Let PlatformIO auto-detect; nothing to do here without a known port.
        print("[pre_upload] No upload port configured — skipping cmd_update_begin")
        return

    try:
        with serial.Serial(upload_port, baud, timeout=0.5) as ser:
            ser.write(b'{"type":"cmd_update_begin"}\n')
            ser.flush()
            # Give the firmware ~300ms to process and start the rainbow
            time.sleep(0.3)
        print(f"[pre_upload] cmd_update_begin sent on {upload_port}")
    except Exception as exc:
        # Non-fatal — esptool will proceed regardless
        print(f"[pre_upload] Could not send cmd_update_begin: {exc}")


env.AddPreAction("upload", before_upload)  # noqa: F821
