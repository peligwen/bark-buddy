"""
MicroPython NDJSON handler for MechDog hybrid mode.

Runs on stock firmware's MicroPython. Speaks the same JSON protocol as
the custom C++ firmware, but uses the stock _dog.move() for servo control
and native IMU/sonar/battery access.

Uploaded and exec'd via REPL by HybridTransport. Once running, the serial
port switches from REPL mode to streaming NDJSON — no more >>> prompts.

Protocol: same as firmware/include/protocol.h
  Commands:  cmd_move, cmd_stand, cmd_balance, cmd_servo, cmd_led, ping
  Telemetry: telem_imu (50Hz), telem_sonar (20Hz), telem_battery (1Hz)
  Control:   boot, pong, ack, error
"""

import sys
import ujson
import utime
import machine

# --- Hardware init (same sequence as REPL init) ---
import Hiwonder, Hiwonder_IIC, HW_MechDog

Hiwonder.disableLowPowerAlarm()
Hiwonder.__bt_open = 0
try:
    _bz = Hiwonder.__bz()
    _bz.setVolume(0)
    _bz.playTone(0, 0, 0)
except:
    pass

_dog = HW_MechDog.__global_dog
_dog.set_default_pose()
_imu = _dog.__imu
_bus = Hiwonder_IIC.IIC(1)
_sonar = Hiwonder_IIC.I2CSonar(_bus)
_sonar.setRGBMode(0)

# Battery ADC (GPIO 34, 11dB attenuation)
_bat_adc = machine.ADC(machine.Pin(34))
_bat_adc.atten(machine.ADC.ATTN_11DB)

# --- State ---
_mode = "idle"
_balance = False

# --- Motion mapping ---
_MOTION = {
    "forward":  (35, 0),
    "backward": (-25, 0),
    "left":     (20, -50),
    "right":    (20, 50),
    "stop":     (0, 0),
}

# --- Telemetry timing (ms) ---
_IMU_INTERVAL = 20      # 50Hz
_SONAR_INTERVAL = 50    # 20Hz
_BAT_INTERVAL = 1000    # 1Hz
_last_imu = 0
_last_sonar = 0
_last_bat = 0


def _send(msg):
    """Write one NDJSON line to serial."""
    sys.stdout.write(ujson.dumps(msg))
    sys.stdout.write('\n')


def _read_battery():
    """Read battery voltage via ADC. Returns millivolts."""
    raw = _bat_adc.read()
    # 12-bit ADC (0-4095), 11dB atten gives ~0-3.3V range
    # Voltage divider ratio ~3.9x on the MechDog board
    mv = int(raw * 3.9 * 3300 / 4095)
    return mv


def _push_telemetry():
    """Send telemetry if intervals have elapsed. Non-blocking."""
    global _last_imu, _last_sonar, _last_bat
    now = utime.ticks_ms()

    if utime.ticks_diff(now, _last_imu) >= _IMU_INTERVAL:
        _last_imu = now
        try:
            angles = _imu.read_angle()
            # read_angle() returns [roll, pitch] on this hardware
            _send({"type": "telem_imu",
                   "roll": angles[0], "pitch": angles[1], "yaw": 0})
        except:
            pass

    if utime.ticks_diff(now, _last_sonar) >= _SONAR_INTERVAL:
        _last_sonar = now
        try:
            d = _sonar.getDistance()
            _send({"type": "telem_sonar", "distance_mm": int(d * 10)})
        except:
            pass

    if utime.ticks_diff(now, _last_bat) >= _BAT_INTERVAL:
        _last_bat = now
        try:
            mv = _read_battery()
            pct = max(0, min(100, (mv - 6000) * 100 // 2400))
            _send({"type": "telem_battery",
                   "voltage_mv": mv, "pct": pct, "low": mv < 6400})
        except:
            pass


def _handle_cmd(msg):
    """Process one JSON command. Returns None or sends ack/pong."""
    global _mode, _balance
    t = msg.get("type", "")

    if t == "ping":
        _send({"type": "pong"})

    elif t == "cmd_move":
        d = msg.get("direction", "stop")
        s = msg.get("speed", 1.0)
        sp, dr = _MOTION.get(d, (0, 0))
        sp = int(sp * s)
        dr = int(dr * s)
        _dog.move(sp, dr)
        _mode = "walk" if d != "stop" else "idle"
        _send({"type": "ack", "ref_type": "cmd_move", "ok": True})

    elif t == "cmd_stand":
        _dog.move(0, 0)
        _dog.set_default_pose()
        _mode = "idle"
        _send({"type": "ack", "ref_type": "cmd_stand", "ok": True})

    elif t == "cmd_balance":
        _balance = msg.get("enabled", False)
        _dog.homeostasis(1 if _balance else 0)
        _send({"type": "ack", "ref_type": "cmd_balance", "ok": True})

    elif t == "cmd_led":
        led = msg.get("led", 1)
        r = msg.get("r", 0)
        g = msg.get("g", 0)
        b = msg.get("b", 0)
        _sonar.setRGB(led, r, g, b)
        _send({"type": "ack", "ref_type": "cmd_led", "ok": True})

    elif t == "cmd_servo":
        idx = msg.get("index", 0)
        us = msg.get("pulse_us", 1500)
        try:
            # Access stock firmware's servo module directly
            servos = HW_MechDog.__servos
            servos.pwm_servo_set_position(idx + 1, us)
            _send({"type": "ack", "ref_type": "cmd_servo", "ok": True})
        except Exception as e:
            _send({"type": "ack", "ref_type": "cmd_servo",
                   "ok": False, "error": str(e)})

    elif t == "cmd_calibrate":
        action = msg.get("action", "")
        if action == "stop":
            _dog.move(0, 0)
            _send({"type": "ack", "ref_type": "cmd_calibrate", "ok": True})
        else:
            _send({"type": "ack", "ref_type": "cmd_calibrate", "ok": True})

    elif t == "cmd_status":
        _send({"type": "telem_status", "mode": _mode,
               "balance": _balance, "servos": True})

    else:
        _send({"type": "ack", "ref_type": t, "ok": False,
               "error": "unknown command"})


# --- Main loop ---

# Set LED to green (connected)
_sonar.setRGB(1, 0, 15, 0)
_sonar.setRGB(2, 0, 15, 0)

# Send boot message
_send({"type": "boot", "firmware": "hybrid", "version": "1.0",
       "imu": True, "sonar": True, "servos": True, "battery": True})

# Use select.poll for non-blocking stdin
import select as _sel
_poll = _sel.poll()
_poll.register(sys.stdin, _sel.POLLIN)

_buf = ""

while True:
    # Check for incoming commands (non-blocking)
    while _poll.poll(0):
        ch = sys.stdin.read(1)
        if ch is None:
            break
        if ch == '\n':
            line = _buf.strip()
            _buf = ""
            if line:
                try:
                    msg = ujson.loads(line)
                    _handle_cmd(msg)
                except ValueError:
                    _send({"type": "error", "msg": "invalid json"})
                except Exception as e:
                    _send({"type": "error", "msg": str(e)})
        else:
            _buf += ch

    # Push telemetry
    _push_telemetry()

    # Yield to other tasks (stock firmware background threads)
    utime.sleep_ms(2)
