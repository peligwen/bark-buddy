"""
Base class for MechDog hardware transports (USB REPL and WiFi WebREPL).

Translates CMD protocol strings into MicroPython REPL calls.
Subclasses only need to implement the I/O layer: _exec() and _exec_read().
"""

import abc
import asyncio
import logging
import time
from typing import Optional

from comms import Transport, READ_TIMEOUT
from dead_reckoning import DeadReckoningMixin

logger = logging.getLogger(__name__)

# MechDog REPL command mappings: CMD motion sub-code → Python call
# move(speed, direction) where direction is yaw angle (neg=left, pos=right)
MOTION_CMDS = {
    1: "_dog.move(0, 0)",         # stop
    2: "_dog.set_default_pose()", # stand
    3: "_dog.move(35, 0)",        # forward
    4: "_dog.move(-25, 0)",       # backward
    5: "_dog.move(20, -50)",      # turn left
    6: "_dog.move(20, 50)",       # turn right
}

# REPL init commands — run once after connecting
INIT_COMMANDS = [
    "import Hiwonder, Hiwonder_IIC, HW_MechDog",
    "Hiwonder.disableLowPowerAlarm()",
    "Hiwonder.__bt_open = 0",  # disable battery alarm thread flag
    # Mute buzzer — single-line to avoid REPL multi-line block issues
    "exec('try:\\n _bz=Hiwonder.__bz();_bz.setVolume(0);_bz.playTone(0,0,0)\\nexcept:\\n pass')",
    "_dog = HW_MechDog.__global_dog",
    "_dog.set_default_pose()",  # clear stop_flag so move() works
    "_imu = _dog.__imu",
    "_bus = Hiwonder_IIC.IIC(1)",
    "_sonar = Hiwonder_IIC.I2CSonar(_bus)",
    "_sonar.setRGBMode(0)",  # manual RGB control
]

# Extra init for USB REPL only — start WebREPL for WiFi switching
USB_INIT_COMMANDS = [
    "exec('try:\\n import webrepl;webrepl.start()\\nexcept:\\n pass')",
]

# Sonar LED brightness (0-255). Keep low for subtle indicator.
LED_BRIGHTNESS = 15

# Sonar LED colors for status indication — values are scaled by LED_BRIGHTNESS
LED_COLORS = {
    "init":       (0, 0, 1.0),     # blue — initializing
    "ready":      (0, 0, 1.0),     # blue — ready for connection
    "connected":  (0, 1.0, 0),     # green — connected
    "controlled": (1.0, 0.5, 0),   # amber — being controlled
    "shutdown":   (1.0, 0, 0),     # red — shutting down / restarting
}


class HardwareTransport(DeadReckoningMixin, Transport):
    """
    Abstract base for transports that talk to MechDog via MicroPython REPL.

    Handles CMD→Python translation, dead reckoning, and response formatting.
    Subclasses implement _exec(), _exec_read(), open(), and close().
    """

    def __init__(self):
        self._open = False
        self._last_response: Optional[str] = None
        self._init_dead_reckoning()

    async def _init_repl(self) -> None:
        """Run REPL init commands to set up MechDog objects."""
        for cmd in INIT_COMMANDS:
            await self._exec(cmd)
        await self.set_led_status("connected")

    async def set_led_status(self, status: str) -> None:
        """Set sonar LED color to indicate transport status."""
        color = LED_COLORS.get(status)
        if not color or not self._open:
            return
        r = int(color[0] * LED_BRIGHTNESS)
        g = int(color[1] * LED_BRIGHTNESS)
        b = int(color[2] * LED_BRIGHTNESS)
        await self._exec(f"_sonar.setRGB(1, {r}, {g}, {b})")
        await self._exec(f"_sonar.setRGB(2, {r}, {g}, {b})")

    async def exec_repl(self, cmd: str) -> None:
        """Execute a MicroPython REPL command (public API for server use)."""
        await self._exec(cmd)

    # --- Transport interface ---

    async def send(self, data: str) -> None:
        if not self._open:
            raise ConnectionError("Transport not open")
        data = data.strip()
        if not (data.startswith("CMD|") and data.endswith("|$")):
            return
        inner = data[4:-2]
        parts = inner.split("|")
        if not parts:
            return
        response = await self._handle_cmd(parts[0], parts[1:])
        if response:
            self._last_response = response

    async def recv(self, timeout: float = READ_TIMEOUT) -> Optional[str]:
        if not self._open:
            raise ConnectionError("Transport not open")
        resp = self._last_response
        self._last_response = None
        return resp

    def is_open(self) -> bool:
        return self._open

    # --- CMD translation ---

    async def _handle_cmd(self, func: str, args: list[str]) -> Optional[str]:
        """Translate a CMD function + args into REPL calls, return CMD response."""
        if func == "1":
            # Posture: args[0]=sub (1=pitch,2=roll,3=balance,4=height)
            sub = args[0] if args else "3"
            val = args[1] if len(args) > 1 else "0"
            if sub == "3":
                await self._exec(f"_dog.homeostasis({1 if val == '1' else 0})")
            return "CMD|1|OK|$"

        elif func == "2":
            # Action group: args[0]=1, args[1]=action_code
            code = int(args[1]) if len(args) > 1 else 1
            if code == 5:
                # Lie down: lower body then deinit servos
                for _ in range(5):
                    await self._exec("_dog.transform([0, 0, -1], [0, 0, 0], 80)")
                await asyncio.sleep(0.5)
                await self._exec("HW_MechDog.__servos.pwm_servo_deinit()")
            elif code == 99:
                # Wake up: re-init servos and stand (used internally)
                await self._exec("_dog.set_default_pose()")
            else:
                await self._exec(f"_dog.action_run({code})")
            return "CMD|2|OK|$"

        elif func == "3":
            # Motion control
            sub = int(args[0]) if args else 1
            self._step_dead_reckoning()
            self._motion_cmd = sub
            self._last_motion_time = time.monotonic()
            py_cmd = MOTION_CMDS.get(sub)
            if py_cmd:
                await self._exec(py_cmd)
            return "CMD|3|OK|$"

        elif func == "4":
            # Ultrasonic — getDistance() returns cm
            result = await self._exec_read("print(_sonar.getDistance())")
            try:
                dist_mm = int(float(result.strip()) * 10) if result.strip() else 0
                return f"CMD|4|{dist_mm}|$"
            except (ValueError, AttributeError):
                return "CMD|4|0|$"

        elif func == "5":
            # IMU — read_angle() returns [roll, pitch]
            result = await self._exec_read("print(_imu.read_angle())")
            try:
                result = result.strip()
                if result.startswith("[") and result.endswith("]"):
                    vals = result[1:-1].split(",")
                    if len(vals) >= 2:
                        roll = float(vals[0].strip())
                        pitch = float(vals[1].strip())
                        return f"CMD|5|{pitch}|{roll}|$"
            except (ValueError, IndexError, AttributeError):
                pass
            return "CMD|5|0.0|0.0|$"

        elif func == "6":
            # Battery — not exposed via REPL, return hardcoded nominal
            return "CMD|6|7400|$"

        return None

    async def check_wifi_status(self) -> dict:
        """Check if the dog has WiFi connected. Returns {connected, ip, ssid}."""
        if not self._open:
            return {"connected": False}
        result = await self._exec_read(
            "import network; s=network.WLAN(network.STA_IF); "
            "print(s.isconnected(), s.ifconfig()[0] if s.isconnected() else '', "
            "s.config('essid') if s.active() else '')"
        )
        try:
            parts = result.strip().split()
            connected = parts[0] == "True" if parts else False
            ip = parts[1] if len(parts) > 1 and connected else ""
            ssid = parts[2] if len(parts) > 2 else ""
            return {"connected": connected, "ip": ip, "ssid": ssid}
        except Exception:
            return {"connected": False}

    async def setup_wifi(self, ssid: str, password: str) -> dict:
        """Connect the dog to WiFi and start WebREPL. Returns {ok, ip, error}."""
        if not self._open:
            return {"ok": False, "error": "Transport not open"}
        try:
            await self._exec("import network")
            await self._exec("_sta = network.WLAN(network.STA_IF)")
            await self._exec("_sta.active(True)")
            await self._exec(f'_sta.connect("{ssid}", "{password}")')
            # Wait for connection
            await self._exec("import time")
            await self._exec(
                "exec('for _ in range(20):\\n if _sta.isconnected(): break\\n time.sleep(0.5)')"
            )
            result = await self._exec_read("print(_sta.isconnected(), _sta.ifconfig()[0])")
            parts = result.strip().split()
            if parts and parts[0] == "True":
                ip = parts[1] if len(parts) > 1 else "unknown"
                # Start WebREPL
                await self._exec('f=open("webrepl_cfg.py","w"); f.write("PASS=\\"bark\\"\\n"); f.close()')
                await self._exec("import webrepl; webrepl.start()")
                return {"ok": True, "ip": ip}
            return {"ok": False, "error": "WiFi connection failed"}
        except Exception as e:
            return {"ok": False, "error": str(e)}

    # --- Abstract I/O (subclasses implement these) ---

    @abc.abstractmethod
    async def _exec(self, cmd: str) -> None:
        """Execute a Python command on the REPL (fire and forget)."""
        ...

    @abc.abstractmethod
    async def _exec_read(self, cmd: str) -> str:
        """Execute a Python command and return its printed output."""
        ...
