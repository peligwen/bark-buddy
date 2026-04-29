"""
TransportSupervisor — owns the active transport and the auto-switching policy.

Responsibilities:
- Hold the current `Dog` transport and its label.
- Reconnect on transient drop (with exponential backoff and OTA suppression).
- Auto-switch on mDNS discovery / USB hot-plug, gated by transport priority.
- Fan replacements out via a single `_switch_lock` so reconnect, mDNS, and
  hot-plug cannot race on the same transport reference.

The supervisor never reaches up: it calls back into the server through the
injected `on_replaced` (after a swap) and `broadcast_status` (to refresh
client-visible state). It owns no UI shape.

Transport priority (higher wins for auto-switching):
- `mock` (dev): 0
- `fw-wifi:<ip>`: 1
- `fw-usb:<port>`: 2

Hot-plug "ignore" semantics: lower-priority devices arriving while a
higher-priority transport is active are logged and discarded. mDNS-found
devices that lose discovery while we are connected to them downgrade us
to the disconnected state and let reconnect take over.
"""

import asyncio
import logging
from typing import Awaitable, Callable

logger = logging.getLogger(__name__)


_TRANSPORT_PRIORITY: dict[str, int] = {
    "mock":     0,
    "fw-wifi":  1,
    "fw-usb":   2,
}


def transport_priority(label: str) -> int:
    """Map a transport label to its auto-switch priority.

    Recognised label shapes (set at construction sites — discover.py, server.py,
    supervisor.py mDNS callback):
      - "mock"
      - "fw-wifi:<ip>" — explicit WiFi target (e.g. via --fw-tcp 192.168.x.y)
      - "fw-usb:<dev>"
      - "fw:<ip|dev>"  — disambiguated by content (legacy detector format)
    """
    if label == "mock":
        return 0
    if ":" not in label:
        return _TRANSPORT_PRIORITY.get(label, 0)
    prefix, rest = label.split(":", 1)
    if prefix in _TRANSPORT_PRIORITY:
        return _TRANSPORT_PRIORITY[prefix]
    if prefix == "fw":
        is_usb = rest.startswith("/dev/") or rest.startswith("COM") or "tty.usb" in rest
        return _TRANSPORT_PRIORITY["fw-usb" if is_usb else "fw-wifi"]
    return 0


class TransportSupervisor:
    def __init__(
        self,
        transport,
        label: str,
        *,
        no_mdns: bool,
        on_replaced: Callable[[object, str], Awaitable[None]],
        broadcast_status: Callable[..., Awaitable[None]],
    ):
        self._transport = transport
        self._label = label
        self._no_mdns = no_mdns
        self._on_replaced = on_replaced
        self._broadcast_status = broadcast_status

        self._switch_lock = asyncio.Lock()
        self._reconnect_task: asyncio.Task | None = None
        self._device_monitor = None
        self._mdns_browser = None
        self._detected_wifi: dict | None = None

    @property
    def transport(self):
        return self._transport

    @property
    def label(self) -> str:
        return self._label

    @property
    def detected_wifi(self) -> dict | None:
        return self._detected_wifi

    def clear_detected_wifi(self) -> None:
        self._detected_wifi = None

    def seed_detected_wifi(self, ip: str, port: int) -> None:
        self._detected_wifi = {"connected": True, "ip": ip, "port": port}

    # --- Lifecycle ---

    async def start(self) -> None:
        self._reconnect_task = asyncio.create_task(self._reconnect_loop(),
                                                   name="supervisor-reconnect")
        from device_monitor import DeviceMonitor
        self._device_monitor = DeviceMonitor(
            on_added=self._on_device_added,
            on_removed=self._on_device_removed,
        )
        await self._device_monitor.start()

        if not self._no_mdns:
            from mdns_browser import MdnsBrowser
            self._mdns_browser = MdnsBrowser(
                on_found=self._on_mdns_found,
                on_lost=self._on_mdns_lost,
            )
            await self._mdns_browser.start()

    async def stop(self) -> None:
        if self._mdns_browser:
            await self._mdns_browser.stop()
            self._mdns_browser = None
        if self._device_monitor:
            await self._device_monitor.stop()
            self._device_monitor = None
        if self._reconnect_task:
            self._reconnect_task.cancel()
            try:
                await self._reconnect_task
            except asyncio.CancelledError:
                pass
            self._reconnect_task = None

    # --- Transport replacement ---

    async def replace(self, new_transport, new_label: str) -> None:
        """Atomically tear down the current transport and swap in a new one.

        Holds `_switch_lock` for the entire swap. The server's `on_replaced`
        callback runs inside the lock so callers can re-bind behaviors and
        re-register callbacks before any other supervisor work resumes.
        """
        async with self._switch_lock:
            old = self._transport
            # Detach old callbacks BEFORE close so a final inbound line
            # cannot mutate state on the half-closed transport.
            for setter in ("set_ack_callback", "set_telem_callback"):
                if old and hasattr(old, setter):
                    getattr(old, setter)(None)
            try:
                if old and old.is_open():
                    await old.disengage_safe(timeout=0.5)
                    await old.close()
            except Exception:
                pass

            self._transport = new_transport
            self._label = new_label

            try:
                await new_transport.open()
            except Exception as e:
                logger.warning("supervisor: new transport open failed: %s — staying on %s",
                               e, new_label)

            await self._on_replaced(new_transport, new_label)
            await self._broadcast_status()

    # --- Reconnect loop ---

    async def _reconnect_loop(self) -> None:
        backoff = 1
        while True:
            try:
                await asyncio.sleep(2)
                fw = self._transport.firmware_info if self._transport else {}
                if fw.get("ota_status") in ("downloading", "flashing"):
                    backoff = 1
                    continue
                if self._switch_lock.locked():
                    continue
                async with self._switch_lock:
                    if not self._transport or self._transport.is_open():
                        backoff = 1
                        continue
                    captured = self._transport
                    logger.warning("supervisor: connection lost — reconnect (backoff=%ds)",
                                   backoff)
                    await self._broadcast_status()
                    try:
                        await captured.close()
                    except Exception:
                        pass
                await asyncio.sleep(backoff)
                async with self._switch_lock:
                    if self._transport is not captured:
                        continue
                    try:
                        await captured.open()
                        backoff = 1
                        logger.info("supervisor: reconnected")
                        await self._broadcast_status()
                    except Exception:
                        backoff = min(backoff * 2, 16)
                        logger.warning("supervisor: reconnect failed, next in %ds",
                                       backoff)
            except asyncio.CancelledError:
                break

    # --- mDNS callbacks ---

    async def _on_mdns_found(self, ip: str, port: int, props: dict) -> None:
        if self._detected_wifi and self._detected_wifi.get("ip") == ip:
            return
        self._detected_wifi = {
            "connected": True, "ip": ip, "port": port,
            "fw_version": props.get("fw_version", ""),
        }
        current = transport_priority(self._label)
        wifi = _TRANSPORT_PRIORITY["fw-wifi"]
        if wifi > current:
            logger.info("mDNS: auto-connecting to %s:%d", ip, port)
            from dog import Dog
            try:
                transport = Dog(host=ip, tcp_port=port)
                await self.replace(transport, f"fw-wifi:{ip}")
            except Exception as e:
                logger.warning("mDNS: auto-connect failed: %s", e)
        else:
            logger.info("mDNS: found %s:%d (not switching, current=%s pri=%d)",
                        ip, port, self._label, current)
            await self._broadcast_status()

    async def _on_mdns_lost(self, ip: str) -> None:
        if not self._detected_wifi or self._detected_wifi.get("ip") != ip:
            return
        self._detected_wifi = None
        if ip in self._label:
            logger.warning("mDNS: active device lost (%s) — disconnected", ip)
            await self._broadcast_status()

    # --- Hot-plug callbacks ---

    async def _on_device_added(self, port: str) -> None:
        from dog.discover import detect_serial_dog
        try:
            transport, label = await detect_serial_dog(port)
        except Exception as e:
            logger.warning("hot-plug: probe failed for %s: %s", port, e)
            return
        current = transport_priority(self._label)
        new_pri = transport_priority(label)
        if new_pri > current:
            logger.info("hot-plug: switching to %s (pri %d > %d)", label, new_pri, current)
            await self.replace(transport, label)
        else:
            logger.info("hot-plug: ignoring %s (pri %d <= %d)", label, new_pri, current)

    async def _on_device_removed(self, port: str) -> None:
        if port not in self._label:
            return
        logger.warning("hot-plug: active device removed: %s", port)
        if self._detected_wifi and self._detected_wifi.get("ip"):
            from dog import Dog
            ip = self._detected_wifi["ip"]
            tcp = self._detected_wifi.get("port", 9000)
            try:
                transport = Dog(host=ip, tcp_port=tcp)
                await self.replace(transport, f"fw-wifi:{ip}")
                return
            except Exception as e:
                logger.warning("hot-plug: WiFi fallback failed: %s", e)
        logger.warning("hot-plug: no fallback — disconnected")
        await self._broadcast_status()
