"""mDNS browser for discovering MechDog robots on the local network."""
import asyncio
import logging
from collections.abc import Awaitable, Callable
from typing import Optional

logger = logging.getLogger(__name__)

SERVICE_TYPE = "_mechdog._tcp.local."


class MdnsBrowser:
    """Browse for _mechdog._tcp.local. services via zeroconf."""

    def __init__(
        self,
        on_found: Callable[[str, int, dict], Awaitable[None]],
        on_lost: Callable[[str], Awaitable[None]],
    ):
        self._on_found = on_found
        self._on_lost = on_lost
        self._zc = None
        self._browser = None

    async def start(self):
        from zeroconf import Zeroconf, ServiceBrowser, ServiceListener

        loop = asyncio.get_running_loop()
        on_found = self._on_found
        on_lost = self._on_lost

        class _Listener(ServiceListener):
            def add_service(self, zc, type_: str, name: str):
                info = zc.get_service_info(type_, name)
                if info and info.parsed_addresses():
                    ip = info.parsed_addresses()[0]
                    port = info.port
                    props = {
                        k.decode() if isinstance(k, bytes) else k:
                        v.decode() if isinstance(v, bytes) else v
                        for k, v in info.properties.items()
                    }
                    logger.info("mDNS: found %s at %s:%d props=%s", name, ip, port, props)
                    asyncio.run_coroutine_threadsafe(on_found(ip, port, props), loop)

            def remove_service(self, zc, type_: str, name: str):
                logger.info("mDNS: lost %s", name)
                asyncio.run_coroutine_threadsafe(on_lost(name), loop)

            def update_service(self, zc, type_: str, name: str):
                self.add_service(zc, type_, name)

        self._zc = Zeroconf()
        self._browser = ServiceBrowser(self._zc, SERVICE_TYPE, _Listener())

    async def stop(self):
        if self._browser:
            self._browser.cancel()
            self._browser = None
        if self._zc:
            self._zc.close()
            self._zc = None
