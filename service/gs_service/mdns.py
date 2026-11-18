"""mDNS advertisement of ``_golfsim._tcp.local.``."""
from __future__ import annotations

import logging
import socket

log = logging.getLogger("gs_service.mdns")
SERVICE_TYPE = "_golfsim._tcp.local."


class MdnsAdvertiser:
    def __init__(self, device_id: str, version: str, port: int):
        self.device_id, self.version, self.port = device_id, version, port
        self._azc = None
        self._info = None

    async def start(self) -> None:
        try:
            from zeroconf import IPVersion, ServiceInfo, get_all_addresses
            from zeroconf.asyncio import AsyncZeroconf

            addrs = [a for a in get_all_addresses() if not a.startswith("127.")] or ["127.0.0.1"]
            self._info = ServiceInfo(
                SERVICE_TYPE,
                f"GolfSim-{self.device_id}.{SERVICE_TYPE}",
                addresses=[socket.inet_aton(a) for a in addrs],
                port=self.port,
                properties={"id": self.device_id, "ver": self.version, "api": "1"},
                server=f"golfsim-{self.device_id}.local.",
            )
            self._azc = AsyncZeroconf(ip_version=IPVersion.V4Only)
            await self._azc.async_register_service(self._info)
            log.info("mDNS advertising %s on %s:%d", SERVICE_TYPE, addrs, self.port)
        except Exception as e:  # never let discovery problems stop the service
            log.warning("mDNS advertisement failed: %s", e)
            self._azc = None

    async def stop(self) -> None:
        if self._azc:
            try:
                if self._info:
                    await self._azc.async_unregister_service(self._info)
                await self._azc.async_close()
            except Exception:
                pass
            self._azc = None
