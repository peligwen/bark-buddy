"""
Camera — pulls still JPEG frames from the vision module over HTTP.

The vision module is a separate WiFi peer running its own firmware
(`camera/`). The host only reads from it; no commands flow this way, so
`Dog.send_json()` stays the single host→firmware command path.
"""

import aiohttp

_SOI = b"\xff\xd8"
_EOI = b"\xff\xd9"
_MAX_FRAME_BYTES = 4 * 1024 * 1024

# A stream's first frame may have been sitting in the sensor's buffer since
# before we connected, so it can show the dog still moving.
_MJPEG_STALE_FRAMES = 1


class CameraError(Exception):
    """The camera did not return a usable frame."""


class Camera:
    """
    Usage:
        cam = Camera("http://bark-cam.local/capture")
        await cam.open()
        jpeg = await cam.grab()
        await cam.close()

    The URL may serve a single JPEG per request or a multipart MJPEG stream;
    `grab()` returns one frame either way.
    """

    def __init__(self, url: str, timeout: float = 5.0):
        self.url = url
        self._timeout = aiohttp.ClientTimeout(total=timeout)
        self._session: aiohttp.ClientSession | None = None

    async def open(self) -> None:
        self._session = aiohttp.ClientSession(timeout=self._timeout)

    async def close(self) -> None:
        if self._session is not None:
            await self._session.close()
            self._session = None

    async def grab(self) -> bytes:
        if self._session is None:
            raise CameraError("camera not open")
        try:
            async with self._session.get(self.url) as resp:
                resp.raise_for_status()
                if resp.content_type.startswith("multipart/"):
                    return await _read_mjpeg_frame(resp.content, _MJPEG_STALE_FRAMES)
                data = await resp.read()
        except (TimeoutError, aiohttp.ClientError) as e:
            raise CameraError(f"{self.url}: {e or type(e).__name__}") from e
        if not (data.startswith(_SOI) and data.rstrip(b"\r\n").endswith(_EOI)):
            raise CameraError(f"{self.url}: response is not a JPEG")
        return data


async def _read_mjpeg_frame(stream: aiohttp.StreamReader, skip: int) -> bytes:
    """
    Return the frame after `skip` complete frames.

    Scans for JPEG start/end markers instead of parsing multipart boundaries:
    camera firmwares disagree on boundary syntax, but entropy-coded JPEG data
    never contains an unescaped end marker.
    """
    buf = bytearray()
    async for chunk in stream.iter_any():
        buf += chunk
        while True:
            start = buf.find(_SOI)
            if start < 0:
                del buf[:-1]  # keep one byte: a marker may straddle chunks
                break
            end = buf.find(_EOI, start + 2)
            if end < 0:
                del buf[:start]
                break
            frame = bytes(buf[start:end + 2])
            del buf[:end + 2]
            if skip == 0:
                return frame
            skip -= 1
        if len(buf) > _MAX_FRAME_BYTES:
            raise CameraError("stream has no JPEG frame boundary")
    raise CameraError("stream ended before a complete frame")
