"""Tests for Camera — one JPEG out of a snapshot endpoint or an MJPEG stream."""

import pytest
from aiohttp import web
from aiohttp.test_utils import TestServer
from camera import Camera, CameraError


def _jpeg(payload: bytes) -> bytes:
    return b"\xff\xd8" + payload + b"\xff\xd9"


async def _open_camera(handler):
    app = web.Application()
    app.router.add_get("/cam", handler)
    server = TestServer(app)
    await server.start_server()
    cam = Camera(str(server.make_url("/cam")), timeout=2.0)
    await cam.open()
    return server, cam


async def _mjpeg_response(request, chunks):
    resp = web.StreamResponse(headers={
        "Content-Type": "multipart/x-mixed-replace;boundary=frame",
    })
    await resp.prepare(request)
    for chunk in chunks:
        await resp.write(chunk)
    return resp


def _part(jpeg: bytes) -> bytes:
    return (b"\r\n--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
            + str(len(jpeg)).encode() + b"\r\n\r\n" + jpeg)


@pytest.mark.asyncio
async def test_snapshot_returns_body():
    frame = _jpeg(b"snapshot")

    async def handler(request):
        return web.Response(body=frame, content_type="image/jpeg")

    server, cam = await _open_camera(handler)
    try:
        assert await cam.grab() == frame
    finally:
        await cam.close()
        await server.close()


@pytest.mark.asyncio
async def test_mjpeg_skips_stale_first_frame():
    stale, fresh = _jpeg(b"stale"), _jpeg(b"fresh")

    async def handler(request):
        return await _mjpeg_response(request, [_part(stale), _part(fresh), _part(_jpeg(b"later"))])

    server, cam = await _open_camera(handler)
    try:
        assert await cam.grab() == fresh
    finally:
        await cam.close()
        await server.close()


@pytest.mark.asyncio
async def test_mjpeg_frame_split_across_chunks():
    stale, fresh = _jpeg(b"stale"), _jpeg(b"fresh-frame-payload")
    body = _part(stale) + _part(fresh)
    cut = body.index(fresh) + 1  # split inside the fresh frame's start marker

    async def handler(request):
        return await _mjpeg_response(request, [body[:cut], body[cut:cut + 5], body[cut + 5:]])

    server, cam = await _open_camera(handler)
    try:
        assert await cam.grab() == fresh
    finally:
        await cam.close()
        await server.close()


@pytest.mark.asyncio
async def test_mjpeg_stream_ending_early_raises():
    async def handler(request):
        return await _mjpeg_response(request, [_part(_jpeg(b"only-one"))])

    server, cam = await _open_camera(handler)
    try:
        with pytest.raises(CameraError):
            await cam.grab()
    finally:
        await cam.close()
        await server.close()


@pytest.mark.asyncio
async def test_non_jpeg_body_raises():
    async def handler(request):
        return web.Response(text="<html>camera page</html>", content_type="text/html")

    server, cam = await _open_camera(handler)
    try:
        with pytest.raises(CameraError):
            await cam.grab()
    finally:
        await cam.close()
        await server.close()


@pytest.mark.asyncio
async def test_http_error_raises():
    async def handler(request):
        return web.Response(status=503)

    server, cam = await _open_camera(handler)
    try:
        with pytest.raises(CameraError):
            await cam.grab()
    finally:
        await cam.close()
        await server.close()


@pytest.mark.asyncio
async def test_grab_before_open_raises():
    cam = Camera("http://127.0.0.1:1/cam")
    with pytest.raises(CameraError):
        await cam.grab()
