"""
Tests for Dog._on_raw_line — multi-JSON-per-line + edge cases.

The reader feeds raw bytes; one TCP packet may contain multiple JSON
objects glued together. The parser needs to:
- skip leading non-JSON garbage
- extract each {...} block and call _handle_telem
- not corrupt internal state on malformed JSON
"""

import asyncio
import pytest

from dog import Dog


def _new_dog():
    # No port/host — we never call open(), only feed raw lines.
    return Dog(host="127.0.0.1", tcp_port=0)


def test_single_json_object():
    dog = _new_dog()
    captured = []
    dog.set_telem_callback(lambda m: captured.append(m))
    dog._on_raw_line(b'{"type":"telem_imu","pitch":1.5}\n')
    assert captured == [{"type": "telem_imu", "pitch": 1.5}]


def test_two_objects_glued_together():
    """Two complete JSON objects in one line should both dispatch."""
    dog = _new_dog()
    captured = []
    dog.set_telem_callback(lambda m: captured.append(m))
    line = b'{"type":"telem_imu","pitch":1.0}{"type":"telem_sonar","distance_mm":250}'
    dog._on_raw_line(line)
    assert len(captured) == 2
    assert captured[0]["type"] == "telem_imu"
    assert captured[1]["type"] == "telem_sonar"


def test_garbage_prefix_then_json():
    dog = _new_dog()
    captured = []
    dog.set_telem_callback(lambda m: captured.append(m))
    # Some pre-pong terminal output before the parser is sync'd.
    dog._on_raw_line(b'noise{"type":"pong"}')
    assert captured == [{"type": "pong"}]


def test_empty_line_is_ignored():
    dog = _new_dog()
    captured = []
    dog.set_telem_callback(lambda m: captured.append(m))
    dog._on_raw_line(b'\n')
    dog._on_raw_line(b'')
    dog._on_raw_line(b'   ')
    assert captured == []


def test_malformed_json_is_dropped_quietly():
    """A bad JSON object should not crash and should not keep the parser
    in a bad state for the next line."""
    dog = _new_dog()
    captured = []
    dog.set_telem_callback(lambda m: captured.append(m))
    dog._on_raw_line(b'{"type":"telem_imu" not actually json}')
    assert captured == []
    # Next line should still parse cleanly.
    dog._on_raw_line(b'{"type":"pong"}')
    assert captured == [{"type": "pong"}]


def test_telem_status_updates_cached_state():
    dog = _new_dog()
    dog._on_raw_line(b'{"type":"telem_status","engaged":true,"ramping":false,"battery_cutoff":false}')
    assert dog.get_engaged() is True
    assert dog.get_ramping() is False
    assert dog.get_battery_cutoff() is False


def test_ack_lands_in_ack_queue():
    dog = _new_dog()
    dog._on_raw_line(b'{"type":"ack","ref_type":"cmd_engage","ok":true}')
    # Synchronous queue check — no await needed (size 1)
    assert dog._ack_queue.qsize() == 1
    msg = dog._ack_queue.get_nowait()
    assert msg["ref_type"] == "cmd_engage"


def test_ack_callback_fires():
    dog = _new_dog()
    seen = []
    dog.set_ack_callback(lambda m: seen.append(m))
    dog._on_raw_line(b'{"type":"ack","ref_type":"cmd_stand","ok":true}')
    assert len(seen) == 1
    assert seen[0]["ref_type"] == "cmd_stand"


@pytest.mark.asyncio
async def test_recv_ack_returns_matching_ref_type():
    dog = _new_dog()
    # Stuff the queue then await
    dog._on_raw_line(b'{"type":"ack","ref_type":"cmd_move","ok":false,"error":"not_engaged"}')
    msg = await dog.recv_ack("cmd_move", timeout=0.5)
    assert msg is not None
    assert msg["ref_type"] == "cmd_move"
    assert msg["ok"] is False
    assert msg["error"] == "not_engaged"


@pytest.mark.asyncio
async def test_recv_ack_times_out_when_no_match():
    dog = _new_dog()
    dog._on_raw_line(b'{"type":"ack","ref_type":"cmd_other","ok":true}')
    msg = await dog.recv_ack("cmd_move", timeout=0.05)
    # Wrong ref_type should NOT be returned (it stays in the queue, gets popped, then timeout).
    assert msg is None
