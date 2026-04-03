#!/usr/bin/env python3
"""
Stock firmware GPIO pin monitor for MechDog.

Runs on the stock MicroPython firmware. Captures GPIO output register
state at high frequency during servo commands, revealing:
- Which GPIOs are actually toggling (pulse widths)
- The timing relationship between servo channels
- How the stock PWMServo C module drives each pin

Usage:
    python3 host/monitor_pins.py [--port /dev/cu.usbserial-10]
"""

import argparse
import json
import serial
import sys
import time


SERVO_PINS = [14, 13, 16, 17, 18, 8, 5, 7]
LABELS = ['FL_hip', 'FL_knee', 'FR_hip', 'FR_knee',
          'RL_hip', 'RL_knee', 'RR_hip', 'RR_knee']
STANDING = [2096, 1621, 2170, 1611, 904, 1389, 1379, 830]


def send(s, cmd, delay=0.5):
    s.write((cmd + '\r\n').encode())
    time.sleep(delay)
    resp = s.read(s.in_waiting).decode(errors='replace')
    clean = [l.strip() for l in resp.strip().split('\r\n')
             if l.strip() and l.strip() != '>>>' and l.strip() != cmd
             and not l.strip().startswith('...')]
    return ' | '.join(clean) if clean else ''


def capture_gpio_snapshot(s, duration_ms=40, sample_interval_us=100):
    """
    Capture GPIO output register at high frequency on the ESP32.
    Returns list of (timestamp_us, gpio_out_value) tuples.

    Uses machine.mem32 to read GPIO_OUT_REG (0x3FF44004) directly.
    Runs a tight MicroPython loop — limited by REPL speed (~5-20 kHz).
    """
    # Upload a capture script to the REPL
    num_samples = (duration_ms * 1000) // sample_interval_us
    # Limit to avoid memory issues
    num_samples = min(num_samples, 500)

    script = (
        f"import machine, time\n"
        f"_r = [0]*{num_samples}\n"
        f"_t = [0]*{num_samples}\n"
        f"_m = machine.mem32\n"
        f"_us = time.ticks_us\n"
        f"_t0 = _us()\n"
        f"for _i in range({num_samples}):\n"
        f" _t[_i] = time.ticks_diff(_us(), _t0)\n"
        f" _r[_i] = _m[0x3FF44004]\n"  # GPIO_OUT_REG
    )
    # Send as exec to avoid multi-line REPL issues
    exec_cmd = "exec('" + script.replace('\n', '\\n').replace("'", "\\'") + "')"
    send(s, exec_cmd, delay=0.5)

    # Read results — output as compact format
    send(s, "print(len(_r))", 0.3)
    result = send(s, "print(list(zip(_t, _r)))", 2.0)
    return result


def analyze_pin_pulses(capture_data, pin):
    """Extract pulse timing for a specific GPIO pin from capture data."""
    mask = 1 << pin
    transitions = []
    prev_state = None
    for t_us, gpio_val in capture_data:
        state = bool(gpio_val & mask)
        if prev_state is not None and state != prev_state:
            transitions.append((t_us, 'rise' if state else 'fall'))
        prev_state = state
    return transitions


def run_monitor(port):
    s = serial.Serial(port, 115200, timeout=3)
    time.sleep(2)
    s.write(b'\x03\x03\r\n')
    time.sleep(1)
    s.read(s.in_waiting)

    # Init
    send(s, 'from HW_MechDog import MechDog')
    send(s, 'd = MechDog()')
    send(s, 'd.set_default_pose()', 1.5)
    time.sleep(1)

    print("=== Stock Firmware GPIO Pin Monitor ===\n")

    # Test 1: Read standing servo positions
    print("Standing servo positions:")
    positions = send(s, 'print(d.read_all_servo())')
    print(f"  {positions}")

    # Test 2: Check which GPIOs have active output by reading GPIO_ENABLE_REG
    enable_reg = send(s, 'import machine; print(hex(machine.mem32[0x3FF44020]))')
    print(f"\nGPIO enable register: {enable_reg}")
    print("  Active output pins:", end=" ")
    try:
        val = int(enable_reg.strip(), 16)
        for pin in SERVO_PINS:
            if val & (1 << pin):
                print(f"GPIO{pin}", end=" ")
        print()
    except ValueError:
        print(f"(parse error: {enable_reg})")

    # Test 3: Read GPIO output state (snapshot)
    out_reg = send(s, 'print(hex(machine.mem32[0x3FF44004]))')
    print(f"\nGPIO output register: {out_reg}")

    # Test 4: Capture GPIO transitions during one PWM period
    print("\nCapturing GPIO transitions during servo PWM (40ms = 2 periods)...")
    # Run a tight sampling loop on the ESP32
    script = """
import machine, time
mem = machine.mem32
us = time.ticks_us
diff = time.ticks_diff
pins = [14, 13, 16, 17, 18, 8, 5, 7]
masks = [(1 << p) for p in pins]
# Sample 500 times over ~40ms
N = 500
reg = 0x3FF44004
times = [0]*N
vals = [0]*N
t0 = us()
for i in range(N):
    times[i] = diff(us(), t0)
    vals[i] = mem[reg]
# Extract per-pin pulse widths
for pi in range(8):
    m = masks[pi]
    rises = []
    falls = []
    prev = vals[0] & m
    for i in range(1, N):
        cur = vals[i] & m
        if cur and not prev:
            rises.append(times[i])
        elif not cur and prev:
            falls.append(times[i])
        prev = cur
    if rises and falls:
        pw = falls[0] - rises[0] if falls[0] > rises[0] else 0
        print("PIN", pins[pi], "pulse_us=", pw, "rises=", len(rises), "falls=", len(falls))
    elif falls and not rises:
        print("PIN", pins[pi], "HIGH_at_start falls=", len(falls), "first_fall=", falls[0])
    else:
        state = "HIGH" if vals[0] & m else "LOW"
        print("PIN", pins[pi], "static=", state)
"""
    exec_cmd = "exec('" + script.replace('\n', '\\n').replace("'", "\\'") + "')"
    result = send(s, exec_cmd, delay=3.0)
    print()
    for line in result.split(' | '):
        line = line.strip()
        if line.startswith('PIN'):
            print(f"  {line}")

    # Test 5: Move one servo and capture to see the change
    print("\nCapturing after moving servo 0 (FL hip) +200us...")
    send(s, 'd.set_servo(1, 2296, 100)')
    time.sleep(0.5)

    script2 = """
import machine, time
mem = machine.mem32
us = time.ticks_us
diff = time.ticks_diff
N = 500
reg = 0x3FF44004
times = [0]*N
vals = [0]*N
t0 = us()
for i in range(N):
    times[i] = diff(us(), t0)
    vals[i] = mem[reg]
m = 1 << 14
rises = []
falls = []
prev = vals[0] & m
for i in range(1, N):
    cur = vals[i] & m
    if cur and not prev:
        rises.append(times[i])
    elif not cur and prev:
        falls.append(times[i])
    prev = cur
if rises and falls:
    pw = falls[0] - rises[0] if falls[0] > rises[0] else 0
    print("GPIO14 pulse_us=", pw)
elif falls:
    print("GPIO14 HIGH_at_start first_fall=", falls[0])
else:
    state = "HIGH" if vals[0] & m else "LOW"
    print("GPIO14 static=", state)
"""
    exec_cmd2 = "exec('" + script2.replace('\n', '\\n').replace("'", "\\'") + "')"
    result2 = send(s, exec_cmd2, delay=3.0)
    for line in result2.split(' | '):
        if 'GPIO14' in line:
            print(f"  {line.strip()}")

    # Return to standing
    send(s, 'd.set_default_pose()', 1.0)

    # Test 6: Capture all 8 pin pulse widths in a single profile
    print("\n=== Final Profile: All 8 Servo Pulse Widths ===")
    print(f"{'Servo':8s} {'GPIO':5s} {'Label':10s} {'StandingUS':10s} {'MeasuredUS':10s} {'Match':6s}")
    print("-" * 52)

    profile_script = """
import machine, time
mem = machine.mem32
us = time.ticks_us
diff = time.ticks_diff
pins = [14, 13, 16, 17, 18, 8, 5, 7]
N = 800
reg = 0x3FF44004
t = [0]*N
v = [0]*N
t0 = us()
for i in range(N):
    t[i] = diff(us(), t0)
    v[i] = mem[reg]
for pi in range(8):
    m = 1 << pins[pi]
    rises = []
    falls = []
    prev = v[0] & m
    for i in range(1, N):
        cur = v[i] & m
        if cur and not prev:
            rises.append(t[i])
        elif not cur and prev:
            falls.append(t[i])
        prev = cur
    if rises and falls and falls[0] > rises[0]:
        pw = falls[0] - rises[0]
    elif falls and not rises:
        pw = falls[0]
    else:
        pw = -1
    print(pins[pi], pw)
"""
    exec_cmd3 = "exec('" + profile_script.replace('\n', '\\n').replace("'", "\\'") + "')"
    result3 = send(s, exec_cmd3, delay=4.0)

    measured = {}
    for line in result3.split(' | '):
        line = line.strip()
        parts = line.split()
        if len(parts) == 2:
            try:
                pin = int(parts[0])
                pw = int(parts[1])
                measured[pin] = pw
            except ValueError:
                pass

    for i in range(8):
        pin = SERVO_PINS[i]
        expected = STANDING[i]
        actual = measured.get(pin, -1)
        match = 'OK' if actual > 0 and abs(actual - expected) < 50 else 'MISS' if actual > 0 else 'NONE'
        print(f"{i:8d} {pin:5d} {LABELS[i]:10s} {expected:8d}   {actual:8d}   {match}")

    s.close()
    return measured


def main():
    parser = argparse.ArgumentParser(description="MechDog stock firmware pin monitor")
    parser.add_argument("--port", default="/dev/cu.usbserial-10")
    args = parser.parse_args()

    run_monitor(args.port)


if __name__ == "__main__":
    main()
