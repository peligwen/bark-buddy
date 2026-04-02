#!/usr/bin/env python3
"""Set up WiFi + WebREPL on MechDog. Run with USB cable connected."""

import serial
import time
import sys
import glob

def find_port():
    ports = glob.glob("/dev/cu.usbserial*")
    if not ports:
        print("No USB serial device found. Is the dog plugged in?")
        sys.exit(1)
    if len(ports) == 1:
        return ports[0]
    print("Multiple serial ports found:")
    for i, p in enumerate(ports):
        print(f"  {i+1}) {p}")
    choice = input("Pick one: ").strip()
    return ports[int(choice) - 1]

def send(ser, cmd, delay=2):
    ser.write((cmd + "\r\n").encode())
    time.sleep(delay)
    return ser.read(ser.in_waiting).decode(errors="replace")

def main():
    port = find_port()
    print(f"Using: {port}")

    ssid = input("WiFi SSID: ").strip()
    password = input("WiFi Password: ").strip()

    ser = serial.Serial(port, 115200, timeout=3)
    time.sleep(2)
    ser.write(b"\x03\x03\r\n")
    time.sleep(0.5)
    ser.read(ser.in_waiting)

    print("Connecting to WiFi...")
    send(ser, "import network")
    send(ser, "sta = network.WLAN(network.STA_IF)")
    send(ser, "sta.active(True)")
    send(ser, f'sta.connect("{ssid}", "{password}")', delay=8)

    resp = send(ser, "print(sta.isconnected(), sta.ifconfig())")
    if "True" not in resp:
        print("WiFi connection failed. Check SSID/password.")
        print(resp)
        ser.close()
        sys.exit(1)

    # Extract IP
    ip = "unknown"
    if "(" in resp:
        try:
            ip = resp.split("'")[1]
        except IndexError:
            pass
    print(f"WiFi connected! IP: {ip}")

    print("Writing WebREPL config...")
    send(ser, 'f = open("webrepl_cfg.py", "w")', delay=0.5)
    send(ser, 'f.write("PASS = \\"bark\\"\\n")', delay=0.5)
    send(ser, "f.close()", delay=0.5)

    print("Starting WebREPL...")
    send(ser, "import webrepl")
    resp = send(ser, "webrepl.start()")
    if "WebREPL server started" in resp or "already started" in resp.lower():
        print(f"WebREPL running at ws://{ip}:8266/")
    else:
        print("WebREPL start output:", resp)

    print()
    print(f"  Server command (WiFi):  python3 host/server.py --wifi {ip}")
    print(f"  Server command (USB):   python3 host/server.py --serial {port}")
    print()
    print("You can now unplug the USB cable.")

    ser.close()

if __name__ == "__main__":
    main()
