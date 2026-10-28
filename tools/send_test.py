#!/usr/bin/env python3
"""Sends made-up SimRacingStudio v102 "api" packets at 60 Hz, to try the visualizer without a
game: a slow swell on pitch and roll, turns that slide, and a bump every few seconds.

    tools/send_test.py [host:port]     (default 127.0.0.1:33001)
"""
import math
import socket
import struct
import sys
import time

FORMAT = "3sI50s50s50sfffifffffffffffIIII"   # native alignment: 236 bytes

host, _, port = (sys.argv[1] if len(sys.argv) > 1 else "127.0.0.1:33001").rpartition(":")
dest = (host or "127.0.0.1", int(port))
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
start = time.monotonic()
print(f"sending test packets to {dest[0]}:{dest[1]}, ^C to stop")
while True:
    t = time.monotonic() - start
    speed = 80 + 20 * math.sin(t * 0.3)
    pitch = 5 + 6 * math.sin(t * 1.1)
    roll = 15 * math.sin(t * 0.45)
    yaw = 8 * math.sin(t * 0.45 - 0.6)
    sway = 12 * math.sin(t * 0.45)
    surge = 4 * math.cos(t * 0.3)
    bump = 25 * math.exp(-((t % 4.0) - 0.2) ** 2 / 0.002)
    heave = 2 * math.sin(t * 2.3) + bump
    rpm = 1500 + speed / 120 * 5500
    packet = struct.pack(FORMAT, b"api", 102, b"Test pattern", b"", b"",
                         speed, rpm, 9000, 1, pitch, roll, yaw, -sway / 5, sway, heave, surge,
                         0, 0, 0, 0, 0, 0, 0, 0)
    sock.sendto(packet, dest)
    time.sleep(1 / 60)
