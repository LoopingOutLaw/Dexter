#!/usr/bin/env python3
"""
udp_test_sender.py
==================
Run this on your PC to simulate the ESP32 sending gesture data.
Use this to test the gesture_receiver node WITHOUT the physical hardware.

Usage:
    python3 udp_test_sender.py              # interactive sine-wave demo
    python3 udp_test_sender.py --static     # send zeros (arm holds position)
    python3 udp_test_sender.py --manual     # type values manually

The script sends UDP packets to 127.0.0.1:5005 (same machine).
If your gesture_receiver is on a different machine, edit TARGET_IP below.
"""

import argparse
import math
import socket
import time

TARGET_IP   = '127.0.0.1'
TARGET_PORT = 5005
RATE_HZ     = 50   # match ESP32 rate


def send_packet(sock, roll, pitch, yaw, flex=1800):
    """Send one IMU packet."""
    msg = f'{roll:.2f},{pitch:.2f},{yaw:.2f},{flex}\n'
    sock.sendto(msg.encode(), (TARGET_IP, TARGET_PORT))


def mode_sine(sock):
    """Smoothly sweep all three axes through their full range."""
    print('Sine-wave mode — press Ctrl+C to stop')
    print(f'Sending to {TARGET_IP}:{TARGET_PORT} at {RATE_HZ} Hz\n')
    t = 0.0
    dt = 1.0 / RATE_HZ
    while True:
        roll  =  45.0 * math.sin(2 * math.pi * 0.2 * t)        # 0.2 Hz
        pitch =  45.0 * math.sin(2 * math.pi * 0.15 * t + 1.0) # 0.15 Hz
        yaw   =  45.0 * math.sin(2 * math.pi * 0.1  * t + 2.0) # 0.1 Hz
        flex  = 1800 + int(700 * (0.5 + 0.5 * math.sin(2 * math.pi * 0.05 * t)))
        send_packet(sock, roll, pitch, yaw, flex)
        print(f'\r  roll={roll:+6.1f}°  pitch={pitch:+6.1f}°  yaw={yaw:+6.1f}°  flex={flex}    ',
              end='', flush=True)
        t += dt
        time.sleep(dt)


def mode_static(sock):
    """Send zeros — arm should hold the home position."""
    print('Static mode (zeros) — press Ctrl+C to stop')
    print(f'Sending to {TARGET_IP}:{TARGET_PORT} at {RATE_HZ} Hz\n')
    while True:
        send_packet(sock, 0, 0, 0, 1800)
        time.sleep(1.0 / RATE_HZ)


def mode_manual(sock):
    """Let you type roll/pitch/yaw values manually."""
    print('Manual mode — type: roll pitch yaw  (degrees, separated by spaces)')
    print('Example: 30 -20 10')
    print(f'Sending to {TARGET_IP}:{TARGET_PORT}\n')
    while True:
        try:
            line = input('> ').strip()
            parts = line.split()
            if len(parts) < 3:
                print('  Need 3 values: roll pitch yaw')
                continue
            roll, pitch, yaw = float(parts[0]), float(parts[1]), float(parts[2])
            flex = int(parts[3]) if len(parts) >= 4 else 1800
            send_packet(sock, roll, pitch, yaw, flex)
            print(f'  Sent: roll={roll}°  pitch={pitch}°  yaw={yaw}°  flex={flex}')
        except (ValueError, EOFError):
            print('  Invalid input')


def main():
    parser = argparse.ArgumentParser(description='Dexter ESP32 UDP simulator')
    group = parser.add_mutually_exclusive_group()
    group.add_argument('--sine',   action='store_true', default=True,
                       help='Sine-wave sweep (default)')
    group.add_argument('--static', action='store_true',
                       help='Send zeros (hold position)')
    group.add_argument('--manual', action='store_true',
                       help='Type values interactively')
    parser.add_argument('--ip',   default=TARGET_IP,   help='Target IP')
    parser.add_argument('--port', default=TARGET_PORT, type=int, help='Target port')
    args = parser.parse_args()

    global TARGET_IP, TARGET_PORT
    TARGET_IP   = args.ip
    TARGET_PORT = args.port

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        if args.static:
            mode_static(sock)
        elif args.manual:
            mode_manual(sock)
        else:
            mode_sine(sock)
    except KeyboardInterrupt:
        print('\nStopped.')
    finally:
        sock.close()


if __name__ == '__main__':
    main()
