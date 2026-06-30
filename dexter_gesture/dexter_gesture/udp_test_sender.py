#!/usr/bin/env python3
# Copyright 2026 aditya
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""
UDP test sender — simulate the ESP32 glove on the PC.

Run this on your PC to send fake IMU packets to the gesture receiver
without the physical ESP32 glove.  Useful when developing the ROS node
or testing against `udp_test_sender.py` in CI.

Usage
-----

    python3 udp_test_sender.py              # sine-wave demo (default)
    python3 udp_test_sender.py --static     # send zeros (arm holds position)
    python3 udp_test_sender.py --manual     # type values interactively
    python3 udp_test_sender.py --ip 10.0.0.5 --port 5005

The script sends UDP packets to ``127.0.0.1:5005`` by default (same
machine).  If your gesture_receiver runs on a different machine,
pass ``--ip`` and ``--port`` to point at it.
"""

import argparse
import math
import socket
import time

DEFAULT_IP = '127.0.0.1'
DEFAULT_PORT = 5005
DEFAULT_RATE_HZ = 50   # match ESP32 burst rate


def send_packet(sock, ip, port, roll, pitch, yaw, flex=1800):
    r"""Send one IMU packet of the form ``roll,pitch,yaw,flex\n``."""
    msg = f'{roll:.2f},{pitch:.2f},{yaw:.2f},{flex}\n'
    sock.sendto(msg.encode(), (ip, port))


def _send_loop(sock, ip, port, builder, rate_hz):
    """Drive the ``builder(now) -> (roll, pitch, yaw, flex)`` at ``rate_hz``."""
    dt = 1.0 / rate_hz
    t = 0.0
    while True:
        roll, pitch, yaw, flex = builder(t)
        send_packet(sock, ip, port, roll, pitch, yaw, flex)
        print(
            f'\r  roll={roll:+6.1f}°  pitch={pitch:+6.1f}°  '
            f'yaw={yaw:+6.1f}°  flex={flex}    ',
            end='', flush=True,
        )
        t += dt
        time.sleep(dt)


def mode_sine(sock, ip, port, rate_hz=DEFAULT_RATE_HZ):
    """Smoothly sweep all three axes through their full range."""

    def builder(t):
        return (
            45.0 * math.sin(2 * math.pi * 0.20 * t),                 # roll  0.20 Hz
            45.0 * math.sin(2 * math.pi * 0.15 * t + 1.0),           # pitch 0.15 Hz
            45.0 * math.sin(2 * math.pi * 0.10 * t + 2.0),           # yaw   0.10 Hz
            1800 + int(700 * (0.5 + 0.5 * math.sin(2 * math.pi * 0.05 * t))),
        )

    print('Sine-wave mode — press Ctrl+C to stop')
    print(f'Sending to {ip}:{port} at {rate_hz} Hz\n')
    _send_loop(sock, ip, port, builder, rate_hz)


def mode_static(sock, ip, port, rate_hz=DEFAULT_RATE_HZ):
    """Send zeros — arm should hold the home position."""
    print('Static mode (zeros) — press Ctrl+C to stop')
    print(f'Sending to {ip}:{port} at {rate_hz} Hz\n')

    def builder(_t):
        return 0.0, 0.0, 0.0, 1800

    _send_loop(sock, ip, port, builder, rate_hz)


def mode_manual(sock, ip, port, _rate_hz=DEFAULT_RATE_HZ):
    """Let you type roll/pitch/yaw values manually."""
    print('Manual mode — type: roll pitch yaw  (degrees, separated by spaces)')
    print('Example: 30 -20 10                   (4th value: flex ADC)')
    print(f'Sending to {ip}:{port}\n')
    while True:
        try:
            line = input('> ').strip()
        except EOFError:
            print('\n  EOF received.')
            return
        parts = line.split()
        if len(parts) < 3:
            print('  Need at least 3 values: roll pitch yaw [flex]')
            continue
        try:
            roll, pitch, yaw = float(parts[0]), float(parts[1]), float(parts[2])
            flex = int(parts[3]) if len(parts) >= 4 else 1800
        except ValueError:
            print('  Invalid number — try again')
            continue
        send_packet(sock, ip, port, roll, pitch, yaw, flex)
        print(
            f'  Sent: roll={roll}°  pitch={pitch}°  '
            f'yaw={yaw}°  flex={flex}'
        )


def main():
    parser = argparse.ArgumentParser(
        description='Dexter ESP32 UDP simulator',
    )
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument(
        '--sine',   action='store_true',  default=True,
        help='Sine-wave sweep (default)',
    )
    mode.add_argument(
        '--static', action='store_true',
        help='Send zeros (hold position)',
    )
    mode.add_argument(
        '--manual', action='store_true',
        help='Type values interactively',
    )
    parser.add_argument(
        '--ip',      default=DEFAULT_IP,
        help=f'Target IP (default: {DEFAULT_IP})',
    )
    parser.add_argument(
        '--port',    default=DEFAULT_PORT, type=int,
        help=f'Target port (default: {DEFAULT_PORT})',
    )
    parser.add_argument(
        '--rate',    default=DEFAULT_RATE_HZ, type=int,
        help=f'Send rate in Hz (default: {DEFAULT_RATE_HZ})',
    )
    args = parser.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        if args.static:
            mode_static(sock, args.ip, args.port, args.rate)
        elif args.manual:
            mode_manual(sock, args.ip, args.port, args.rate)
        else:
            mode_sine(sock, args.ip, args.port, args.rate)
    except KeyboardInterrupt:
        print('\nStopped.')
    finally:
        sock.close()


if __name__ == '__main__':
    main()
