#!/usr/bin/env python3
"""
Dexter Gesture Receiver — Intuitive Mapping
============================================

IMPORTANT: An MPU6050 measures ROTATION ONLY, not position.
It cannot detect "hand moved forward in space". But it can detect
how your wrist/forearm is oriented, which we map intuitively:

  ┌─────────────────────────────────────────────────────────────┐
  │  Hand motion                   → Robot motion               │
  ├─────────────────────────────────────────────────────────────┤
  │  Rotate forearm left/right     → Base rotates (joint_1)     │
  │  (like turning a doorknob)     │                             │
  ├─────────────────────────────────────────────────────────────┤
  │  Tilt wrist down (point fwd)   → Arm reaches FORWARD        │
  │  Tilt wrist up   (point up)    → Arm lifts UP               │
  │  (pitch drives shoulder+elbow  → natural "reach" motion)    │
  ├─────────────────────────────────────────────────────────────┤
  │  Roll wrist side to side       → Elbow fine-tune (joint_3)  │
  ├─────────────────────────────────────────────────────────────┤
  │  Bend fingers (flex sensor)    → Gripper closes/opens       │
  └─────────────────────────────────────────────────────────────┘

The "reach forward" effect is achieved by coupling pitch to BOTH
joint_2 (shoulder) and joint_3 (elbow) simultaneously with a
tuned ratio, which causes the arm to extend/retract along a
roughly horizontal line — exactly like moving forward.

UDP packet from ESP32:  "roll,pitch,yaw,flex\\n"
"""

import math
import socket
import threading
import time

import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


# ──────────────────────────────────────────────────────────────────
# Helpers
# ──────────────────────────────────────────────────────────────────

def clamp(v, lo, hi):
    return max(lo, min(hi, v))

def deg2rad(d):
    return d * math.pi / 180.0


# ──────────────────────────────────────────────────────────────────
# Flex → gripper conversion
# ──────────────────────────────────────────────────────────────────

def flex_to_gripper(flex_raw, flex_open, flex_closed):
    """
    Map flex ADC → joint_4 angle.
      flex_open   → 0.0 rad  (gripper fully open)
      flex_closed → -π/2 rad (gripper fully closed)
    The sensor increases in resistance (and ADC) when bent.
    """
    if flex_closed <= flex_open:
        return 0.0  # miscalibrated, default open
    t = clamp((flex_raw - flex_open) / (flex_closed - flex_open), 0.0, 1.0)
    # t=0 → open (0.0 rad), t=1 → closed (-π/2 rad)
    return t * (-math.pi / 2.0)


# ──────────────────────────────────────────────────────────────────
# Coupling function: pitch → shoulder + elbow ("reach" mode)
# ──────────────────────────────────────────────────────────────────

def pitch_to_reach(pitch_deg, shoulder_gain=1.0, elbow_gain=0.6):
    """
    Map a single wrist pitch angle to both shoulder and elbow
    so the arm extends/retracts in a roughly horizontal arc.

    pitch_deg > 0  (wrist tilted down / pointing forward)
      → shoulder goes forward + elbow extends → arm reaches out
    pitch_deg < 0  (wrist tilted up)
      → shoulder lifts + elbow folds → arm rises / retracts

    shoulder_gain and elbow_gain tune how much each joint contributes.
    These are good starting defaults; adjust if arm doesn't track well.
    """
    shoulder_rad = clamp(deg2rad(pitch_deg) * shoulder_gain,
                         -math.pi / 2, math.pi / 2)
    elbow_rad    = clamp(deg2rad(pitch_deg) * elbow_gain,
                         -math.pi / 2, math.pi / 2)
    return shoulder_rad, elbow_rad


# ──────────────────────────────────────────────────────────────────
# Node
# ──────────────────────────────────────────────────────────────────

class GestureReceiver(Node):

    JOINT_LIMITS = {
        'joint_1': (-math.pi / 2, math.pi / 2),
        'joint_2': (-math.pi / 2, math.pi / 2),
        'joint_3': (-math.pi / 2, math.pi / 2),
        'joint_4': (-math.pi / 2, 0.0),
    }

    def __init__(self):
        super().__init__('gesture_receiver')

        # ── Parameters ───────────────────────────────────────────
        self.declare_parameter('udp_port', 5005)
        self.declare_parameter('smoothing_alpha', 0.20)
        self.declare_parameter('deadband_deg', 0.5)
        self.declare_parameter('trajectory_duration_ms', 80)

        # Axis inversion — flip if arm moves opposite to your hand
        self.declare_parameter('invert_roll',  False)
        self.declare_parameter('invert_pitch', False)
        self.declare_parameter('invert_yaw',   False)

        # Flex sensor calibration (update after running ESP32 Serial Monitor)
        self.declare_parameter('flex_open_adc',   2900.0)
        self.declare_parameter('flex_closed_adc', 3700.0)

        # Coupling gains for the "reach forward" mapping
        # shoulder_gain: how much wrist pitch drives the shoulder
        # elbow_gain:    how much wrist pitch also drives the elbow
        # Together they produce the extend/retract arc.
        self.declare_parameter('shoulder_gain', 1.0)
        self.declare_parameter('elbow_gain',    0.6)

        self._udp_port   = self.get_parameter('udp_port').value
        self._alpha      = self.get_parameter('smoothing_alpha').value
        self._deadband   = deg2rad(self.get_parameter('deadband_deg').value)
        self._traj_ms    = self.get_parameter('trajectory_duration_ms').value
        self._inv_roll   = self.get_parameter('invert_roll').value
        self._inv_pitch  = self.get_parameter('invert_pitch').value
        self._inv_yaw    = self.get_parameter('invert_yaw').value
        self._flex_open  = self.get_parameter('flex_open_adc').value
        self._flex_closed= self.get_parameter('flex_closed_adc').value
        self._sh_gain    = self.get_parameter('shoulder_gain').value
        self._el_gain    = self.get_parameter('elbow_gain').value

        # ── Publishers ────────────────────────────────────────────
        self._arm_pub = self.create_publisher(
            JointTrajectory, 'arm_controller/joint_trajectory', 10)
        self._grip_pub = self.create_publisher(
            JointTrajectory, 'gripper_controller/joint_trajectory', 10)

        # ── Smoothed state (radians) ──────────────────────────────
        self._lock = threading.Lock()
        # [j1_base, j2_shoulder, j3_elbow]
        self._arm   = [0.0, 0.0, 0.0]
        self._grip  = 0.0
        self._prev_arm  = None
        self._prev_grip = None

        self._last_pkt  = 0.0
        self._pkt_count = 0

        # ── UDP socket ────────────────────────────────────────────
        self._running = True
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.settimeout(1.0)
        try:
            self._sock.bind(('0.0.0.0', self._udp_port))
        except OSError as e:
            self.get_logger().fatal(f'Cannot bind UDP port {self._udp_port}: {e}')
            raise

        threading.Thread(target=self._udp_loop, daemon=True,
                         name='gesture_udp').start()

        self.create_timer(3.0, self._watchdog)

        self.get_logger().info('=' * 55)
        self.get_logger().info(f'Gesture receiver listening on UDP port {self._udp_port}')
        self.get_logger().info('Hand → Joint mapping:')
        self.get_logger().info('  Rotate forearm L/R  → Base    (joint_1)')
        self.get_logger().info('  Tilt wrist fwd/back → Reach   (joint_2 + joint_3 coupled)')
        self.get_logger().info('  Roll wrist side      → Elbow fine-tune (additive)')
        self.get_logger().info('  Bend fingers (flex)  → Gripper (joint_4)')
        self.get_logger().info(f'  flex_open={self._flex_open:.0f}  '
                               f'flex_closed={self._flex_closed:.0f}')
        self.get_logger().info('=' * 55)

    # ── UDP receive loop ─────────────────────────────────────────
    def _udp_loop(self):
        while self._running:
            try:
                data, _ = self._sock.recvfrom(256)
            except socket.timeout:
                continue
            except OSError:
                break
            try:
                self._handle(data.decode('utf-8', errors='ignore').strip())
            except Exception as e:
                self.get_logger().warn(f'Packet error: {e}',
                                       throttle_duration_sec=5.0)

    # ── Parse and map one packet ─────────────────────────────────
    def _handle(self, raw: str):
        parts = raw.split(',')
        if len(parts) < 4:
            return
        try:
            roll  = float(parts[0])
            pitch = float(parts[1])
            yaw   = float(parts[2])
            flex  = float(parts[3])
        except ValueError:
            return

        # Optional axis flips
        if self._inv_roll:  roll  = -roll
        if self._inv_pitch: pitch = -pitch
        if self._inv_yaw:   yaw   = -yaw

        # Clamp to sensor range
        roll  = clamp(roll,  -90.0, 90.0)
        pitch = clamp(pitch, -90.0, 90.0)
        yaw   = clamp(yaw,   -90.0, 90.0)

        # ── JOINT MAPPING ────────────────────────────────────────
        #
        # joint_1 (BASE ROTATION):
        #   Driven by YAW — forearm twist left/right.
        #   This IS the rotate-from-base motion you asked about.
        #
        j1 = clamp(deg2rad(yaw), *self.JOINT_LIMITS['joint_1'])

        #
        # joint_2 (SHOULDER) + joint_3 (ELBOW) — "REACH" coupling:
        #   Wrist PITCH drives both joints together.
        #   This creates a coordinated arc that extends the arm
        #   forward/backward — the closest thing to "move forward"
        #   you can get from an orientation-only sensor.
        #
        #   ROLL adds a fine-tune offset on top of the elbow only,
        #   giving you independent elbow control if needed.
        #
        j2_reach, j3_reach = pitch_to_reach(pitch, self._sh_gain, self._el_gain)
        j3_roll_offset = clamp(deg2rad(roll) * 0.4, -math.pi/4, math.pi/4)

        j2 = clamp(j2_reach, *self.JOINT_LIMITS['joint_2'])
        j3 = clamp(j3_reach + j3_roll_offset, *self.JOINT_LIMITS['joint_3'])

        # ── GRIPPER (flex sensor) ─────────────────────────────────
        g = flex_to_gripper(flex, self._flex_open, self._flex_closed)
        g = clamp(g, *self.JOINT_LIMITS['joint_4'])

        # ── Exponential smoothing (low-pass) ─────────────────────
        a = self._alpha
        with self._lock:
            self._arm[0] += a * (j1 - self._arm[0])
            self._arm[1] += a * (j2 - self._arm[1])
            self._arm[2] += a * (j3 - self._arm[2])
            self._grip   += a * (g  - self._grip)
            arm  = list(self._arm)
            grip = self._grip

        self._last_pkt  = time.monotonic()
        self._pkt_count += 1

        # ── Deadband + publish ────────────────────────────────────
        arm_moved = (
            self._prev_arm is None or
            any(abs(arm[i] - self._prev_arm[i]) > self._deadband for i in range(3))
        )
        grip_moved = (
            self._prev_grip is None or
            abs(grip - self._prev_grip) > self._deadband
        )

        if arm_moved:
            self._pub_arm(arm)
            self._prev_arm = arm[:]
        if grip_moved:
            self._pub_grip(grip)
            self._prev_grip = grip

    # ── Trajectory publishers ────────────────────────────────────
    def _traj_point(self, positions):
        pt = JointTrajectoryPoint()
        pt.positions = [float(p) for p in positions]
        ms = self._traj_ms
        pt.time_from_start = Duration(sec=ms // 1000,
                                      nanosec=(ms % 1000) * 1_000_000)
        return pt

    def _pub_arm(self, joints):
        msg = JointTrajectory()
        msg.joint_names = ['joint_1', 'joint_2', 'joint_3']
        msg.points.append(self._traj_point(joints))
        self._arm_pub.publish(msg)

    def _pub_grip(self, pos):
        msg = JointTrajectory()
        msg.joint_names = ['joint_4']
        msg.points.append(self._traj_point([pos]))
        self._grip_pub.publish(msg)

    # ── Watchdog ──────────────────────────────────────────────────
    def _watchdog(self):
        if self._last_pkt == 0.0:
            self.get_logger().warn(
                'No UDP packets yet — is ESP32 powered and on the same WiFi?',
                throttle_duration_sec=10.0)
        elif time.monotonic() - self._last_pkt > 3.0:
            self.get_logger().warn(
                f'No packet for {time.monotonic()-self._last_pkt:.0f}s',
                throttle_duration_sec=5.0)

    # ── Cleanup ───────────────────────────────────────────────────
    def destroy_node(self):
        self._running = False
        try:
            self._sock.close()
        except Exception:
            pass
        super().destroy_node()


# ──────────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    try:
        node = GestureReceiver()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        rclpy.shutdown()


if __name__ == '__main__':
    main()
