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
Dexter Gesture Receiver.

Intuitive hand-to-joint mapping plus a 4-stage jerk-bounded smoothing
pipeline. See ``~/Dexter/docs/superpowers/specs/2026-06-15-dexter-motion
-smoothing-design.md`` for the full design rationale.

Hand motion                -> Robot motion.
------------------------------ ---------------------------------
Rotate forearm left/right  -> Base rotates (joint_1)
Tilt wrist down (point fwd) -> Arm reaches FORWARD
Tilt wrist up (point up)    -> Arm lifts UP
Roll wrist side to side     -> Elbow fine-tune (joint_3)
Bend fingers (flex sensor)  -> Gripper closes/opens

Smoothing pipeline (4 stages):
  Stage A -- Deadband:    ignore changes < 0.5 deg
  Stage B -- Low-pass:    one-pole EMA on raw sensor vector
  Stage C -- S-curve:     3rd-order jerk-bounded smoother @ 100 Hz
                          (wall-clock timer, decoupled from UDP)
  Stage D -- Trajectory:  publish only if joint moved > 0.005 rad

UDP packet format from ESP32: ``"roll,pitch,yaw,flex"`` then newline.
"""

import math
import socket
import threading
import time

from builtin_interfaces.msg import Duration
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from .motion_smoother import JointSmoother


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


def _deg2rad(d):
    return d * math.pi / 180.0


def _flex_to_gripper(flex_raw, flex_open, flex_closed):
    if flex_closed <= flex_open:
        return 0.0
    t = _clamp((flex_raw - flex_open) / (flex_closed - flex_open), 0.0, 1.0)
    return t * (-math.pi / 2.0)


def _pitch_to_reach(pitch_deg, shoulder_gain, elbow_gain):
    shoulder_rad = _clamp(_deg2rad(pitch_deg) * shoulder_gain,
                          -math.pi / 2, math.pi / 2)
    elbow_rad = _clamp(_deg2rad(pitch_deg) * elbow_gain,
                       -math.pi / 2, math.pi / 2)
    return shoulder_rad, elbow_rad


class GestureReceiver(Node):

    JOINT_LIMITS = {
        'joint_1': (-math.pi / 2, math.pi / 2),
        'joint_2': (-math.pi / 2, math.pi / 2),
        'joint_3': (-math.pi / 2, math.pi / 2),
        'joint_4': (-math.pi / 2, 0.0),
    }

    def __init__(self):
        super().__init__('gesture_receiver')

        self.declare_parameter('udp_port', 5005)
        self.declare_parameter('deadband_deg', 0.5)
        self.declare_parameter('low_pass_alpha', 0.12)
        self.declare_parameter('smoother_rate_hz', 100)
        self.declare_parameter('publish_deadband_rad', 0.005)
        self.declare_parameter('max_velocity_rad_s', 1.2)
        self.declare_parameter('max_acceleration_rad_s2', 6.0)
        self.declare_parameter('max_jerk_rad_s3', 8.0)
        self.declare_parameter('settling_time_s', 0.15)
        self.declare_parameter('gripper_max_velocity_rad_s', 0.8)
        self.declare_parameter('gripper_max_acceleration_rad_s2', 4.0)
        self.declare_parameter('gripper_max_jerk_rad_s3', 5.0)
        self.declare_parameter('gripper_settling_time_s', 0.20)
        self.declare_parameter('trajectory_duration_ms', 80)
        self.declare_parameter('invert_roll', False)
        self.declare_parameter('invert_pitch', False)
        self.declare_parameter('invert_yaw', False)
        self.declare_parameter('flex_open_adc', 2900.0)
        self.declare_parameter('flex_closed_adc', 3700.0)
        self.declare_parameter('shoulder_gain', 1.0)
        self.declare_parameter('elbow_gain', 0.6)

        self._udp_port = self.get_parameter('udp_port').value
        self._deadband = _deg2rad(self.get_parameter('deadband_deg').value)
        self._alpha = self.get_parameter('low_pass_alpha').value
        self._smoother_hz = self.get_parameter('smoother_rate_hz').value
        self._pub_deadband = self.get_parameter('publish_deadband_rad').value
        self._traj_ms = self.get_parameter('trajectory_duration_ms').value
        self._inv_roll = self.get_parameter('invert_roll').value
        self._inv_pitch = self.get_parameter('invert_pitch').value
        self._inv_yaw = self.get_parameter('invert_yaw').value
        self._flex_open = self.get_parameter('flex_open_adc').value
        self._flex_closed = self.get_parameter('flex_closed_adc').value
        self._sh_gain = self.get_parameter('shoulder_gain').value
        self._el_gain = self.get_parameter('elbow_gain').value

        arm_v = self.get_parameter('max_velocity_rad_s').value
        arm_a = self.get_parameter('max_acceleration_rad_s2').value
        arm_j = self.get_parameter('max_jerk_rad_s3').value
        arm_st = self.get_parameter('settling_time_s').value
        grip_v = self.get_parameter('gripper_max_velocity_rad_s').value
        grip_a = self.get_parameter('gripper_max_acceleration_rad_s2').value
        grip_j = self.get_parameter('gripper_max_jerk_rad_s3').value
        grip_st = self.get_parameter('gripper_settling_time_s').value

        self._arm_pub = self.create_publisher(
            JointTrajectory, 'arm_controller/joint_trajectory', 10)
        self._grip_pub = self.create_publisher(
            JointTrajectory, 'gripper_controller/joint_trajectory', 10)

        arm_params = {
            'max_velocity': arm_v,
            'max_acceleration': arm_a,
            'max_jerk': arm_j,
            'settling_time': arm_st,
        }
        self._arm_smoothers = [
            JointSmoother(**arm_params),
            JointSmoother(**arm_params),
            JointSmoother(**arm_params),
        ]
        self._grip_smoother = JointSmoother(
            max_velocity=grip_v, max_acceleration=grip_a,
            max_jerk=grip_j, settling_time=grip_st)

        self._lock = threading.Lock()
        self._lp_arm = [0.0, 0.0, 0.0]
        self._lp_grip = 0.0
        self._prev_raw_arm = [0.0, 0.0, 0.0]
        self._prev_raw_grip = 0.0

        self._prev_pub_arm = None
        self._prev_pub_grip = None

        self._last_pkt = 0.0
        self._pkt_count = 0
        self._pub_count = 0

        smoother_dt = 1.0 / self._smoother_hz
        self._last_step_t = time.monotonic()
        self.create_timer(smoother_dt, self._smoother_tick)

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

        self.get_logger().info('=' * 60)
        self.get_logger().info(f'Gesture receiver (jerk-bounded) on UDP {self._udp_port}')
        self.get_logger().info(f'  Smoother @ {self._smoother_hz} Hz  '
                               f'| v_max={arm_v}  a_max={arm_a}  j_max={arm_j}')
        self.get_logger().info('  Hand -> Joint mapping:')
        self.get_logger().info('    Rotate forearm L/R  -> Base (joint_1)')
        self.get_logger().info('    Tilt wrist fwd/back -> Reach (joint_2+3 coupled)')
        self.get_logger().info('    Roll wrist side      -> Elbow fine-tune (additive)')
        self.get_logger().info('    Bend fingers (flex)  -> Gripper (joint_4)')
        self.get_logger().info(f'  flex_open={self._flex_open:.0f}  '
                               f'flex_closed={self._flex_closed:.0f}')
        self.get_logger().info('=' * 60)

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

    def _handle(self, raw):
        parts = raw.split(',')
        if len(parts) < 4:
            return
        try:
            roll = float(parts[0])
            pitch = float(parts[1])
            yaw = float(parts[2])
            flex = float(parts[3])
        except ValueError:
            return

        if self._inv_roll:
            roll = -roll
        if self._inv_pitch:
            pitch = -pitch
        if self._inv_yaw:
            yaw = -yaw

        roll = _clamp(roll,  -90.0, 90.0)
        pitch = _clamp(pitch, -90.0, 90.0)
        yaw = _clamp(yaw,   -90.0, 90.0)

        j1 = _clamp(_deg2rad(yaw), *self.JOINT_LIMITS['joint_1'])

        j2_reach, j3_reach = _pitch_to_reach(pitch, self._sh_gain, self._el_gain)
        j3_roll_offset = _clamp(_deg2rad(roll) * 0.4, -math.pi/4, math.pi/4)

        j2 = _clamp(j2_reach, *self.JOINT_LIMITS['joint_2'])
        j3 = _clamp(j3_reach + j3_roll_offset, *self.JOINT_LIMITS['joint_3'])

        g = _flex_to_gripper(flex, self._flex_open, self._flex_closed)
        g = _clamp(g, *self.JOINT_LIMITS['joint_4'])

        new_arm = [j1, j2, j3]
        new_grip = g

        with self._lock:
            for i in range(3):
                if abs(new_arm[i] - self._prev_raw_arm[i]) < self._deadband:
                    new_arm[i] = self._prev_raw_arm[i]
                self._prev_raw_arm[i] = new_arm[i]
                self._lp_arm[i] += self._alpha * (new_arm[i] - self._lp_arm[i])

            if abs(new_grip - self._prev_raw_grip) < self._deadband:
                new_grip = self._prev_raw_grip
            self._prev_raw_grip = new_grip
            self._lp_grip += self._alpha * (new_grip - self._lp_grip)

            # Update liveness counter alongside the state it protects -- the
            # smoother-tick thread and _watchdog() both read these.
            self._last_pkt = time.monotonic()
            self._pkt_count += 1

    def _smoother_tick(self):
        now = time.monotonic()
        dt = now - self._last_step_t
        # Guard against stalls (e.g. GC pause, Python preemption) so a single
        # tick uses a sane step -- the smoother's internal cap would still hold
        # the integration but the velocity profiler needs a realistic dt to
        # plan from.
        if dt > 0.1:
            dt = 0.1
        self._last_step_t = now

        with self._lock:
            arm_targets = list(self._lp_arm)
            grip_target = self._lp_grip

        arm_positions = []
        for i, smoother in enumerate(self._arm_smoothers):
            arm_positions.append(smoother.step(arm_targets[i], dt))

        grip_position = self._grip_smoother.step(grip_target, dt)

        arm_moved = (
            self._prev_pub_arm is None or
            any(abs(arm_positions[i] - self._prev_pub_arm[i]) > self._pub_deadband
                for i in range(3))
        )
        grip_moved = (
            self._prev_pub_grip is None or
            abs(grip_position - self._prev_pub_grip) > self._pub_deadband
        )

        with self._lock:
            if arm_moved or grip_moved:
                self._pub_count += 1
        if arm_moved:
            self._pub_arm(arm_positions)
            self._prev_pub_arm = list(arm_positions)
        if grip_moved:
            self._pub_grip(grip_position)
            self._prev_pub_grip = grip_position
            self._pub_count += 1

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

    def _watchdog(self):
        if self._last_pkt == 0.0:
            self.get_logger().warn(
                'No UDP packets yet -- is ESP32 powered and on the same WiFi?',
                throttle_duration_sec=10.0)
        elif time.monotonic() - self._last_pkt > 3.0:
            self.get_logger().warn(
                f'No packet for {time.monotonic()-self._last_pkt:.0f}s',
                throttle_duration_sec=5.0)

    def destroy_node(self):
        self._running = False
        try:
            self._sock.close()
        except Exception:
            pass
        super().destroy_node()


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
