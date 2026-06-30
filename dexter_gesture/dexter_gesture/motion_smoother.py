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
Jerk-limited S-curve trajectory generator for a single joint.

Three-stage motion profile:
  1. velocity_desire   = P(position_error),      clamped to [-v_max,  v_max]
  2. accel_desire      = P(vel_desire - vel),    clamped to [-a_max,  a_max]
  3. accel_state       = trapezoidal slew of a_des limited by jerk*dt

Stage 3 is what makes this C2 continuous: instead of capping acceleration
itself, it caps the CHANGE in acceleration per-second. The result is a
bounded-joint jerk, bounded-joint acceleration, bounded-joint velocity,
position-tracking profile suitable for cheap hobby servos.
"""


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


class JointSmoother:
    """Single-axis, jerk-limited trajectory generator."""

    def __init__(
        self,
        max_velocity=1.2,
        max_acceleration=6.0,
        max_jerk=8.0,
        settling_time=0.15,
    ):
        if max_velocity <= 0:
            raise ValueError(f'max_velocity must be > 0, got {max_velocity}')
        if max_acceleration <= 0:
            raise ValueError(f'max_acceleration must be > 0, got {max_acceleration}')
        if max_jerk <= 0:
            raise ValueError(f'max_jerk must be > 0, got {max_jerk}')
        if settling_time <= 0:
            raise ValueError(f'settling_time must be > 0, got {settling_time}')

        self.max_velocity = float(max_velocity)
        self.max_acceleration = float(max_acceleration)
        self.max_jerk = float(max_jerk)
        self._kv = 1.0 / float(settling_time)

        self.pos = 0.0
        self.vel = 0.0
        self.acc = 0.0

    def step(self, target, dt):
        """
        Advance one time step toward ``target``.

        See module docstring for the algorithm description.
        """
        dt = _clamp(float(dt), 0.001, 0.05)
        target = float(target)

        v_des = _clamp(self._kv * (target - self.pos),
                       -self.max_velocity, self.max_velocity)

        a_des = _clamp(v_des - self.vel,
                       -self.max_acceleration, self.max_acceleration)

        a_delta = a_des - self.acc
        a_delta = _clamp(a_delta, -self.max_jerk * dt, self.max_jerk * dt)
        self.acc = self.acc + a_delta

        self.vel = self.vel + self.acc * dt
        self.vel = _clamp(self.vel, -self.max_velocity, self.max_velocity)

        self.pos = self.pos + self.vel * dt

        return self.pos

    def reset(self, pos=0.0):
        """Force-set the smoother state at startup or homing."""
        self.pos = float(pos)
        self.vel = 0.0
        self.acc = 0.0


class GripperSmoother:
    """Single-axis, jerk-limited trajectory generator tuned for the gripper."""

    def __init__(
        self,
        max_velocity=0.8,
        max_acceleration=4.0,
        max_jerk=5.0,
        settling_time=0.20,
    ):
        if max_velocity <= 0:
            raise ValueError(f'max_velocity must be > 0, got {max_velocity}')
        if max_acceleration <= 0:
            raise ValueError(f'max_acceleration must be > 0, got {max_acceleration}')
        if max_jerk <= 0:
            raise ValueError(f'max_jerk must be > 0, got {max_jerk}')
        if settling_time <= 0:
            raise ValueError(f'settling_time must be > 0, got {settling_time}')

        self.max_velocity = float(max_velocity)
        self.max_acceleration = float(max_acceleration)
        self.max_jerk = float(max_jerk)
        self._kv = 1.0 / float(settling_time)

        self.pos = 0.0
        self.vel = 0.0
        self.acc = 0.0

    def step(self, target, dt):
        """
        Advance one time step toward ``target``.

        See module docstring for the algorithm description.
        """
        dt = _clamp(float(dt), 0.001, 0.05)
        target = float(target)

        v_des = _clamp(self._kv * (target - self.pos),
                       -self.max_velocity, self.max_velocity)

        a_des = _clamp(v_des - self.vel,
                       -self.max_acceleration, self.max_acceleration)

        a_delta = a_des - self.acc
        a_delta = _clamp(a_delta, -self.max_jerk * dt, self.max_jerk * dt)
        self.acc = self.acc + a_delta

        self.vel = self.vel + self.acc * dt
        self.vel = _clamp(self.vel, -self.max_velocity, self.max_velocity)

        self.pos = self.pos + self.vel * dt

        return self.pos

    def reset(self, pos=0.0):
        """Force-set the smoother state at startup or homing."""
        self.pos = float(pos)
        self.vel = 0.0
        self.acc = 0.0
