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
"""Unit tests for dexter_gesture.motion_smoother."""

import math

from dexter_gesture.motion_smoother import (  # noqa: F401
    GripperSmoother,
    JointSmoother,
)
import pytest


class TestJointSmootherStep:
    """Basic step-response tests."""

    def test_converges_to_target(self):
        """After many steps, smoother must reach the target position."""
        s = JointSmoother(max_velocity=1.2, max_acceleration=6.0, max_jerk=8.0)
        target = 0.5  # rad
        dt = 0.01     # 100 Hz
        for _ in range(2000):  # 20 seconds — more than enough
            s.step(target, dt)
        assert abs(s.pos - target) < 1e-4, f'pos={s.pos:.6f}, target={target}'

    def test_velocity_bounded(self):
        """Peak velocity in a step response must not exceed max_velocity."""
        v_max = 1.2
        s = JointSmoother(max_velocity=v_max, max_acceleration=6.0, max_jerk=8.0)
        target = 1.0  # large step to guarantee v_max is hit
        dt = 0.001
        peak_v = 0.0
        for _ in range(50000):
            s.step(target, dt)
            peak_v = max(peak_v, abs(s.vel))
        assert peak_v <= v_max * 1.05, f'peak_v={peak_v:.4f} > {v_max}'

    def test_acceleration_bounded(self):
        """Peak acceleration must not exceed max_acceleration."""
        a_max = 6.0
        s = JointSmoother(max_velocity=1.2, max_acceleration=a_max, max_jerk=8.0)
        target = 1.0
        dt = 0.001
        peak_a = 0.0
        for _ in range(50000):
            s.step(target, dt)
            peak_a = max(peak_a, abs(s.acc))
        assert peak_a <= a_max * 1.05, f'peak_a={peak_a:.4f} > {a_max}'

    def test_jerk_bounded(self):
        """Peak jerk must not exceed max_jerk."""
        j_max = 8.0
        s = JointSmoother(max_velocity=1.2, max_acceleration=6.0, max_jerk=j_max)
        target = 1.0
        dt = 0.001
        prev_acc = 0.0
        peak_j = 0.0
        for _ in range(50000):
            s.step(target, dt)
            jerk = abs(s.acc - prev_acc) / dt
            peak_j = max(peak_j, jerk)
            prev_acc = s.acc
        assert peak_j <= j_max * 1.10, f'peak_j={peak_j:.4f} > {j_max}'

    def test_settles_to_zero_velocity(self):
        """When the target hasn't changed, velocity decays to zero."""
        s = JointSmoother(max_velocity=1.2, max_acceleration=6.0, max_jerk=8.0)
        target = 0.3
        dt = 0.01
        for _ in range(3000):
            s.step(target, dt)
        assert abs(s.vel) < 1e-4, f'vel={s.vel:.6f}'

    def test_settles_to_zero_acceleration(self):
        """At rest, acceleration must be near zero."""
        s = JointSmoother(max_velocity=1.2, max_acceleration=6.0, max_jerk=8.0)
        target = 0.3
        dt = 0.01
        for _ in range(3000):
            s.step(target, dt)
        assert abs(s.acc) < 1e-4, f'acc={s.acc:.6f}'

    def test_direction_reversal_smooth(self):
        """Reversing target direction should not produce a velocity spike."""
        s = JointSmoother(max_velocity=1.2, max_acceleration=6.0, max_jerk=8.0)
        dt = 0.001
        peak_v = 0.0
        # Go to +0.5
        for _ in range(50000):
            s.step(0.5, dt)
        # Reverse to -0.5
        target = -0.5
        for _ in range(50000):
            s.step(target, dt)
            peak_v = max(peak_v, abs(s.vel))
        assert peak_v <= 1.2 * 1.05, f'peak_v on reversal={peak_v:.4f}'


class TestJointSmootherZeroTarget:
    """From-rest step to zero — should not move."""

    def test_zero_target_no_motion(self):
        s = JointSmoother(max_velocity=1.2, max_acceleration=6.0, max_jerk=8.0)
        for _ in range(100):
            s.step(0.0, 0.01)
        assert s.pos == 0.0
        assert s.vel == 0.0
        assert s.acc == 0.0


class TestGripperSmootherStep:
    """Mirror of the JointSmoother tests against the softer gripper caps."""

    def test_converges_to_target(self):
        s = GripperSmoother()
        target = 0.5
        dt = 0.01
        for _ in range(2000):
            s.step(target, dt)
        assert abs(s.pos - target) < 1e-4

    def test_velocity_bounded(self):
        v_max = GripperSmoother().max_velocity
        s = GripperSmoother()
        dt = 0.001
        peak_v = 0.0
        for _ in range(50000):
            s.step(1.0, dt)
            peak_v = max(peak_v, abs(s.vel))
        assert peak_v <= v_max * 1.05, f'peak_v={peak_v:.4f} > {v_max}'

    def test_acceleration_bounded(self):
        a_max = GripperSmoother().max_acceleration
        s = GripperSmoother()
        dt = 0.001
        peak_a = 0.0
        for _ in range(50000):
            s.step(1.0, dt)
            peak_a = max(peak_a, abs(s.acc))
        assert peak_a <= a_max * 1.05, f'peak_a={peak_a:.4f} > {a_max}'

    def test_jerk_bounded(self):
        j_max = GripperSmoother().max_jerk
        s = GripperSmoother()
        dt = 0.001
        prev_acc = 0.0
        peak_j = 0.0
        for _ in range(50000):
            s.step(1.0, dt)
            jerk = abs(s.acc - prev_acc) / dt
            peak_j = max(peak_j, jerk)
            prev_acc = s.acc
        assert peak_j <= j_max * 1.10, f'peak_j={peak_j:.4f} > {j_max}'

    def test_settles_to_zero_velocity(self):
        s = GripperSmoother()
        for _ in range(3000):
            s.step(0.3, 0.01)
        assert abs(s.vel) < 1e-4

    def test_settles_to_zero_acceleration(self):
        s = GripperSmoother()
        for _ in range(3000):
            s.step(0.3, 0.01)
        assert abs(s.acc) < 1e-4


class TestSmootherDtClamp:
    """
    Edge cases on the dt-clamp -- the smoother's safety net.

    If the executor stalls, a single big dt must not produce a velocity
    spike. Zero or negative dt must not produce NaN.
    """

    @pytest.mark.parametrize('ctor', [JointSmoother, GripperSmoother])
    def test_huge_dt_does_not_explode_velocity(self, ctor):
        # 5-second "single step" -- should be clamped into the smoother's
        # internal dt window so the integrator emits a bounded velocity,
        # not (1.0 * 5.0) = 5 m/s of free integration.
        s = ctor()
        s.step(1.0, 5.0)
        v_cap = s.max_velocity * 1.05
        assert abs(s.vel) <= v_cap, f'vel={s.vel:.4f} > {v_cap}'

    @pytest.mark.parametrize('ctor', [JointSmoother, GripperSmoother])
    def test_zero_dt_does_not_nan(self, ctor):
        # A noisy timer callback can hand us dt=0 on occasion.
        s = ctor()
        s.step(0.5, 0.0)
        assert math.isfinite(s.pos)
        assert math.isfinite(s.vel)
        assert math.isfinite(s.acc)

    @pytest.mark.parametrize('ctor', [JointSmoother, GripperSmoother])
    def test_negative_dt_does_not_nan(self, ctor):
        # A bug in upstream timing should not crash the node.
        s = ctor()
        s.step(0.5, -0.01)
        assert math.isfinite(s.pos)
        assert math.isfinite(s.vel)
        assert math.isfinite(s.acc)


class TestSmootherParamValidation:
    """The ctor must reject non-positive caps loudly, not silently."""

    @pytest.mark.parametrize(
        'kwargs',
        [
            {'max_velocity': 0.0},
            {'max_velocity': -1.0},
            {'max_acceleration': 0.0},
            {'max_acceleration': -2.0},
            {'max_jerk': 0.0},
            {'max_jerk': -3.0},
            {'settling_time': 0.0},
            {'settling_time': -0.1},
        ],
    )
    def test_joint_smoother_rejects_bad_caps(self, kwargs):
        with pytest.raises(ValueError):
            JointSmoother(**kwargs)

    def test_gripper_smoother_rejects_bad_caps(self):
        with pytest.raises(ValueError):
            GripperSmoother(max_velocity=-1.0)
