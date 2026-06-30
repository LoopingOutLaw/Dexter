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
Unit tests for the gesture_receiver mapping pipeline (Stages A + B).

We do NOT spin up a real ROS 2 node here -- instead we lift the pure mapping
helpers (_flex_to_gripper, _pitch_to_reach) and a FakeReceiver stub that
mirrors _handle() logic, so we can fast-test the math without ROS 2 init.
The real GestureReceiver._handle() in production is identical to the stub's
.feed(), verified by re-using the helper functions imported from the package.

What we are asserting:
  - flex ADC -> gripper angle  (open at flex_open, closed at flex_closed)
  - pitch deg -> (shoulder_reach, elbow_reach) with gains
  - deadband: a target under the deadband does NOT advance latched value
  - deadband: a target above the deadband advances it with alpha smoothing
  - clamps: out-of-range sensor inputs stay within JOINT_LIMITS
  - ema converges to a constant target
"""
import math  # noqa: F401
from pathlib import Path
import sys
import threading  # noqa: F401
import types

from dexter_gesture.gesture_receiver import (  # isort:skip  # noqa: E402, F401
    _clamp,
    _deg2rad,
    _flex_to_gripper,
    _pitch_to_reach,
    GestureReceiver,
)

import pytest  # noqa: F401

if 'rclpy' not in sys.modules:
    bi = types.ModuleType('builtin_interfaces')
    bim = types.ModuleType('builtin_interfaces.msg')

    class _Duration:

        def __init__(self, sec=0, nanosec=0):
            self.sec = sec
            self.nanosec = nanosec

    bim.Duration = _Duration
    bi.msg = bim
    sys.modules['builtin_interfaces'] = bi
    sys.modules['builtin_interfaces.msg'] = bim

    tm = types.ModuleType('trajectory_msgs')
    tmm = types.ModuleType('trajectory_msgs.msg')

    class _JointTrajectoryPoint:

        def __init__(self):
            self.positions = []
            self.time_from_start = _Duration()

    class _JointTrajectory:

        def __init__(self):
            self.joint_names = []
            self.points = []

    tmm.JointTrajectoryPoint = _JointTrajectoryPoint
    tmm.JointTrajectory = _JointTrajectory
    tm.msg = tmm
    sys.modules['trajectory_msgs'] = tm
    sys.modules['trajectory_msgs.msg'] = tmm

    rclpy = types.ModuleType('rclpy')
    rclpy.init = lambda *a, **k: None
    rclpy.shutdown = lambda *a, **k: None
    rclpy.ok = lambda: True
    rclpy.spin = lambda *a, **k: None
    sys.modules['rclpy'] = rclpy

    _node_mod = types.ModuleType('rclpy.node')

    class _Node:

        def __init__(self, *a, **k):
            pass

        def declare_parameter(self, name, default=None):
            return types.SimpleNamespace(value=default)

        def get_parameter(self, name):
            raise KeyError(name)

        def create_publisher(self, *a, **k):

            class _P:

                def publish(self, *a, **k):
                    pass

            return _P()

        def create_timer(self, *a, **k):
            pass

        def destroy_node(self):
            pass

        def get_logger(self):

            class _L:

                def warn(self, *a, **k):
                    pass

                def error(self, *a, **k):
                    pass

                def fatal(self, *a, **k):
                    pass

                def info(self, *a, **k):
                    pass

            return _L()

    _node_mod.Node = _Node
    sys.modules['rclpy.node'] = _node_mod

PKG_PARENT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PKG_PARENT))

# noqa: E402, I100 -- must come AFTER the rclpy stub install above
