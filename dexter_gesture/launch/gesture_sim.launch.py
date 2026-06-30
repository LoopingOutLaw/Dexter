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
Launch the full gesture-controlled simulation stack in one command.

Pulls in the Gazebo world from `dexter_description`, the ros2_control
controllers from `dexter_controller`, optional MoveIt 2 planning from
`dexter_moveit`, and the (jerk-bounded) `gesture_receiver` from this
package.  Everything stays on a single launch so you can simulate the
whole glove-to-arm chain without the physical hardware.

Launch order
------------

    Gazebo --> ros2_control --> MoveIt 2 (optional) --> gesture_receiver

The gesture node is delayed by 5 s so controllers have time to come up.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    Build the full sim launch description.

    Returns a ``LaunchDescription`` containing:
      * Gazebo (from dexter_description)
      * arm_controller + gripper_controller (from dexter_controller)
      * MoveIt 2 (from dexter_moveit) — optional, gated by ``use_moveit``
      * gesture_receiver node, delayed 5 s
    """
    # ── arguments ────────────────────────────────────────────────────
    use_moveit_arg = DeclareLaunchArgument(
        'use_moveit', default_value='true',
        description='Launch MoveIt 2 alongside the gesture receiver',
    )

    udp_port_arg = DeclareLaunchArgument(
        'udp_port', default_value='5005',
        description='UDP port the receiver listens on for ESP32 packets',
    )

    # Smoothing pipeline tunables (must match the gesture_config.yaml defaults)
    low_pass_arg = DeclareLaunchArgument(
        'low_pass_alpha', default_value='0.12',
        description='Stage B EMA coefficient (0=frozen, 1=raw)',
    )
    deadband_arg = DeclareLaunchArgument(
        'deadband_deg', default_value='0.5',
        description='Stage A deadband (degrees)',
    )
    publish_deadband_arg = DeclareLaunchArgument(
        'publish_deadband_rad', default_value='0.005',
        description='Stage D: suppress publishes below this joint delta',
    )
    smoother_rate_arg = DeclareLaunchArgument(
        'smoother_rate_hz', default_value='100',
        description='Wall-clock rate of the Stage C S-curve tick',
    )

    # Arm caps
    arm_v_arg = DeclareLaunchArgument(
        'max_velocity_rad_s', default_value='1.2',
        description='Arm |velocity| cap (rad/s)',
    )
    arm_a_arg = DeclareLaunchArgument(
        'max_acceleration_rad_s2', default_value='6.0',
        description='Arm |acceleration| cap (rad/s^2)',
    )
    arm_j_arg = DeclareLaunchArgument(
        'max_jerk_rad_s3', default_value='8.0',
        description='Arm |jerk| cap (rad/s^3)',
    )

    # Gripper caps (slower, gentler)
    gripper_v_arg = DeclareLaunchArgument(
        'gripper_max_velocity_rad_s', default_value='0.8',
        description='Gripper |velocity| cap (rad/s)',
    )
    gripper_a_arg = DeclareLaunchArgument(
        'gripper_max_acceleration_rad_s2', default_value='4.0',
        description='Gripper |acceleration| cap (rad/s^2)',
    )
    gripper_j_arg = DeclareLaunchArgument(
        'gripper_max_jerk_rad_s3', default_value='5.0',
        description='Gripper |jerk| cap (rad/s^3)',
    )

    traj_dur_arg = DeclareLaunchArgument(
        'trajectory_duration_ms', default_value='80',
        description='JointTrajectory point time_from_start (ms)',
    )

    invert_roll_arg = DeclareLaunchArgument(
        'invert_roll', default_value='false',
        description='Flip roll axis',
    )
    invert_pitch_arg = DeclareLaunchArgument(
        'invert_pitch', default_value='false',
        description='Flip pitch axis',
    )
    invert_yaw_arg = DeclareLaunchArgument(
        'invert_yaw', default_value='false',
        description='Flip yaw axis',
    )

    # ── sub-launches ─────────────────────────────────────────────────
    gazebo = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory('dexter_description'),
            'launch', 'gazebo.launch.py',
        ),
    )

    controller = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory('dexter_controller'),
            'launch', 'controller.launch.py',
        ),
        launch_arguments={'is_sim': 'True'}.items(),
    )

    moveit = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory('dexter_moveit'),
            'launch', 'moveit.launch.py',
        ),
        launch_arguments={'is_sim': 'True'}.items(),
        condition=IfCondition(LaunchConfiguration('use_moveit')),
    )

    # ── gesture receiver node ────────────────────────────────────────
    gesture_receiver_node = Node(
        package='dexter_gesture',
        executable='gesture_receiver.py',
        name='gesture_receiver',
        output='screen',
        parameters=[{
            'udp_port':                       LaunchConfiguration('udp_port'),
            'low_pass_alpha':                 LaunchConfiguration('low_pass_alpha'),
            'deadband_deg':                   LaunchConfiguration('deadband_deg'),
            'publish_deadband_rad':           LaunchConfiguration('publish_deadband_rad'),
            'smoother_rate_hz':               LaunchConfiguration('smoother_rate_hz'),
            'max_velocity_rad_s':             LaunchConfiguration('max_velocity_rad_s'),
            'max_acceleration_rad_s2':        LaunchConfiguration('max_acceleration_rad_s2'),
            'max_jerk_rad_s3':                LaunchConfiguration('max_jerk_rad_s3'),
            'gripper_max_velocity_rad_s':     LaunchConfiguration('gripper_max_velocity_rad_s'),
            # noqa: E501 -- long key required to match ROS parameter name
            'gripper_max_acceleration_rad_s2':
                LaunchConfiguration('gripper_max_acceleration_rad_s2'),
            'gripper_max_jerk_rad_s3':         LaunchConfiguration('gripper_max_jerk_rad_s3'),
            'trajectory_duration_ms':         LaunchConfiguration('trajectory_duration_ms'),
            'invert_roll':                    LaunchConfiguration('invert_roll'),
            'invert_pitch':                   LaunchConfiguration('invert_pitch'),
            'invert_yaw':                     LaunchConfiguration('invert_yaw'),
        }],
    )

    gesture_delayed = TimerAction(
        period=5.0,  # wait 5 s for controllers to be ready
        actions=[gesture_receiver_node],
    )

    return LaunchDescription([
        use_moveit_arg,
        udp_port_arg,
        low_pass_arg,
        deadband_arg,
        publish_deadband_arg,
        smoother_rate_arg,
        arm_v_arg,
        arm_a_arg,
        arm_j_arg,
        gripper_v_arg,
        gripper_a_arg,
        gripper_j_arg,
        traj_dur_arg,
        invert_roll_arg,
        invert_pitch_arg,
        invert_yaw_arg,
        gazebo,
        controller,
        moveit,
        gesture_delayed,
    ])
