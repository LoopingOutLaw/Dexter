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
Launch the gesture_receiver node with jerk-bounded smoother parameters.

Exposes every parameter from the 4-stage pipeline (deadband, low-pass,
smoother rate, v/a/j caps, gripper-specific caps, publish deadband,
axis-inversion flags, reach coupling gains, flex sensor calibration) as a
launch-time override. Defaults match ``config/gesture_config.yaml``.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    args = [
        # Network / pipeline timing
        DeclareLaunchArgument('udp_port',               default_value='5005'),
        DeclareLaunchArgument('smoother_rate_hz',       default_value='100'),
        DeclareLaunchArgument('trajectory_duration_ms', default_value='80'),

        # Stage A -- deadband
        DeclareLaunchArgument('deadband_deg',           default_value='0.5'),

        # Stage B -- low-pass EMA
        DeclareLaunchArgument('low_pass_alpha',         default_value='0.12'),

        # Stage C -- jerk-bounded S-curve (arm)
        DeclareLaunchArgument('max_velocity_rad_s',        default_value='1.2'),
        DeclareLaunchArgument('max_acceleration_rad_s2',   default_value='6.0'),
        DeclareLaunchArgument('max_jerk_rad_s3',           default_value='8.0'),
        DeclareLaunchArgument('settling_time_s',           default_value='0.15'),

        # Stage C -- jerk-bounded S-curve (gripper)
        DeclareLaunchArgument('gripper_max_velocity_rad_s',        default_value='0.8'),
        DeclareLaunchArgument('gripper_max_acceleration_rad_s2',   default_value='4.0'),
        DeclareLaunchArgument('gripper_max_jerk_rad_s3',           default_value='5.0'),
        DeclareLaunchArgument('gripper_settling_time_s',           default_value='0.20'),

        # Stage D -- publish deadband
        DeclareLaunchArgument('publish_deadband_rad',   default_value='0.005'),

        # Axis inversion
        DeclareLaunchArgument('invert_roll',            default_value='false'),
        DeclareLaunchArgument('invert_pitch',           default_value='false'),
        DeclareLaunchArgument('invert_yaw',             default_value='false'),

        # Reach coupling
        DeclareLaunchArgument('shoulder_gain',          default_value='1.0'),
        DeclareLaunchArgument('elbow_gain',             default_value='0.6'),

        # Flex sensor calibration
        DeclareLaunchArgument('flex_open_adc',          default_value='2900.0'),
        DeclareLaunchArgument('flex_closed_adc',        default_value='3700.0'),
    ]

    node = Node(
        package='dexter_gesture',
        executable='gesture_receiver.py',
        name='gesture_receiver',
        output='screen',
        parameters=[{
            'udp_port':                       LaunchConfiguration('udp_port'),
            'smoother_rate_hz':               LaunchConfiguration('smoother_rate_hz'),
            'trajectory_duration_ms':         LaunchConfiguration('trajectory_duration_ms'),
            'deadband_deg':                   LaunchConfiguration('deadband_deg'),
            'low_pass_alpha':                 LaunchConfiguration('low_pass_alpha'),
            'max_velocity_rad_s':             LaunchConfiguration('max_velocity_rad_s'),
            'max_acceleration_rad_s2':        LaunchConfiguration('max_acceleration_rad_s2'),
            'max_jerk_rad_s3':                LaunchConfiguration('max_jerk_rad_s3'),
            'settling_time_s':                LaunchConfiguration('settling_time_s'),
            'gripper_max_velocity_rad_s':     LaunchConfiguration('gripper_max_velocity_rad_s'),
            # noqa: E501 -- long key required to match ROS parameter name
            'gripper_max_acceleration_rad_s2':
                LaunchConfiguration('gripper_max_acceleration_rad_s2'),
            'gripper_max_jerk_rad_s3':         LaunchConfiguration('gripper_max_jerk_rad_s3'),
            'gripper_settling_time_s':         LaunchConfiguration('gripper_settling_time_s'),
            'publish_deadband_rad':           LaunchConfiguration('publish_deadband_rad'),
            'invert_roll':                    LaunchConfiguration('invert_roll'),
            'invert_pitch':                   LaunchConfiguration('invert_pitch'),
            'invert_yaw':                     LaunchConfiguration('invert_yaw'),
            'shoulder_gain':                  LaunchConfiguration('shoulder_gain'),
            'elbow_gain':                     LaunchConfiguration('elbow_gain'),
            'flex_open_adc':                  LaunchConfiguration('flex_open_adc'),
            'flex_closed_adc':                LaunchConfiguration('flex_closed_adc'),
        }]
    )

    return LaunchDescription(args + [node])
