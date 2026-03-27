"""
gesture_sim.launch.py
======================
ONE command to launch the complete gesture-controlled simulation:
  • Gazebo with Dexter robot
  • ros2_control (arm_controller + gripper_controller)
  • MoveIt 2 (optional, for path planning and collision avoidance)
  • Gesture receiver (UDP → joint trajectories)

Usage:
    ros2 launch dexter_gesture gesture_sim.launch.py
    ros2 launch dexter_gesture gesture_sim.launch.py use_moveit:=false
    ros2 launch dexter_gesture gesture_sim.launch.py udp_port:=5005 smoothing_alpha:=0.3
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # ---- Arguments ---------------------------------------------------
    use_moveit_arg = DeclareLaunchArgument(
        'use_moveit', default_value='true',
        description='Launch MoveIt 2 move_group alongside the gesture receiver')

    udp_port_arg = DeclareLaunchArgument(
        'udp_port', default_value='5005',
        description='UDP port for ESP32 gesture packets')

    smoothing_arg = DeclareLaunchArgument(
        'smoothing_alpha', default_value='0.25',
        description='Low-pass filter coefficient (0=frozen, 1=no smoothing)')

    deadband_arg = DeclareLaunchArgument(
        'deadband_deg', default_value='0.5',
        description='Angle deadband in degrees before publishing a new command')

    traj_dur_arg = DeclareLaunchArgument(
        'trajectory_duration_ms', default_value='80',
        description='Trajectory execution window (ms)')

    invert_roll_arg  = DeclareLaunchArgument('invert_roll',  default_value='false')
    invert_pitch_arg = DeclareLaunchArgument('invert_pitch', default_value='false')
    invert_yaw_arg   = DeclareLaunchArgument('invert_yaw',   default_value='false')

    # ---- Sub-launches ------------------------------------------------
    # 1. Gazebo simulation
    gazebo = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory('dexter_description'),
            'launch', 'gazebo.launch.py'
        )
    )

    # 2. Controllers (spawns arm_controller + gripper_controller)
    controller = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory('dexter_controller'),
            'launch', 'controller.launch.py'
        ),
        launch_arguments={'is_sim': 'True'}.items()
    )

    # 3. MoveIt 2 (optional — provides RViz with MotionPlanning panel)
    moveit = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory('dexter_moveit'),
            'launch', 'moveit.launch.py'
        ),
        launch_arguments={'is_sim': 'True'}.items(),
        condition=IfCondition(LaunchConfiguration('use_moveit'))
    )

    # 4. Gesture receiver — delayed by 5 s to let controllers start first
    gesture_receiver_node = Node(
        package='dexter_gesture',
        executable='gesture_receiver.py',
        name='gesture_receiver',
        output='screen',
        parameters=[{
            'udp_port':               LaunchConfiguration('udp_port'),
            'smoothing_alpha':        LaunchConfiguration('smoothing_alpha'),
            'deadband_deg':           LaunchConfiguration('deadband_deg'),
            'trajectory_duration_ms': LaunchConfiguration('trajectory_duration_ms'),
            'invert_roll':            LaunchConfiguration('invert_roll'),
            'invert_pitch':           LaunchConfiguration('invert_pitch'),
            'invert_yaw':             LaunchConfiguration('invert_yaw'),
        }]
    )

    gesture_delayed = TimerAction(
        period=5.0,  # wait 5 s for controllers to be ready
        actions=[gesture_receiver_node]
    )

    return LaunchDescription([
        use_moveit_arg,
        udp_port_arg,
        smoothing_arg,
        deadband_arg,
        traj_dur_arg,
        invert_roll_arg,
        invert_pitch_arg,
        invert_yaw_arg,
        # Launch order: Gazebo → controllers → MoveIt → gesture
        gazebo,
        controller,
        moveit,
        gesture_delayed,
    ])
