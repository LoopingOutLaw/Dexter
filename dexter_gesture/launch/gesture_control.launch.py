from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    args = [
        DeclareLaunchArgument('udp_port',               default_value='5005'),
        DeclareLaunchArgument('smoothing_alpha',         default_value='0.20'),
        DeclareLaunchArgument('deadband_deg',            default_value='0.5'),
        DeclareLaunchArgument('trajectory_duration_ms',  default_value='80'),
        DeclareLaunchArgument('invert_roll',             default_value='false'),
        DeclareLaunchArgument('invert_pitch',            default_value='false'),
        DeclareLaunchArgument('invert_yaw',              default_value='false'),
        DeclareLaunchArgument('shoulder_gain',           default_value='1.0'),
        DeclareLaunchArgument('elbow_gain',              default_value='0.6'),
        DeclareLaunchArgument('flex_open_adc',           default_value='2900.0'),
        DeclareLaunchArgument('flex_closed_adc',         default_value='3700.0'),
    ]

    node = Node(
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
            'shoulder_gain':          LaunchConfiguration('shoulder_gain'),
            'elbow_gain':             LaunchConfiguration('elbow_gain'),
            'flex_open_adc':          LaunchConfiguration('flex_open_adc'),
            'flex_closed_adc':        LaunchConfiguration('flex_closed_adc'),
        }]
    )

    return LaunchDescription(args + [node])
