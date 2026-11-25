"""Launch the standalone eight-point mission after Nav2 has been started."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use the Gazebo clock',
        ),
        DeclareLaunchArgument(
            'wait_seconds', default_value='2.0',
            description='Pause after each successfully reached waypoint',
        ),
        Node(
            package='roslander_navigation',
            executable='eight_point_navigation',
            name='eight_point_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': ParameterValue(
                    LaunchConfiguration('use_sim_time'), value_type=bool),
                'wait_seconds': ParameterValue(
                    LaunchConfiguration('wait_seconds'), value_type=float),
            }],
        ),
    ])
