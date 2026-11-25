"""Start the existing simulation, map/AMCL, Nav2 and the navigation RViz view."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from nav2_common.launch import RewrittenYaml


def _check_files(context):
    for name in ('map', 'params_file', 'world', 'rviz_config'):
        path = LaunchConfiguration(name).perform(context)
        if not os.path.isfile(path):
            raise RuntimeError(f'{name} file does not exist: {path}')
    return []


def generate_launch_description():
    share = get_package_share_directory('roslander_navigation')
    description_share = get_package_share_directory('roslander_description')
    nav2_launch = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')
    use_sim_time = LaunchConfiguration('use_sim_time')
    # Wheel slip makes encoder yaw unreliable. Publish raw encoder odometry
    # separately; EKF owns the only odom -> base_footprint transform.
    controllers = RewrittenYaml(
        source_file=os.path.join(description_share, 'config', 'controllers.yaml'),
        root_key='', param_rewrites={'enable_odom_tf': 'false'}, convert_types=True,
    )

    # The supplied map frame is translated from Gazebo world so the lower-left
    # inner wall corner in the Gazebo view is map (0, 0). The axes keep their
    # Gazebo orientation: red is +X and green is +Y. initial_pose_* defaults
    # apply the same translation to the Gazebo spawn pose.
    params = RewrittenYaml(
        source_file=LaunchConfiguration('params_file'),
        root_key='',
        param_rewrites={
            'use_sim_time': use_sim_time,
            'set_initial_pose': LaunchConfiguration('set_initial_pose'),
            'amcl.ros__parameters.initial_pose.x': LaunchConfiguration('initial_pose_x'),
            'amcl.ros__parameters.initial_pose.y': LaunchConfiguration('initial_pose_y'),
            'amcl.ros__parameters.initial_pose.yaw': LaunchConfiguration('initial_pose_yaw'),
            'default_nav_to_pose_bt_xml': os.path.join(
                share, 'behavior_trees', 'navigate_to_pose.xml'),
            'default_nav_through_poses_bt_xml': os.path.join(
                share, 'behavior_trees', 'navigate_through_poses.xml'),
        },
        convert_types=True,
    )

    common = {
        'use_sim_time': use_sim_time,
        'params_file': params,
        'autostart': LaunchConfiguration('autostart'),
        'use_composition': 'False',
        'log_level': LaunchConfiguration('log_level'),
    }
    return LaunchDescription([
        DeclareLaunchArgument('start_sim', default_value='true',
                              description='Start Gazebo and the existing chassis controllers'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('gui', default_value='true', description='Show Gazebo GUI'),
        DeclareLaunchArgument('rviz', default_value='true', description='Show navigation RViz'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('log_level', default_value='info'),
        DeclareLaunchArgument('paused', default_value='false'),
        DeclareLaunchArgument('verbose', default_value='false'),
        DeclareLaunchArgument('world', default_value=os.path.join(
            description_share, 'worlds', 'roslander_arena.world')),
        DeclareLaunchArgument('map', default_value=os.path.join(share, 'maps', 'current.yaml')),
        DeclareLaunchArgument('params_file', default_value=os.path.join(
            share, 'config', 'nav2_params.yaml')),
        DeclareLaunchArgument('rviz_config', default_value=os.path.join(
            share, 'rviz', 'navigation.rviz')),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument('set_initial_pose', default_value='true',
                              description='Seed AMCL; false requires RViz 2D Pose Estimate'),
        DeclareLaunchArgument(
            'initial_pose_x',
            default_value=PythonExpression([LaunchConfiguration('x'), ' + 0.34']),
        ),
        DeclareLaunchArgument(
            'initial_pose_y',
            default_value=PythonExpression([LaunchConfiguration('y'), ' - 0.27']),
        ),
        DeclareLaunchArgument('initial_pose_yaw', default_value=LaunchConfiguration('yaw')),
        OpaqueFunction(function=_check_files),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                description_share, 'launch', 'simulation.launch.py')),
            condition=IfCondition(LaunchConfiguration('start_sim')),
            launch_arguments={**{name: LaunchConfiguration(name) for name in (
                'world', 'gui', 'paused', 'verbose', 'use_sim_time', 'x', 'y', 'z', 'yaw'
            )}, 'controllers_file': controllers, 'odom_topic': '/wheel/odom'}.items(),
        ),
        Node(
            package='robot_localization', executable='ekf_node', name='ekf_filter_node',
            output='screen',
            parameters=[os.path.join(share, 'config', 'ekf.yaml'),
                        {'use_sim_time': ParameterValue(use_sim_time, value_type=bool)}],
            remappings=[('odometry/filtered', '/odom')],
        ),
        # Gazebo publishes /clock only after the world and robot plugins load.
        # Starting lifecycle transitions during that initial clock jump can
        # make a service request time out and leave Nav2 inactive.
        TimerAction(
            period=5.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(os.path.join(
                        nav2_launch, 'localization_launch.py')),
                    launch_arguments={
                        **common, 'map': LaunchConfiguration('map')}.items(),
                ),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(os.path.join(
                        nav2_launch, 'navigation_launch.py')),
                    launch_arguments=common.items(),
                ),
            ],
        ),
        Node(
            package='rviz2', executable='rviz2', name='rviz2', output='screen',
            condition=IfCondition(LaunchConfiguration('rviz')),
            arguments=['-d', LaunchConfiguration('rviz_config')],
            parameters=[{'use_sim_time': ParameterValue(use_sim_time, value_type=bool)}],
        ),
    ])
