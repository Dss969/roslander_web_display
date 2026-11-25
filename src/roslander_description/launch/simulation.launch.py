import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    description_share = get_package_share_directory('roslander_description')
    gazebo_share = get_package_share_directory('gazebo_ros')

    # Gazebo Classic converts package://roslander_description/... mesh URIs
    # to model://roslander_description/....  It therefore needs the parent
    # directory containing the installed roslander_description package.
    model_paths = [
        os.path.dirname(description_share),
        os.path.expanduser('~/.gazebo/models'),
        '/usr/share/gazebo-11/models',
    ]
    gazebo_model_path = os.pathsep.join(
        path for path in model_paths if os.path.isdir(path)
    )

    xacro_file = os.path.join(
        description_share, 'urdf', 'roslander.urdf.xacro'
    )
    default_world = os.path.join(
        description_share, 'worlds', 'roslander_arena.world'
    )

    world = LaunchConfiguration('world')
    gui = LaunchConfiguration('gui')
    paused = LaunchConfiguration('paused')
    verbose = LaunchConfiguration('verbose')
    use_sim_time = LaunchConfiguration('use_sim_time')
    spawn_x = LaunchConfiguration('x')
    spawn_y = LaunchConfiguration('y')
    spawn_z = LaunchConfiguration('z')
    spawn_yaw = LaunchConfiguration('yaw')

    robot_description = ParameterValue(
        Command([
            FindExecutable(name='xacro'), ' ', xacro_file,
            ' controllers_file:=', LaunchConfiguration('controllers_file'),
            ' odom_topic:=', LaunchConfiguration('odom_topic'),
        ]),
        value_type=str,
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_share, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': world,
            'gui': gui,
            'pause': paused,
            'verbose': verbose,
        }.items(),
    )

    state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
        }],
    )

    spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_roslander',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'roslander',
            '-reference_frame', 'world',
            '-x', spawn_x,
            '-y', spawn_y,
            '-z', spawn_z,
            '-Y', spawn_yaw,
            '-timeout', '60.0',
        ],
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        name='joint_state_broadcaster_spawner',
        output='screen',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '60',
            '--switch-timeout', '30',
        ],
    )

    diff_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        name='diff_drive_controller_spawner',
        output='screen',
        arguments=[
            'diff_drive_controller',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '60',
            '--switch-timeout', '30',
        ],
    )

    load_joint_state_broadcaster = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    load_diff_drive_controller = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[diff_drive_controller_spawner],
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument('controllers_file', default_value=os.path.join(
            description_share, 'config', 'controllers.yaml')),
        DeclareLaunchArgument('odom_topic', default_value='/odom'),
        DeclareLaunchArgument('world', default_value=default_world),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('paused', default_value='false'),
        DeclareLaunchArgument('verbose', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        SetEnvironmentVariable('GAZEBO_MODEL_DATABASE_URI', ''),
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', gazebo_model_path),
        SetEnvironmentVariable('LIDAR_TYPE', 'S2L'),
        gazebo,
        state_publisher,
        load_joint_state_broadcaster,
        load_diff_drive_controller,
        spawn,
    ])
