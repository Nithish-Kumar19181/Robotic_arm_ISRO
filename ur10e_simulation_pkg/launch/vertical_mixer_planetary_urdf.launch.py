#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    enable_demo_motion = LaunchConfiguration('enable_demo_motion')
    demo_amplitude = LaunchConfiguration('demo_amplitude')
    demo_frequency = LaunchConfiguration('demo_frequency')

    pkg_share         = get_package_share_directory('ur10e_simulation_pkg')
    sdf_template_path = os.path.join(pkg_share, 'model', 'vertical_mixer_planetary.sdf')
    urdf_path         = os.path.join(pkg_share, 'urdf', 'vertical_mixer_planetary.urdf')
    sdf_spawn_path    = '/tmp/vertical_mixer_planetary_spawn.sdf'

    with open(sdf_template_path, 'r', encoding='utf-8') as f:
        sdf_content = f.read().replace('PACKAGE_PATH', pkg_share)
    with open(sdf_spawn_path, 'w', encoding='utf-8') as f:
        f.write(sdf_content)

    with open(urdf_path, 'r', encoding='utf-8') as f:
        urdf_content = f.read()

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'), 'launch', 'gazebo.launch.py'
            ])
        ),
        launch_arguments={'verbose': 'false'}.items(),
    )

    spawn = TimerAction(
        period=2.0,
        actions=[Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'planetary_mixer',
                '-file',   sdf_spawn_path,
                '-x', '0.0', '-y', '0.0', '-z', '0.00',
            ],
            output='screen',
        )],
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': urdf_content},
        ],
    )

    kinematics_node = TimerAction(
        period=7.0,
        actions=[Node(
            package='ur10e_simulation_pkg',
            executable='planetary_kinematics_node.py',
            output='screen',
        )],
    )

    demo_motion_node = TimerAction(
        period=7.2,
        actions=[Node(
            package='ur10e_simulation_pkg',
            executable='sun_joint_demo_trajectory.py',
            output='screen',
            condition=IfCondition(enable_demo_motion),
            parameters=[
                {'amplitude': demo_amplitude},
                {'frequency': demo_frequency},
            ],
        )],
    )

    jsb_spawner = TimerAction(
        period=4.5,
        actions=[Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
            output='screen',
        )],
    )

    fpc_spawner = TimerAction(
        period=5.5,
        actions=[Node(
            package='controller_manager',
            executable='spawner',
            arguments=['forward_position_controller', '--controller-manager', '/controller_manager'],
            output='screen',
        )],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'enable_demo_motion',
            default_value='true',
            description='Start sine-wave /sun_joint_cmd publisher for visible motion.'
        ),
        DeclareLaunchArgument(
            'demo_amplitude',
            default_value='1.5',
            description='Demo sine amplitude in radians for /sun_joint_cmd.'
        ),
        DeclareLaunchArgument(
            'demo_frequency',
            default_value='0.25',
            description='Demo sine frequency in Hz for /sun_joint_cmd.'
        ),
        gazebo,
        robot_state_publisher,
        spawn,
        jsb_spawner,
        fpc_spawner,
        kinematics_node,
        demo_motion_node,
    ])