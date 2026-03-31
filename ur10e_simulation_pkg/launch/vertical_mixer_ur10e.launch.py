#!/usr/bin/env python3
"""Launch UR10e (MoveIt) and planetary mixer with isolated control stacks."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.conditions import IfCondition


def generate_launch_description():
    enable_demo_motion = LaunchConfiguration('enable_demo_motion')
    demo_amplitude = LaunchConfiguration('demo_amplitude')
    demo_frequency = LaunchConfiguration('demo_frequency')

    # Package paths
    ur10e_pkg_share = get_package_share_directory('ur10e_simulation_pkg')

    # Paths for planetary mixer files
    sdf_template_path = os.path.join(ur10e_pkg_share, 'model', 'vertical_mixer_planetary.sdf')
    urdf_path = os.path.join(ur10e_pkg_share, 'urdf', 'vertical_mixer_planetary.urdf')
    sdf_spawn_path = '/tmp/vertical_mixer_planetary_spawn.sdf'

    # Load and prepare SDF file for planetary mixer (replace PACKAGE_PATH placeholders)
    with open(sdf_template_path, 'r', encoding='utf-8') as f:
        sdf_content = f.read().replace('PACKAGE_PATH', ur10e_pkg_share)
    with open(sdf_spawn_path, 'w', encoding='utf-8') as f:
        f.write(sdf_content)

    # Load URDF for planetary mixer
    with open(urdf_path, 'r', encoding='utf-8') as f:
        planetary_urdf_content = f.read()

    # UR10e controllers configuration
    controllers_yaml = PathJoinSubstitution([
        FindPackageShare("ur10e_testing_pkg"),
        "config",
        "controller_manager.yaml"
    ])

    # ==================== UR10E ROBOT SETUP ====================
    # Launch UR10e with MoveIt and all necessary controllers
    ur_sim_moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("ur_simulation_gazebo"),
                "launch",
                "ur_sim_moveit.launch.py"
            ])
        ),
        launch_arguments={
            "ur_type": "ur10e",
            "controllers_file": controllers_yaml,
        }.items()
    )

    # Load compliance controller for UR10e but keep it inactive.
    spawn_cartesian_compliance = ExecuteProcess(
        cmd=[
            "ros2", "run", "controller_manager", "spawner",
            "cartesian_compliance_controller",
            "--inactive",
            "--controller-manager", "/controller_manager"
        ],
        output="screen"
    )

    delayed_ur_compliance_loader = TimerAction(
        period=5.0,
        actions=[spawn_cartesian_compliance],
    )

    # Spawn mixer model in /mixer namespace, isolated from UR10e ROS graph.
    spawn_planetary_mixer = TimerAction(
        period=5.0,
        actions=[Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'planetary_mixer',
                '-file', sdf_spawn_path,
                '-x', '0.0',
                '-y', '0.0',
                '-z', '0.00',
            ],
            output='screen',
        )],
    )

    # Dedicated mixer robot_state_publisher in /mixer with prefixed TF frames.
    planetary_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='mixer',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': planetary_urdf_content},
            {'frame_prefix': 'mixer_'},
        ],
        remappings=[
            ('joint_states', '/mixer/joint_states'),
        ]
    )

    # Mixer controllers must target mixer's namespaced controller manager.
    jsb_spawner_mixer = TimerAction(
        period=4.5,
        actions=[Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster', '--controller-manager', '/mixer/controller_manager'],
            output='screen',
        )],
    )

    fpc_spawner_mixer = TimerAction(
        period=5.5,
        actions=[Node(
            package='controller_manager',
            executable='spawner',
            arguments=['forward_position_controller', '--controller-manager', '/mixer/controller_manager'],
            output='screen',
        )],
    )

    # Mixer-only helper nodes in /mixer namespace.
    kinematics_node = TimerAction(
        period=7.0,
        actions=[Node(
            package='ur10e_simulation_pkg',
            executable='planetary_kinematics_node.py',
            namespace='mixer',
            output='screen',
        )],
    )

    demo_motion_node = TimerAction(
        period=7.2,
        actions=[Node(
            package='ur10e_simulation_pkg',
            executable='sun_joint_demo_trajectory.py',
            namespace='mixer',
            output='screen',
            condition=IfCondition(enable_demo_motion),
            parameters=[
                {'amplitude': demo_amplitude},
                {'frequency': demo_frequency},
            ],
        )],
    )

    filtered_force_node = Node(
        package="ur10e_testing_pkg",
        executable="ur10e_force_filter",
        name="ur10e_force_filter",
        output="screen"
    )

    # Tool path visualization marker array
    marker_array_node = Node(
        package="ur10e_testing_pkg",
        executable="ur10e_marker_array",
        name="ur10e_marker_array",
        output="screen"
    )

    delayed_marker_node = TimerAction(
        period=2.0,
        actions=[marker_array_node]
    )

    return LaunchDescription([

        DeclareLaunchArgument(
            'enable_demo_motion',
            default_value='true',
            description='Start sine-wave publisher on /mixer/sun_joint_cmd.'
        ),
        DeclareLaunchArgument(
            'demo_amplitude',
            default_value='1.5',
            description='Demo sine amplitude in radians for /mixer/sun_joint_cmd.'
        ),
        DeclareLaunchArgument(
            'demo_frequency',
            default_value='0.25',
            description='Demo sine frequency in Hz for /mixer/sun_joint_cmd.'
        ),

        # UR10e (MoveIt + /controller_manager)
        spawn_planetary_mixer,
        ur_sim_moveit,
        delayed_ur_compliance_loader,
        delayed_marker_node,
        filtered_force_node,
        # Mixer (/mixer + /mixer/controller_manager)
        jsb_spawner_mixer,
        fpc_spawner_mixer,
        planetary_state_publisher,
        kinematics_node,
        # demo_motion_node,

    ])
