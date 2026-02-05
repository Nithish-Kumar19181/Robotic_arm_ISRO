from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    
    bowl_sdf_path = "/home/tachyon/ur10e_ws/src/ur10e_simulation_pkg/model/bowl.sdf"

    controllers_yaml = PathJoinSubstitution([
        FindPackageShare("ur10e_testing_pkg"),
        "config",
        "controller_manager.yaml"
    ])

    ur_sim_moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("ur_simulation_gz"), #for the package ign-gz
                "launch",
                "ur_sim_moveit.launch.py"
            ])
        ),
        launch_arguments={
            "ur_type": "ur10e",
            "controllers_file": controllers_yaml,
        }.items()
    )

    spawn_cartesian_motion = ExecuteProcess(
        cmd=[
            "ros2", "run", "controller_manager", "spawner",
            "cartesian_motion_controller",
            "--inactive",
            "--controller-manager", "/controller_manager"
        ],
        output="screen"
    )

    spawn_cartesian_force = ExecuteProcess(
        cmd=[
            "ros2", "run", "controller_manager", "spawner",
            "cartesian_force_controller",
            "--inactive",
            "--controller-manager", "/controller_manager"
        ],
        output="screen"
    )

    spawn_cartesian_compliance = ExecuteProcess(
        cmd=[
            "ros2", "run", "controller_manager", "spawner",
            "cartesian_compliance_controller",
            "--inactive",
            "--controller-manager", "/controller_manager"
        ],
        output="screen"
    )

    delayed_cartesian_start = TimerAction(
        period=10.0,   # Gazebo is slower than ur_robot_driver
        actions=[
            spawn_cartesian_motion,
            spawn_cartesian_force,
            spawn_cartesian_compliance,
        ]
    )

    wall_srdf = TimerAction(
        period=15.0,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=[
                    '-entity', 'bowl',
                    '-file', bowl_sdf_path,
                    '-x', '0.1',  
                    '-y', '0.003',
                    '-z', '0.0',
                    '-Y', '0.0'
                ],
                output='screen',
            )
        ]
    )
    #forclassic gazebo
    # filtered_force_node = Node(
    #     package="ur10e_testing_pkg",
    #     executable="ur10e_force_filter",
    #     name="ur10e_force_filter",
    #     output="screen"
    # )

    filtered_force_gz_node = Node(
        package="ur10e_testing_pkg",
        executable="ur10e_force_filter_gz",
        name="ur10e_force_filter_gz",
        output="screen"
    )

    marker_array_node = Node(
        package="ur10e_testing_pkg",
        executable="ur10e_marker_array",
        name="ur10e_marker_array",
        output="screen"
    )

    delayed_marker_node = TimerAction(
        period=12.0,
        actions=[marker_array_node]
    )

    force_torque_bridge = Node(
    package="ros_gz_bridge",
    executable="parameter_bridge",
    arguments=[
        "/wrench@geometry_msgs/msg/Wrench@gz.msgs.Wrench"
    ],
    output="screen"
)

    return LaunchDescription([
        ur_sim_moveit,
        delayed_cartesian_start,
        delayed_marker_node,
        wall_srdf,
        force_torque_bridge,
        # filtered_force_node,
        filtered_force_gz_node,
    ])
