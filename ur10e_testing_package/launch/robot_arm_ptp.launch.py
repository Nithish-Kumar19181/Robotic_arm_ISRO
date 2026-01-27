from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro
import os


def generate_launch_description():

    bowl_sdf_path = "/home/nithish/ur10e_ws/src/ros2_nr_motion_control/model/bowl.sdf"
    ur_type = 'ur10e'

    ur_sim_moveit = os.path.join(
        get_package_share_directory('ur_simulation_gazebo'),
        'launch',
        'ur_sim_moveit.launch.py'
    )

    ur_simulation_gazebo_pkg_dir = get_package_share_directory('ur_simulation_gazebo')
    ur_sim_launch_file = os.path.join(ur_simulation_gazebo_pkg_dir, 'launch', 'ur_sim_control.launch.py')

    ur10e_moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ur_sim_moveit),
        launch_arguments={
            'ur_type': 'ur10e',
            'use_fake_hardware': 'true',
            'use_sim_time': 'true',
            'launch_rviz': 'true',
        }.items()
    )

    ur10e_home_node = TimerAction(
        period=3.0,   # wait until everything is ready
        actions=[
            Node(
                package='ur10e_testing_package',
                executable='ur10e_home',
                name='ur10e_home',
                output='screen'
            )
        ]
    )

    wall_srdf = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'bowl',
                '-file', bowl_sdf_path,
                '-x', '0.0',  
                '-y', '0.0',
                '-z', '0.0',
                '-Y', '0.0'
            ],
            output='screen',
        ),

    joint_angle_publisher_node = Node(
        package='ur10e_testing_package',
        executable='joint_angle_publisher',
        name='joint_angle_publisher',
        output='screen'
    )   

    wall_srdf = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-entity', 'bowl',
                    '-file', bowl_sdf_path,
                    '-x', '0.0',  
                    '-y', '0.0',
                    '-z', '0.0',
                    '-Y', '0.0'
                ],
                output='screen',
            )
        ]
    )

    ur10e_ground_plane = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='ur10e_testing_package',
                executable='ur10e_ground_plane',
                name='ur10e_ground_plane',
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        ur10e_moveit,
        wall_srdf,
        ur10e_ground_plane,
        ur10e_home_node,
        joint_angle_publisher_node,
    ])
