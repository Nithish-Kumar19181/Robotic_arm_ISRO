from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    ur_moveit_launch = os.path.join(
        get_package_share_directory('ur_moveit_config'),
        'launch',
        'ur_moveit.launch.py'
    )

    ur_driver_launch = os.path.join(
        get_package_share_directory('ur_robot_driver'),
        'launch',
        'ur10e.launch.py'
    )

    ground_plane = Node(
        package='ur10e_testing_package',
        executable='ur10e_ground_plane',
        name='ur10e_ground_plane',
        output='screen'
    )

    ur_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ur_driver_launch),
        launch_arguments={
            'robot_ip': '172.17.0.2',
            'use_fake_hardware': 'true'
        }.items()
    )

    ur_moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ur_moveit_launch),
        launch_arguments={
            'ur_type': 'ur10e',
            'use_fake_hardware': 'true',
        }.items()
    )

    # Joint angle publisher
    joint_angle_pub = Node(
        package='ur10e_testing_package',
        executable='joint_angle_publisher',
        name='joint_angle_publisher',
        output='screen'
    )

    return LaunchDescription([
        ur_driver,
        ur_moveit,
        joint_angle_pub,
        ground_plane,
    ])
