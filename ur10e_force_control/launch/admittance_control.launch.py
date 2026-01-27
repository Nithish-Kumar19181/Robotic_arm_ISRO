from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro
import os


def generate_launch_description():
    
    ur_type = 'ur10e'

    wall_detection_node = TimerAction(
        period=1.0,   # wait until everything is ready
        actions=[
            Node(
                package='ur10e_force_control',
                executable='wall_detection_node',
                name='wall_detection_node',
                output='screen',
            )   
        ]
    )

    admittance_control_node = TimerAction(
        period=1.0,   # wait until everything is ready
        actions=[
            Node(
                package='ur10e_force_control',
                executable='admittance_control_node',
                name='admittance_control_node',
                output='screen',
            )   
        ]
    )

    lifecycle_manager_node = TimerAction(
        period=7.0,   # wait until everything is ready
        actions=[
            Node(
                package='ur10e_force_control',
                executable='lifecycle_manager_node',
                name='lifecycle_manager_node',
                output='screen',
            )
        ]
    )

    marker_publisher_node = TimerAction(
        period=0.0,   # wait until everything is ready
        actions=[
            Node(
                package='ur10e_force_control',
                executable='marker_array_node',
                name='marker_array_node',
                output='screen',
            )
        ]
    )

    return LaunchDescription([
        wall_detection_node,
        admittance_control_node,
        lifecycle_manager_node,
        marker_publisher_node
    ])
