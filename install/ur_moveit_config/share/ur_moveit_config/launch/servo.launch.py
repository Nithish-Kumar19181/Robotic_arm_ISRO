# Copyright (c) 2021 PickNik, Inc.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
import os

def launch_setup(context, *args, **kwargs):
    # Initialize arguments
    use_sim_time = LaunchConfiguration("use_sim_time")
    launch_rviz = LaunchConfiguration("launch_rviz")
    launch_servo = LaunchConfiguration("launch_servo")
    
    # Find the path to ur_moveit_config
    ur_moveit_config_path = FindPackageShare('ur_moveit_config')
    
    # Include the MoveIt launch file
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                ur_moveit_config_path,
                'launch',
                'ur_moveit.launch.py'
            ])
        ]),
        launch_arguments={
            "ur_type": "ur10e",
            "use_sim_time": use_sim_time,
            "launch_rviz": launch_rviz,
            "launch_servo": launch_servo,
        }.items(),
    )
    
    return [moveit_launch]

def generate_launch_description():
    declared_arguments = []
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation clock",
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="true",
            description="Launch RViz",
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_servo", 
            default_value="true",
            description="Launch Servo node",
        )
    )
    
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])