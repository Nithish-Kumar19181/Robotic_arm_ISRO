# !/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    urdf_path = "/home/nithish/isro_arm_ws/src/ur10e_testing_package/urdf/ur10e.urdf.xacro"

    robot_description = Command([
        "xacro ", urdf_path
    ])

    rviz_config_file = os.path.join(
        get_package_share_directory('ur10e_testing_package'),
        "rviz",
        "view_robot.rviz"
    )

    return LaunchDescription([

        # Robot State Publisher - ESSENTIAL for RViz
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[{
                "robot_description": robot_description,
                "publish_frequency": 50.0  # Match your IK publishing rate
            }]
        ),

        # Joint State Publisher GUI - COMMENTED OUT (we'll publish from our node)
        # Node(
        #     package="joint_state_publisher_gui",
        #     executable="joint_state_publisher_gui",
        #     output="screen"
        # ),

        # RViz
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", rviz_config_file]
        ),
    ])