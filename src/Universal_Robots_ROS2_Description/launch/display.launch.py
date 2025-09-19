import os
# Add this import to find ROS packages
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Path to your ur10e xacro file
    default_model_path = "/home/nithish/robotic_arm_ws/src/Universal_Robots_ROS2_Description/urdf/ur10e.urdf.xacro"

    # *** START OF ADDED/MODIFIED SECTION ***

    # Find the path to the ur_description package
    ur_description_path = get_package_share_directory('ur_description')
    
    # Define the path to the default rviz config file
    default_rviz_config_path = os.path.join(ur_description_path, 'rviz', 'view_robot.rviz')
    
    # *** END OF ADDED/MODIFIED SECTION ***
    

    # Declare a launch argument for the model
    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=default_model_path,
        description="Absolute path to robot URDF/XACRO file"
    )
    
    # Generate robot_description parameter by calling xacro
    robot_description = ParameterValue(
        Command(["xacro ", LaunchConfiguration("model")]),
        value_type=str
    )

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            "robot_description": robot_description
        }]
    )

    # Joint State Publisher GUI
    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui"
    )

    # RViz2 (visualization)
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        # Add the argument to load our config file
        arguments=['-d', default_rviz_config_path]
    )

    return LaunchDescription([
        model_arg,
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
    ])