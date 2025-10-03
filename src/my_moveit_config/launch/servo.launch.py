import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro

def load_file(package_name, file_path):
    """A helper function to load a file from a package."""
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, "r") as file:
            return file.read()
    except EnvironmentError:
        return None

def load_yaml(package_name, file_path):
    """A helper function to load a YAML file from a package."""
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None

def generate_launch_description():
    # Define package and robot names
    moveit_config_pkg = 'my_moveit_config'
    trajectory_pkg = 'ros2_nr_motion_control' # Your C++ node's package
    ur_type = 'ur10e'

    # Load robot model & semantic description files
    robot_description_config = load_file(moveit_config_pkg, f'config/{ur_type}.urdf.xacro')
    robot_description = {"robot_description": robot_description_config}

    robot_description_semantic_config = load_file(moveit_config_pkg, f'config/{ur_type}.srdf')
    robot_description_semantic = {"robot_description_semantic": robot_description_semantic_config}

    # Load kinematics and Servo configuration files
    kinematics_yaml = load_yaml(moveit_config_pkg, 'config/kinematics.yaml')
    servo_yaml = load_yaml(moveit_config_pkg, 'config/servo.yaml')

    # Get paths to other launch and config files
    ur_simulation_gazebo_pkg_dir = get_package_share_directory('ur_simulation_gazebo')
    ur_sim_launch_file = os.path.join(ur_simulation_gazebo_pkg_dir, 'launch', 'ur_sim_control.launch.py')
    rviz_config_file = os.path.join(get_package_share_directory(moveit_config_pkg), "config", "moveit.rviz")

    # 1. Launch Gazebo Simulation with the UR10e
    gazebo_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ur_sim_launch_file),
        launch_arguments={'ur_type': ur_type, 'launch_rviz': 'False'}.items(),
    )

    # 2. Launch RViz with the MoveIt configuration
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[robot_description, robot_description_semantic, kinematics_yaml],
    )

    # 3. Launch the MoveIt Servo node
    servo_node = Node(
        package="moveit_servo",  # <-- FIX: Use the correct package name
        executable="servo_node_main",
        parameters=[
            servo_yaml,
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            {'use_sim_time': True},
        ],
        output="screen",
    )
    
    # 4. Launch your C++ trajectory publisher node
    trajectory_publisher_node = Node(
        package=trajectory_pkg,
        executable='helix_trajectory_publisher', # Use the correct executable name from your CMakeLists.txt
        name='cylinder_cleaning_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}]
    )

    return LaunchDescription([
        gazebo_sim_launch,
        rviz_node,
        # Use a TimerAction to delay the start of your custom nodes
        # This ensures Gazebo and the controllers are ready
        TimerAction(period=5.0, actions=[
            servo_node,
            # trajectory_publisher_node # Uncomment this when you want to run your trajectory node
        ])
    ])