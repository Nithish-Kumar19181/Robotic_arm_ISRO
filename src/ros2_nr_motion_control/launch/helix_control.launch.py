from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os
import xacro

def generate_launch_description():

    # === Load the robot's URDF model ===
    # This is the most reliable way to get the robot_description
    ur_type = 'ur10e'
    robot_description_path = os.path.join(
        get_package_share_directory('ur_description'),
        'urdf',
        f'{ur_type}.urdf.xacro'
    )
    robot_description_config = xacro.process_file(robot_description_path)
    robot_description_params = {'robot_description': robot_description_config.toxml()}

    # === Get paths to other necessary files ===
    ur_simulation_gazebo_pkg_dir = get_package_share_directory('ur_simulation_gazebo')
    ur_sim_launch_file = os.path.join(ur_simulation_gazebo_pkg_dir, 'launch', 'ur_sim_control.launch.py')
    rviz_config_file = os.path.join(
        get_package_share_directory('ros2_nr_motion_control'), # CHANGE THIS to your package name
        "rviz", 
        "view_robot.rviz"
    )

    return LaunchDescription([

        # 1. Launch the Gazebo simulation with the UR10e robot
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ur_sim_launch_file),
            launch_arguments={
                'ur_type': ur_type,
                'launch_rviz': 'False'  # We launch our own RViz instance
            }.items(),
        ),

        # 2. Launch RViz for visualization
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="log",
            arguments=["-d", rviz_config_file],
        ),

        # 3. Delay the start of your node to allow Gazebo and controllers to fully load
        TimerAction(
            period=5.0, # 5-second delay
            actions=[
                Node(
                    package='ros2_nr_motion_control', # CHANGE THIS to your package name
                    # IMPORTANT: Ensure this executable name matches your CMakeLists.txt
                    executable='helix_trajectory_publisher', 
                    name='helix_trajectory_publisher',
                    output='screen',
                    parameters=[
                        robot_description_params, # Pass the loaded robot description
                        {'use_sim_time': True}   # Use the simulation clock
                    ]
                )
            ]
        )
    ])