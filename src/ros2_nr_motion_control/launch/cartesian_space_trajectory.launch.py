from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os
import xacro

def generate_launch_description():

    ur_type = 'ur10e'

    robot_description_path = os.path.join(
        get_package_share_directory('my_moveit_config'),
        'config',
        f'{ur_type}.urdf.xacro'
    )

    robot_description_config = xacro.process_file(robot_description_path)
    robot_description_content = robot_description_config.toxml()

    ur_simulation_gazebo_pkg_dir = get_package_share_directory('ur_simulation_gazebo')
    ur_sim_launch_file = os.path.join(ur_simulation_gazebo_pkg_dir, 'launch', 'ur_sim_control.launch.py')
    
    # Wall SDF file path
    wall_sdf_path = "/home/nithish/ur10e_ws/src/ros2_nr_motion_control/model/wall.sdf"
    
    rviz_config_file = os.path.join(
        get_package_share_directory('ros2_nr_motion_control'),
        "rviz",
        "view_robot.rviz"
    )

    return LaunchDescription([

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ur_sim_launch_file),
            launch_arguments={
                'ur_type': ur_type,
                'launch_rviz': 'False', 
            }.items(),
        ),

        # Spawn the wall in Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'wall',
                '-file', wall_sdf_path,
                '-x', '0.8',  # Position the wall 1.5m away in x direction
                '-y', '0.0',
                '-z', '0.5',
                '-Y', '0.0'
            ],
            output='screen',
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="log",
            arguments=["-d", rviz_config_file],
        ),

        TimerAction(
            period=2.0,  # Wait 2 seconds for everything to start
            actions=[
                Node(
                    package='ros2_nr_motion_control',
                    executable='joint_angle_publisher',
                    name='joint_angle_publisher',
                    output='screen',
                ),

                Node(
                    package='ros2_nr_motion_control',
                    executable='cartesian_space_trajectory_node',
                    name='cartesian_space_trajectory_node',
                    output='screen',
                )
            ]
        )
    ])