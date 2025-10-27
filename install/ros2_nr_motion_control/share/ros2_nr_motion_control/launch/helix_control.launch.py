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
    
    rviz_config_file = os.path.join(
        get_package_share_directory('ros2_nr_motion_control'),
        "rviz",
        "view_robot.rviz"
    )

    # Wall SDF (simple box obstacle)
    wall_sdf = os.path.join(
        get_package_share_directory('ros2_nr_motion_control'),
        'models',
        'wall.sdf'
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ur_sim_launch_file),
            launch_arguments={
                'ur_type': ur_type,
                'launch_rviz': 'False',
            }.items(),
        ),

        # Spawn a solid wall model in Gazebo at x = 0.5 m
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-file', wall_sdf, '-entity', 'solid_wall', '-x', '0.5', '-y', '0.0', '-z', '0.4'],
            output='screen'
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="log",
            arguments=["-d", rviz_config_file],
        ),

        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='ros2_nr_motion_control',
                    executable='force_react_node',
                    name='force_react_node',
                    output='screen',
                ),
            ]
        )
    ])
