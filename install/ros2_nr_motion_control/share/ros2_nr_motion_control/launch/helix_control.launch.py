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
    
    ur_robot_driver_pkg_dir = get_package_share_directory('ur_robot_driver')
    ur_driver_launch_file = os.path.join(ur_robot_driver_pkg_dir,'launch','ur_control.launch.py')
    
    rviz_config_file = os.path.join(
        get_package_share_directory('ros2_nr_motion_control'),
        "rviz",
        "view_robot.rviz"
    )

    # Wall SDF model
    wall_sdf = """
    <?xml version="1.0" ?>
    <sdf version="1.6">
      <model name="wall">
        <static>true</static>
        <link name="wall_link">
          <collision name="collision">
            <geometry>
              <box>
                <size>0.1 2.0 2.0</size>
              </box>
            </geometry>
          </collision>
          <visual name="visual">
            <geometry>
              <box>
                <size>0.1 2.0 2.0</size>
              </box>
            </geometry>
            <material>
              <ambient>0.8 0.2 0.2 1.0</ambient>
              <diffuse>0.8 0.2 0.2 1.0</diffuse>
            </material>
          </visual>
        </link>
      </model>
    </sdf>
    """

    return LaunchDescription([

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ur_sim_launch_file),
            launch_arguments={
                'ur_type': ur_type,
                'launch_rviz': 'False', 
            }.items(),
        ),

        # Spawn wall in Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_wall',
            output='screen',
            arguments=[
                '-entity', 'wall',
                '-x', '1.0',  # Position in front of robot
                '-y', '0.0',
                '-z', '1.0',
                '-file', '/tmp/wall.sdf'
            ],
            parameters=[{'sdf': wall_sdf}]
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="log",
            arguments=["-d", rviz_config_file],
        ),

        TimerAction(
            period=2.0,  # Wait for simulation to start
            actions=[
                Node(
                    package='ros2_nr_motion_control',
                    executable='joint_angle_publisher',
                    name='joint_angle_publisher',
                    output='screen',
                ),

                Node(
                    package='ros2_nr_motion_control',
                    executable='joint_sine_publisher',
                    name='joint_sine_publisher',
                    output='screen',
                )
            ]
        )
    ])