from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os
import xacro
from launch.substitutions import Command
from launch.substitutions import PathJoinSubstitution, Command, FindExecutable
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    ur_type = 'ur10e'

    robot_description_path = os.path.join(
        get_package_share_directory('ur10e_testing_package'),
        'urdf',
        f'{ur_type}.urdf.xacro'
    )

    urdf_path = "/home/nithish/isro_arm_ws/src/ur10e_testing_package/urdf/ur10e.urdf.xacro"


    robot_description = Command([
        "xacro ", urdf_path
    ])

    robot_description_config = xacro.process_file(robot_description_path)
    robot_description_content = robot_description_config.toxml()
   
    ur_simulation_gazebo_pkg_dir = get_package_share_directory('ur_simulation_gazebo')
    ur_sim_launch_file = os.path.join(ur_simulation_gazebo_pkg_dir, 'launch', 'ur_sim_control.launch.py')
    
    # Wall SDF file path
    # bowl_sdf_path = "/home/nithish/ur10e_ws/src/ros2_nr_motion_control/model/bowl.sdf"
    
    rviz_config_file = os.path.join(
        get_package_share_directory('ur10e_testing_package'),
        "rviz",
        "view_robot.rviz"
    )

    return LaunchDescription([

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ur_sim_launch_file),
            launch_arguments={
                'ur_type': ur_type,
                'launch_rviz': 'True', 
            }.items(),
        ),
        
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[{
                "robot_description": robot_description,
                "publish_frequency": 50.0  # Match your IK publishing rate
            }]
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="log",
            arguments=["-d", rviz_config_file],
        )

    ])