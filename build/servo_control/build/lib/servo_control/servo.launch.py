from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    robot_description = Command([
        'xacro ',
        PathJoinSubstitution([
            FindPackageShare('my_robot_moveit_config'),
            'urdf',
            'my_robot.urdf.xacro'
        ])
    ])

    return LaunchDescription([
        Node(
            package='moveit_servo',
            executable='servo_node',
            name='moveit_servo',
            output='screen',
            parameters=[
                {'use_sim_time': True},  # if running in Gazebo
                {'robot_description': robot_description},
                PathJoinSubstitution([FindPackageShare('my_robot_moveit_config'), 'config', 'servo.yaml']),
                PathJoinSubstitution([FindPackageShare('my_robot_moveit_config'), 'config', 'kinematics.yaml']),
                PathJoinSubstitution([FindPackageShare('my_robot_moveit_config'), 'config', 'ros2_controllers.yaml']),
                PathJoinSubstitution([FindPackageShare('my_robot_moveit_config'), 'config', 'moveit_controllers.yaml'])
            ]
        ),
    ])
