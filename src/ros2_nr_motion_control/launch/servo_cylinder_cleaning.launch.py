import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue # <-- 1. ADD THIS IMPORT

def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument( "ur_type", default_value="ur10e", description="Type/series of used UR robot."),
    ]
    ur_type = LaunchConfiguration("ur_type")
    
    # --- 1. GET ALL CONFIGS AND DESCRIPTIONS ---

    # Load the physical robot description (URDF)
    robot_description_content = ParameterValue(
        Command(
            [
                PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
                PathJoinSubstitution(
                    [FindPackageShare("ros2_nr_motion_control"), "config", "ur10e.urdf.xacro"]
                ),
            ]
        ),
        value_type=str
    )
    robot_description = {"robot_description": robot_description_content}

    # Load your local semantic robot description (SRDF)
    robot_description_semantic_content = ParameterValue( # <-- 3. WRAP THE COMMAND
        Command(
            [
                PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
                PathJoinSubstitution(
                    [FindPackageShare("ros2_nr_motion_control"), "config", "ur10e.srdf"]
                ),
            ]
        ),
        value_type=str
    )
    robot_description_semantic = {"robot_description_semantic": robot_description_semantic_content}
    
    # Get Kinematics & Servo configs
    kinematics_yaml = PathJoinSubstitution(
        [FindPackageShare("ur_moveit_config"), "config", ur_type, "kinematics.yaml"]
    )
    servo_config = PathJoinSubstitution(
        [FindPackageShare("ros2_nr_motion_control"), "config", "ur_servo.yaml"]
    )
    
    # --- 2. LAUNCH CORE COMPONENTS ---
    ur_simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ur_simulation_gazebo'), 'launch', 'ur_sim_control.launch.py')
        ),
        launch_arguments={'ur_type': ur_type, 'launch_rviz': 'true', 'use_sim_time': 'true'}.items(),
    )
    
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("ur_moveit_config"), "launch", "ur_moveit.launch.py")
        ),
        launch_arguments={ "ur_type": ur_type, "use_sim_time": "true", "launch_rviz": "false"}.items(),
    )

    # --- 3. LAUNCH MOVEIT SERVO NODE ---
    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        parameters=[
            robot_description,
            robot_description_semantic,
            {"robot_description_kinematics": kinematics_yaml},
            {"moveit_servo": servo_config},
        ],
        output="screen",
    )
    
    # --- 4. LAUNCH THE CARTESIAN GOAL PUBLISHER ---
    cartesian_goal_publisher_node = Node(
        package='ros2_nr_motion_control',
        executable='cartesian_goal_publisher',
        name='cartesian_goal_publisher',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # --- 5. USE TIMER ACTIONS FOR STARTUP ORDER ---
    delayed_servo_node = TimerAction(period=10.0, actions=[servo_node])
    delayed_goal_publisher = TimerAction(period=15.0, actions=[cartesian_goal_publisher_node])

    return LaunchDescription(
        declared_arguments
        + [
            ur_simulation_launch,
            moveit_launch,
            delayed_servo_node,
            delayed_goal_publisher,
        ]
    )