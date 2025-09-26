from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Use this package's name
    this_pkg = FindPackageShare("path_planner")

    # Robot description
    description_file = PathJoinSubstitution(
        [this_pkg, "config", "ur10e_cartesian_simulation.urdf.xacro"]
    )
    
    robot_description_content = Command(
    [
        "bash -c '",
        FindExecutable(name="xacro"),
        " ",
        description_file,
        " 2>/dev/null", # This discards the stderr warning
        "'",
    ]
    )

    robot_description = {"robot_description": robot_description_content}

    # Controller configuration
    robot_controllers = PathJoinSubstitution([this_pkg, "config", "ur_controllers.yaml"])

    # ros2_control node for simulation
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="screen",
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # Spawner for the ACTIVE controllers
    active_spawners = [
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[controller],
        )
        for controller in ["joint_state_broadcaster", "scaled_joint_trajectory_controller"]
    ]

    # Spawner for the INACTIVE Cartesian controllers
    inactive_spawners = [
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[controller, "--inactive"],
        )
        for controller in ["cartesian_motion_controller", "motion_control_handle"]
    ]
    
    # RViz
    rviz_config = PathJoinSubstitution([this_pkg, "config", "control.rviz"])
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
    )

    nodes_to_start = [control_node, robot_state_publisher, rviz] + active_spawners + inactive_spawners
    
    return LaunchDescription(nodes_to_start)