import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    pkg_share = get_package_share_directory('ur10e_simulation_pkg')

    urdf_path = os.path.join(pkg_share, 'urdf', 'vertical_mixer_planetary.urdf')
    controllers_yaml = os.path.join(pkg_share, 'config', 'controllers.yaml')

    # Read URDF and inject the controllers.yaml absolute path at launch time
    # (replaces the CONTROLLERS_YAML_PLACEHOLDER token inside the URDF)
    with open(urdf_path, 'r', encoding='utf-8') as f:
        robot_description = f.read().replace(
            'CONTROLLERS_YAML_PLACEHOLDER', controllers_yaml)

    # ── Optional GUI launch argument ─────────────────────────────────────
    # Pass gui:=false to run Gazebo headless (prevents gzclient OOM crashes
    # on systems with limited GPU memory or software rendering).
    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Launch the Gazebo GUI client (set false for headless)')

    # ── ROBOT STATE PUBLISHER ────────────────────────────────────────────
    state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )

    # ── GUI (SUN JOINT CONTROL) ──────────────────────────────────────────
    joint_state_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        remappings=[('/joint_states', '/joint_states_gui')],
    )

    # ── KINEMATICS NODE ──────────────────────────────────────────────────
    kinematics_node = Node(
        package='ur10e_simulation_pkg',
        executable='planetary_kinematics_node.py',
        output='screen',
    )

    # ── GAZEBO ───────────────────────────────────────────────────────────
    # gui LaunchConfiguration controls whether gzclient starts.
    # Using separate gzserver + gzclient (via gazebo.launch.py gui arg)
    # avoids a crash when there is no GPU / display available.
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py',
            ])
        ),
        launch_arguments={'gui': LaunchConfiguration('gui')}.items(),
    )

    # ── SPAWN ROBOT ──────────────────────────────────────────────────────
    spawn_entity = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-topic', 'robot_description',
                    '-entity', 'planetary_mixer',
                ],
                output='screen',
            )
        ],
    )

    # ── CONTROLLERS ──────────────────────────────────────────────────────
    load_joint_state_broadcaster = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=[
                    'joint_state_broadcaster',
                    '--controller-manager', '/controller_manager',
                ],
                output='screen',
            )
        ],
    )

    load_forward_controller = TimerAction(
        period=6.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=[
                    'forward_position_controller',
                    '--controller-manager', '/controller_manager',
                ],
                output='screen',
            )
        ],
    )

    # ── RVIZ ─────────────────────────────────────────────────────────────
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
    )

    # ── FINAL LAUNCH ─────────────────────────────────────────────────────
    return LaunchDescription([
        gui_arg,
        state_publisher,
        joint_state_gui,
        kinematics_node,
        gazebo,
        spawn_entity,
        load_joint_state_broadcaster,
        load_forward_controller,
        rviz_node,
    ])
