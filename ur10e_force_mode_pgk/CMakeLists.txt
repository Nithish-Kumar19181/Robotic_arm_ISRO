cmake_minimum_required(VERSION 3.8)
project(ur10e_force_mode_pgk)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(ur_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ur_controllers REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_core REQUIRED)
find_package(srdfdom REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(controller_manager_msgs REQUIRED)


add_executable(ur_force_mode_wall_finder
  src/ur_force_mode_wall_finder.cpp
)

add_executable(ur_force_mode_wall_follower
  src/ur_force_mode_wall_follower.cpp
)

add_executable(ur_force_mode_helix 
  src/ur_force_mode_helix.cpp
)

add_executable(ur_force_mode_wall_follower_loop
  src/ur_force_mode_wall_follower_loop.cpp
)

add_executable(wall_follower_lifecycle
  src/wall_follower_life_cycle.cpp
)

add_executable(wall_finder_life_cycle
  src/wall_finder_life_cycle.cpp
)

add_executable(life_cycle_manager_node
  src/life_cycle_manager_node.cpp
)

add_executable(force_mode_wall_seek_orientation_control
  src/force_mode_wall_seek_orientation_control.cpp
)

ament_target_dependencies(ur_force_mode_wall_finder
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  ur_controllers
  orocos_kdl
  kdl_parser
  urdf
  srdfdom
  moveit_ros_planning_interface
  moveit_core
  tf2_ros
  tf2_geometry_msgs
  controller_manager_msgs
)

ament_target_dependencies(ur_force_mode_wall_follower
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  ur_controllers
  orocos_kdl
  kdl_parser
  urdf
  srdfdom
  moveit_ros_planning_interface
  moveit_core
  tf2_ros
  tf2_geometry_msgs
  controller_manager_msgs
)

ament_target_dependencies(ur_force_mode_helix
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  ur_controllers
  orocos_kdl
  kdl_parser
  urdf
  srdfdom
  moveit_ros_planning_interface
  moveit_core
  tf2_ros
  tf2_geometry_msgs
  controller_manager_msgs
)

ament_target_dependencies(ur_force_mode_wall_follower_loop
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  ur_controllers
  orocos_kdl
  kdl_parser
  urdf
  srdfdom
  moveit_ros_planning_interface
  moveit_core
  tf2_ros
  tf2_geometry_msgs
  controller_manager_msgs
)

ament_target_dependencies(wall_follower_lifecycle
 rclcpp
  rclcpp_lifecycle
  kdl_parser
  std_msgs
  sensor_msgs
  trajectory_msgs
  controller_manager_msgs
  std_srvs
  ur_msgs
  geometry_msgs
  lifecycle_msgs
  urdf
  orocos_kdl
)


ament_target_dependencies(wall_finder_life_cycle
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  ur_controllers
  orocos_kdl
  kdl_parser
  urdf
  srdfdom
  moveit_ros_planning_interface
  moveit_core
  tf2_ros
  tf2_geometry_msgs
  controller_manager_msgs
)

ament_target_dependencies(life_cycle_manager_node
  rclcpp
  rclcpp_lifecycle
  lifecycle_msgs
  std_msgs
  sensor_msgs
  geometry_msgs
  controller_manager_msgs
  std_srvs
  ur_msgs
  urdf
  orocos_kdl
  kdl_parser
)

ament_target_dependencies(force_mode_wall_seek_orientation_control
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  controller_manager_msgs
  std_srvs
  ur_msgs
  urdf
  orocos_kdl
  kdl_parser
)


install(TARGETS
  ur_force_mode_wall_finder
  ur_force_mode_wall_follower
  ur_force_mode_helix
  ur_force_mode_wall_follower_loop
  wall_follower_lifecycle
  wall_finder_life_cycle
  life_cycle_manager_node
  force_mode_wall_seek_orientation_control
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
