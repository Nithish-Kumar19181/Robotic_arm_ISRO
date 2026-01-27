cmake_minimum_required(VERSION 3.8)
project(ur10e_robot_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(sensor_msgs REQUIRED) 
find_package(kdl_parser REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(Eigen3 REQUIRED) 
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)  
find_package(moveit_ros_planning REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

add_executable(ur10e_straight_line src/ur10e_straight_line.cpp)
add_executable(ur10e_force_retract src/ur10e_force_retract.cpp)
add_executable(ur10e_wall_detection_node src/ur10e_wall_detection_node.cpp)
add_executable(ur10e_admittance_control_node src/ur10e_admittance_control_node.cpp)
add_executable(lifecycle_manager_node src/lifecycle_manager_node.cpp)
add_executable(ur10e_cartesian_lift_and_reverse_circle_node src/ur10e_cartesian_lift_and_reverse_circle_node.cpp)
add_executable(scaled_jtc_test src/scaled_jtc_test.cpp)

ament_target_dependencies(ur10e_straight_line
  rclcpp
  std_msgs
  orocos_kdl
  trajectory_msgs
  urdf
  kdl_parser
  Eigen3
  sensor_msgs
  visualization_msgs
  tf2_ros
  tf2_geometry_msgs
)

ament_target_dependencies(ur10e_force_retract
  rclcpp
  std_msgs
  orocos_kdl
  trajectory_msgs
  urdf
  kdl_parser
  Eigen3
  sensor_msgs
  visualization_msgs
  tf2_ros
  tf2_geometry_msgs
)

ament_target_dependencies(ur10e_wall_detection_node
  rclcpp
  moveit_core
  moveit_ros_planning_interface
  moveit_ros_planning
  geometry_msgs
  control_msgs
  std_srvs
  trajectory_msgs
)

ament_target_dependencies(ur10e_admittance_control_node
  rclcpp
  rclcpp_lifecycle
  lifecycle_msgs
  std_msgs
  trajectory_msgs
  urdf
  kdl_parser
  sensor_msgs
  visualization_msgs
  tf2_ros
  tf2_geometry_msgs
)

ament_target_dependencies(lifecycle_manager_node
  rclcpp
  rclcpp_lifecycle
  lifecycle_msgs
  std_msgs
  trajectory_msgs
  urdf
  kdl_parser
  sensor_msgs
  visualization_msgs
  tf2_ros
  tf2_geometry_msgs
)

ament_target_dependencies(ur10e_admittance_control_node
  rclcpp
  std_msgs
  lifecycle_msgs
  control_msgs
)

ament_target_dependencies(ur10e_cartesian_lift_and_reverse_circle_node
  rclcpp
  rclcpp_lifecycle
  lifecycle_msgs
  std_msgs
  trajectory_msgs
  urdf
  kdl_parser
  sensor_msgs
  visualization_msgs
  tf2_ros
  tf2_geometry_msgs
)

ament_target_dependencies(scaled_jtc_test
  rclcpp
  rclcpp_lifecycle
  lifecycle_msgs
  std_msgs
  trajectory_msgs
  urdf
  kdl_parser
  sensor_msgs
  visualization_msgs
  tf2_ros
  tf2_geometry_msgs
  nav_msgs
)

install(TARGETS 
  ur10e_straight_line
  ur10e_force_retract
  ur10e_wall_detection_node
  ur10e_admittance_control_node
  lifecycle_manager_node
  ur10e_cartesian_lift_and_reverse_circle_node
  scaled_jtc_test
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

target_include_directories(ur10e_straight_line PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
