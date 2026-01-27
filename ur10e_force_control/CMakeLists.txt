cmake_minimum_required(VERSION 3.8)
project(ur10e_force_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(sensor_msgs REQUIRED) 
find_package(kdl_parser REQUIRED)
find_package(orocos_kdl REQUIRED)
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


add_executable(wall_detection_node src/wall_detection_node.cpp)
add_executable(admittance_control_node src/admittance_control_node.cpp)
add_executable(lifecycle_manager_node src/lifecycle_manager_node.cpp)
add_executable(marker_array_node src/marker_array_node.cpp)
add_executable(cartesian_lift_and_reverse_circle_node src/cartesian_lift_and_reverse_circle_node.cpp)

ament_target_dependencies(wall_detection_node
  rclcpp
  std_msgs
  rclcpp_lifecycle
  lifecycle_msgs
  trajectory_msgs
  urdf
  kdl_parser
  sensor_msgs
  visualization_msgs
  rclcpp_lifecycle
  tf2_ros
  tf2_geometry_msgs
)

ament_target_dependencies(admittance_control_node
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
  std_msgs
  lifecycle_msgs
  control_msgs
  urdf
  kdl_parser
)

ament_target_dependencies(marker_array_node
  rclcpp
  tf2_ros
  visualization_msgs
)

ament_target_dependencies(cartesian_lift_and_reverse_circle_node
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

install(TARGETS 
  wall_detection_node
  admittance_control_node
  lifecycle_manager_node
  marker_array_node
  cartesian_lift_and_reverse_circle_node
  DESTINATION lib/${PROJECT_NAME}
)

target_link_libraries(wall_detection_node
  ${orocos_kdl_LIBRARIES}
  Eigen3::Eigen
)

target_link_libraries(admittance_control_node
  ${orocos_kdl_LIBRARIES}
  Eigen3::Eigen
)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
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
