cmake_minimum_required(VERSION 3.8)
project(ur10e_testing_package)

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

add_executable(ur10e_straight_line src/ur10e_straight_line.cpp)

add_executable(ur10e_home src/ur10e_home.cpp)

add_executable(ur10e_ground_plane src/ur10e_ground_plane.cpp)

add_executable(joint_angle_publisher src/joint_angle_publisher.cpp)

add_executable(ur10e_force_retract src/ur10e_force_retract.cpp)

add_executable(ur10e_stiffness_detection src/ur10e_stiffness_detection.cpp)

install(
  DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(
  DIRECTORY rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
)

install(
  DIRECTORY model/
  DESTINATION share/${PROJECT_NAME}/model
)

install(
  DIRECTORY include/
  DESTINATION share/${PROJECT_NAME}/include
)

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

ament_target_dependencies(ur10e_home
  rclcpp
  std_msgs
  orocos_kdl
  trajectory_msgs
  urdf
  kdl_parser
  sensor_msgs
  moveit_core
  moveit_ros_planning_interface
  moveit_ros_planning
  tf2_geometry_msgs
  visualization_msgs
)

ament_target_dependencies(ur10e_ground_plane
  rclcpp
  std_msgs
  moveit_msgs
  moveit_core
  moveit_ros_planning_interface
  moveit_ros_planning
)

ament_target_dependencies(joint_angle_publisher
  rclcpp 
  rclcpp_action 
  control_msgs 
  trajectory_msgs 
  sensor_msgs
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

ament_target_dependencies(ur10e_stiffness_detection
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

install(TARGETS 
  ur10e_straight_line
  ur10e_home
  ur10e_ground_plane
  joint_angle_publisher
  ur10e_force_retract
  ur10e_stiffness_detection
  DESTINATION lib/${PROJECT_NAME}
)

target_include_directories(ur10e_straight_line PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)

target_include_directories(ur10e_home PUBLIC
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
