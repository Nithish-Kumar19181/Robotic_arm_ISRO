cmake_minimum_required(VERSION 3.8)
project(ur10e_robot_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(urdf REQUIRED)

add_executable(ur10e_cylinder_follow src/ur10e_cylinder_follow.cpp)
add_executable(ur10e_helix_retract src/ur10e_helix_retract.cpp)

ament_target_dependencies(ur10e_cylinder_follow
  rclcpp
  geometry_msgs
  std_msgs
  sensor_msgs
  orocos_kdl
  kdl_parser
  urdf
  tf2_ros
)

ament_target_dependencies(ur10e_helix_retract
  rclcpp
  geometry_msgs
  std_msgs
  sensor_msgs
  orocos_kdl
  kdl_parser
  urdf
  tf2_ros
)

install(TARGETS
  ur10e_cylinder_follow
  ur10e_helix_retract
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
