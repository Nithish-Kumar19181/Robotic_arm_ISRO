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
find_package(robot_task_interfaces REQUIRED)


add_executable(ur10e_cylinder_follow src/ur10e_cylinder_follow.cpp)
add_executable(ur10e_helix_retract src/ur10e_helix_retract.cpp)
add_executable(action_circle_follow src/action_circle_follow.cpp)
add_executable(action_retract src/action_retract.cpp)
add_executable(action_task_manager src/action_task_manager)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

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


ament_target_dependencies(action_circle_follow
  rclcpp
  rclcpp_action
  robot_task_interfaces
  geometry_msgs
  sensor_msgs
  std_msgs
  urdf
  kdl_parser
  orocos_kdl
)

ament_target_dependencies(action_retract
  rclcpp
  rclcpp_action
  robot_task_interfaces
  geometry_msgs
  sensor_msgs
  std_msgs
  urdf
  kdl_parser
  orocos_kdl
)

ament_target_dependencies(action_task_manager
  rclcpp
  rclcpp_action
  robot_task_interfaces
)


install(TARGETS
  ur10e_cylinder_follow
  ur10e_helix_retract
  action_circle_follow
  action_retract
  action_task_manager
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
