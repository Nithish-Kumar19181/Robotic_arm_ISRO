cmake_minimum_required(VERSION 3.8)
project(ur10e_simulation_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

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
find_package(rclcpp_action REQUIRED)

add_executable(ur10e_circle_follow src/ur10e_circle_follow.cpp)
add_executable(ur10e_retract src/ur10e_retract.cpp)
add_executable(action_circle_follow src/action_circle_follow.cpp)
add_executable(action_retract src/action_retract.cpp)
add_executable(action_task_manager src/action_task_manager.cpp)
add_executable(ur10e_reverse_circle src/ur10e_reverse_circle.cpp)

install(
  DIRECTORY include/
  DESTINATION share/${PROJECT_NAME}/include
)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(
  DIRECTORY model/
  DESTINATION share/${PROJECT_NAME}/model
)
ament_target_dependencies(ur10e_circle_follow
  rclcpp
  geometry_msgs
  tf2
  sensor_msgs
  urdf
  orocos_kdl
  kdl_parser
)

ament_target_dependencies(ur10e_retract
  rclcpp
  geometry_msgs
  tf2
  sensor_msgs
  urdf
  kdl_parser
  orocos_kdl
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

ament_target_dependencies(ur10e_reverse_circle
  rclcpp
  rclcpp_action
  geometry_msgs
  sensor_msgs
  std_msgs
  urdf
  kdl_parser
  orocos_kdl
)  

install(TARGETS
  ur10e_circle_follow
  ur10e_retract
  action_circle_follow
  action_retract
  action_task_manager
  ur10e_reverse_circle
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
