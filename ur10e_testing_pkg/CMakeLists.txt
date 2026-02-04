cmake_minimum_required(VERSION 3.8)
project(ur10e_testing_pkg)

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

add_executable(ur10e_home src/ur10e_home.cpp)
add_executable(ur10e_force_filter src/ur10e_force_filter.cpp)
add_executable(ur10e_st_line src/ur10e_st_line.cpp)
add_executable(ur10e_marker_array src/ur10e_marker_array.cpp)

install(
  DIRECTORY include/
  DESTINATION share/${PROJECT_NAME}/include
)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_target_dependencies(ur10e_home
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  tf2
)

ament_target_dependencies(ur10e_force_filter
  rclcpp
  geometry_msgs
  tf2
)

ament_target_dependencies(ur10e_st_line
  rclcpp
  sensor_msgs
  geometry_msgs
  kdl_parser
  urdf
)

ament_target_dependencies(ur10e_marker_array
  rclcpp
  visualization_msgs
  tf2_ros
)


install(TARGETS
  ur10e_home
  ur10e_force_filter
  ur10e_st_line
  ur10e_marker_array
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

# colcon build \
#   --packages-skip cartesian_controller_simulation cartesian_controller_tests \
#   --cmake-args -DCMAKE_BUILD_TYPE=Release \
#   --executor sequential


# ros2 launch ur10e_testing_pkg ur10e_home.launch.py
# ros2 launch ur10e_testing_pkg ur10e_force_filter.launch.py
# ros2 launch ur_robot_driver ur10e_robot_driver.launch.py robot_ip:=172.17.0.2 ur_type:=ur10e initial_joint_controller:=joint_trajectory_controller use_fake_hardware:=true

