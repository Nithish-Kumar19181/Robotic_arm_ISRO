cmake_minimum_required(VERSION 3.8)
project(ur10e_force_control_cylinder)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ur10e_force_control_base REQUIRED)

add_library(${PROJECT_NAME}_primitives SHARED
  src/primitives/approach.cpp
  src/primitives/circle.cpp
  src/primitives/helix.cpp
  src/primitives/retract.cpp
  src/robot_state_provider.cpp
)
target_include_directories(${PROJECT_NAME}_primitives PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}_primitives
  rclcpp
  geometry_msgs
  sensor_msgs
  std_msgs
  ur10e_force_control_base
)

add_executable(approach_node src/approach_node.cpp)
add_executable(circle_follow_node src/circle_follow_node.cpp)
add_executable(approach_circle_node src/approach_circle_node.cpp)

target_include_directories(approach_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_include_directories(circle_follow_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_include_directories(approach_circle_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_link_libraries(approach_node
  ${PROJECT_NAME}_primitives
)

target_link_libraries(circle_follow_node
  ${PROJECT_NAME}_primitives
)

target_link_libraries(approach_circle_node
  ${PROJECT_NAME}_primitives
)

ament_target_dependencies(approach_node
  rclcpp 
  geometry_msgs 
  sensor_msgs 
  std_msgs 
  ur10e_force_control_base
)

ament_target_dependencies(circle_follow_node
  rclcpp 
  geometry_msgs 
  sensor_msgs 
  std_msgs 
  ur10e_force_control_base
)

ament_target_dependencies(approach_circle_node
  rclcpp 
  geometry_msgs 
  sensor_msgs 
  std_msgs 
  ur10e_force_control_base
)

install(TARGETS ${PROJECT_NAME}_primitives
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS 
  approach_node
  circle_follow_node
  approach_circle_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  geometry_msgs
  sensor_msgs
  std_msgs
  ur10e_force_control_base
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
