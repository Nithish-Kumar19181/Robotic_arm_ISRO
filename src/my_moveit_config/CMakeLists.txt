cmake_minimum_required(VERSION 3.22)
project(my_moveit_config)

find_package(ament_cmake REQUIRED)

# Install launch files
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
    PATTERN "setup_assistant.launch" EXCLUDE)
endif()

# Install config files
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

# Install setup assistant file
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})

ament_package()   # <–– keep this at the very bottom
