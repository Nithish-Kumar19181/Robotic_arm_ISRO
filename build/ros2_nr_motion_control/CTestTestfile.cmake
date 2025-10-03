# CMake generated Testfile for 
# Source directory: /home/nithish/ur10e_ws/src/ros2_nr_motion_control
# Build directory: /home/nithish/ur10e_ws/build/ros2_nr_motion_control
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(test_robot_arm_motion_planner "/usr/bin/python3" "-u" "/opt/ros/humble/share/ament_cmake_test/cmake/run_test.py" "/home/nithish/ur10e_ws/build/ros2_nr_motion_control/test_results/ros2_nr_motion_control/test_robot_arm_motion_planner.gtest.xml" "--package-name" "ros2_nr_motion_control" "--output-file" "/home/nithish/ur10e_ws/build/ros2_nr_motion_control/ament_cmake_gtest/test_robot_arm_motion_planner.txt" "--command" "/home/nithish/ur10e_ws/build/ros2_nr_motion_control/test_robot_arm_motion_planner" "--gtest_output=xml:/home/nithish/ur10e_ws/build/ros2_nr_motion_control/test_results/ros2_nr_motion_control/test_robot_arm_motion_planner.gtest.xml")
set_tests_properties(test_robot_arm_motion_planner PROPERTIES  LABELS "functional" REQUIRED_FILES "/home/nithish/ur10e_ws/build/ros2_nr_motion_control/test_robot_arm_motion_planner" TIMEOUT "60" WORKING_DIRECTORY "/home/nithish/ur10e_ws/build/ros2_nr_motion_control" _BACKTRACE_TRIPLES "/opt/ros/humble/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/humble/share/ament_cmake_gtest/cmake/ament_add_gtest_test.cmake;86;ament_add_test;/opt/ros/humble/share/ament_cmake_gtest/cmake/ament_add_gtest.cmake;93;ament_add_gtest_test;/home/nithish/ur10e_ws/src/ros2_nr_motion_control/CMakeLists.txt;82;ament_add_gtest;/home/nithish/ur10e_ws/src/ros2_nr_motion_control/CMakeLists.txt;0;")
subdirs("gtest")
