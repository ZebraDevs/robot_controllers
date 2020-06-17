cmake_minimum_required(VERSION 3.5)
project(robot_controllers)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(orocos_kdl REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(angles REQUIRED)
find_package(control_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(robot_controllers_interface REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(urdf REQUIRED)

include_directories(
  include
  SYSTEM ${orocos_kdl_INCLUDE_DIRS}
)

add_library(robot_controllers SHARED
  src/cartesian_pose.cpp
  src/cartesian_twist.cpp
  src/cartesian_wrench.cpp
  src/diff_drive_base.cpp
  src/follow_joint_trajectory.cpp
  src/gravity_compensation.cpp
  src/parallel_gripper.cpp
  src/pid.cpp
  src/point_head.cpp
  src/scaled_mimic.cpp
)
target_link_libraries(robot_controllers
  ${orocos_kdl_LIBRARIES}
)
ament_target_dependencies(robot_controllers
  control_msgs
  geometry_msgs
  kdl_parser
  nav_msgs
  pluginlib
  rclcpp
  rclcpp_action
  robot_controllers_interface
  sensor_msgs
  tf2_geometry_msgs
  tf2_ros
  trajectory_msgs
  urdf
)

if (BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(${PROJECT_NAME}-trajectory-tests test/trajectory_tests.cpp)
  ament_target_dependencies(${PROJECT_NAME}-trajectory-tests
    angles
    rclcpp
    robot_controllers_interface
  )

  find_package(ament_cmake_cpplint)
  ament_cpplint(FILTERS "-whitespace/braces" "-whitespace/newline")
endif()

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS robot_controllers
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/robot_controllers
)

pluginlib_export_plugin_description_file(robot_controllers_interface robot_controllers.xml)

ament_export_include_directories(include)
ament_export_libraries(robot_controllers)
ament_export_dependencies(
  control_msgs
  geometry_msgs
  kdl_parser
  nav_msgs
  pluginlib
  rclcpp
  rclcpp_action
  robot_controllers_interface
  sensor_msgs
  tf2_geometry_msgs
  tf2_ros
  trajectory_msgs
  urdf
)
ament_package()
