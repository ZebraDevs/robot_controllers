cmake_minimum_required(VERSION 3.5)
project(robot_controllers_interface)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if (CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
endif()

find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(robot_controllers_msgs REQUIRED)

include_directories(
  include
)

add_library(robot_controllers_interface SHARED
  src/controller_loader.cpp
  src/controller_manager.cpp
)
ament_target_dependencies(robot_controllers_interface
  pluginlib
  rclcpp
  rclcpp_action
  robot_controllers_msgs
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS robot_controllers_interface
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/robot_controllers_interface
)

install(
  PROGRAMS
    scripts/get_controller_state.py
    scripts/start_controller.py
    scripts/stop_controller.py
  DESTINATION
    lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(robot_controllers_interface)
ament_export_dependencies(
  pluginlib
  rclcpp
  rclcpp_action
  robot_controllers_msgs
)
ament_package()
