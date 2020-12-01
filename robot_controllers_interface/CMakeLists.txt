cmake_minimum_required(VERSION 3.5)
project(robot_controllers_interface)

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

find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
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
  robot_controllers_msgs
)
target_compile_definitions(robot_controllers_interface PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

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

if(BUILD_TESTING)
  find_package(ament_cmake_cpplint)
  ament_cpplint(FILTERS "-whitespace/braces" "-whitespace/newline")
endif()

ament_export_include_directories(include)
ament_export_libraries(robot_controllers_interface)
ament_export_dependencies(
  pluginlib
  rclcpp
  robot_controllers_msgs
)
ament_package()
