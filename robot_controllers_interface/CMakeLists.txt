cmake_minimum_required(VERSION 2.8.3)
project(robot_controllers_interface)

find_package(Boost REQUIRED system)
find_package(catkin REQUIRED
  COMPONENTS
    actionlib
    pluginlib
    roscpp
    robot_controllers_msgs
)

catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    actionlib
    pluginlib
    roscpp
    robot_controllers_msgs
  DEPENDS
    Boost
  LIBRARIES
    robot_controllers_interface
)

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_library(robot_controllers_interface
  src/controller_loader.cpp
  src/controller_manager.cpp
)
target_link_libraries(robot_controllers_interface
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
)
add_dependencies(robot_controllers_interface
  robot_controllers_msgs_gencpp
)

install(DIRECTORY include/ DESTINATION include)

install(TARGETS robot_controllers_interface LIBRARY
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
