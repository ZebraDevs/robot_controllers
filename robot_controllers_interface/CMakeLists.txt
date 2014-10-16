cmake_minimum_required(VERSION 2.8.3)
project(robot_controllers_interface)

find_package(Boost REQUIRED system)
find_package(catkin REQUIRED
  COMPONENTS
    pluginlib
    roscpp
)

catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    pluginlib
    roscpp
  DEPENDS
    Boost
)

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)
