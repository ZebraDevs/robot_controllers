find_package(rostest)

catkin_add_gtest(trajectory_tests trajectory_tests.cpp)
target_link_libraries(trajectory_tests
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
)

catkin_add_gtest(linear_lookup_table_test linear_lookup_table_test.cpp)
target_link_libraries(linear_lookup_table_test
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  robot_controllers
)

add_executable(linear_lookup_table_ros_test linear_lookup_table_ros_test.cpp)
target_link_libraries(linear_lookup_table_ros_test
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  ${GTEST_LIBRARIES}
  robot_controllers
)

add_dependencies(tests linear_lookup_table_ros_test)

add_rostest(linear_lookup_table_ros_test.test)
