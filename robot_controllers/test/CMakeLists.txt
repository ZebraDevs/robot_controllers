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