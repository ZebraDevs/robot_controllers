catkin_add_gtest(trajectory_tests trajectory_tests.cpp)
target_link_libraries(trajectory_tests
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
)

catkin_add_gtest(diff_drive_limiter_test diff_drive_limiter_test.cpp)
target_link_libraries(diff_drive_limiter_test robot_controllers)

