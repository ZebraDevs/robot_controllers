#include <gtest/gtest.h>
#include <robot_controllers/linear_lookup_table.h>
#include <ros/ros.h>

using robot_controllers::LinearLookupTable;

TEST(test_linear_lookup_table, ros_init_tests)
{
  robot_controllers::LinearLookupTable lkup;

  {
    ros::NodeHandle nh("test_init_off_table_extrapolate");
    ASSERT_TRUE(lkup.init(nh));
    EXPECT_EQ(lkup.getOffTable(), LinearLookupTable::Extrapolate);
  }

  {
    ros::NodeHandle nh("test_init_off_table_closest");
    ASSERT_TRUE(lkup.init(nh));
    EXPECT_EQ(lkup.getOffTable(), LinearLookupTable::ReturnClosest);
  }

  {
    ros::NodeHandle nh("test_init_off_table_nan");
    ASSERT_TRUE(lkup.init(nh));
    EXPECT_EQ(lkup.getOffTable(), LinearLookupTable::ReturnNaN);
  }

  {
    ros::NodeHandle nh("no_parameters");
    ASSERT_FALSE(lkup.init(nh));
  }

  {
    ros::NodeHandle nh("no_off_table");
    ASSERT_FALSE(lkup.init(nh));
  }

  {
    ros::NodeHandle nh("invalid_off_table");
    ASSERT_FALSE(lkup.init(nh));
  }

  {
    ros::NodeHandle nh("no_table");
    ASSERT_FALSE(lkup.init(nh));
  }

  {
    ros::NodeHandle nh("table_is_not_an_array");
    ASSERT_FALSE(lkup.init(nh));
  }

  {
    ros::NodeHandle nh("table_element_is_not_an_array");
    ASSERT_FALSE(lkup.init(nh));
  }

  {
    ros::NodeHandle nh("table_element_does_not_have_2_elements");
    ASSERT_FALSE(lkup.init(nh));
  }

  {
    ros::NodeHandle nh("table_element_is_not_array_of_doubles");
    ASSERT_FALSE(lkup.init(nh));
  }
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  ros::init(argc, argv, "linear_lookup_table_ros_test");
  testing::InitGoogleTest(&argc, argv);
  ros::NodeHandle nh("linear_lookup_table_ros_test");
  return RUN_ALL_TESTS();
}
