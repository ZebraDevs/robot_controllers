#include <gtest/gtest.h>
#include <robot_controllers/linear_lookup_table.h>
#include <limits>

using robot_controllers::LinearLookupTable;

void simple_tests_helper(const robot_controllers::LinearLookupTable &lkup)
{
  // Make sure values between -1 and 1 return proper values
  double max_error = 1e-6;
  EXPECT_NEAR(lkup.lookup(0), 0, max_error);
  EXPECT_NEAR(lkup.lookup(1), -2, max_error);
  EXPECT_NEAR(lkup.lookup(-1), 2, max_error);
  EXPECT_NEAR(lkup.lookup(0.5), -1, max_error);
  EXPECT_NEAR(lkup.lookup(-0.5), 1, max_error);
  EXPECT_NEAR(lkup.lookup(0.005), -0.01, max_error);
  EXPECT_NEAR(lkup.lookup(-0.005), 0.01, max_error);
}

TEST(test_linear_lookup_table, simple_tests)
{
  robot_controllers::LinearLookupTable lkup;

  std::vector<std::pair<double,double> > table;
  // Empty table should not init
  ASSERT_FALSE(lkup.init(table, LinearLookupTable::RETURN_NAN));

  // Empty table should return NaN
  ASSERT_TRUE(isnan(lkup.lookup(0.0)));

  // Table with 1 entry should not allow EXTRAPOLATE
  table.push_back(std::pair<double,double>(-1,2));
  ASSERT_FALSE(lkup.init(table, LinearLookupTable::EXTRAPOLATE));

  // Table with 1 entry should be ok for other things
  ASSERT_TRUE(lkup.init(table, LinearLookupTable::RETURN_NAN));

  table.push_back(std::pair<double,double>(1,-2));

  // Table with 2 entries should be fine
  ASSERT_TRUE(lkup.init(table, LinearLookupTable::RETURN_NAN));

  // Make sure values inside normal range return correct values
  simple_tests_helper(lkup);

  // Make sure value outside -1 and 1 return NaN
  ASSERT_TRUE(std::isnan(lkup.lookup(-1.0001)));
  ASSERT_TRUE(std::isnan(lkup.lookup(1.0001)));

  // Change type of table to closest and check values outside normal range,
  ASSERT_TRUE(lkup.init(table, LinearLookupTable::RETURN_CLOSEST));
  simple_tests_helper(lkup); // Make sure stuff inside range still returns correct values
  double max_error = 1e-6;
  EXPECT_NEAR(lkup.lookup(-1.0001), 2.0, max_error);
  EXPECT_NEAR(lkup.lookup(1.0001), -2.0, max_error);
  EXPECT_NEAR(lkup.lookup(-1000), 2.0, max_error);
  EXPECT_NEAR(lkup.lookup(1000), -2.0, max_error);

  // Change type of table to linear extrapolate and make
  // sure stuff outside ranges return correct values
  ASSERT_TRUE(lkup.init(table, LinearLookupTable::EXTRAPOLATE));
  simple_tests_helper(lkup); // Make sure stuff inside range still returns correct values
  EXPECT_NEAR(lkup.lookup(-1.0001), 2.0002, max_error);
  EXPECT_NEAR(lkup.lookup(1.0001), -2.0002, max_error);
  EXPECT_NEAR(lkup.lookup(-1000), 2000, max_error);
  EXPECT_NEAR(lkup.lookup(1000), -2000, max_error);

  // Make sure init with table that has multiple matching entries returns false
  table.push_back(std::pair<double,double>(1,-3));
  ASSERT_FALSE(lkup.init(table, LinearLookupTable::RETURN_NAN));

  // Make sure init with table that has really close entries returns false
  table.back().first = 1 + 1e-9;
  ASSERT_FALSE(lkup.init(table, LinearLookupTable::RETURN_NAN));

  // Make sure init with table that has nonfinite inputs returns false
  table.back().first = NAN;
  ASSERT_FALSE(lkup.init(table, LinearLookupTable::RETURN_NAN));
  table.back().first = std::numeric_limits<double>::infinity();
  ASSERT_FALSE(lkup.init(table, LinearLookupTable::RETURN_NAN));

  // Make sure init with table that nonfinite outputs returns false
  table.back().first = 2;
  table.back().second = NAN;
  ASSERT_FALSE(lkup.init(table, LinearLookupTable::RETURN_NAN));
  table.back().second = std::numeric_limits<double>::infinity();
  ASSERT_FALSE(lkup.init(table, LinearLookupTable::RETURN_NAN));

  // Make sure init with table that relatively close entries also returns false
  table.back().first = 1e9;
  table.back().second = 3;
  table.push_back(std::pair<double,double>(1e9+1,4));
  ASSERT_FALSE(lkup.init(table, LinearLookupTable::RETURN_NAN));
}


void interpolate_tests_helper(const std::vector<std::pair<double,double> > &ordered_table,
                              const std::vector<std::pair<double,double> > &unordered_table)
{
  robot_controllers::LinearLookupTable lkup;
  lkup.init(unordered_table, LinearLookupTable::EXTRAPOLATE);
  //std::cerr << lkup << std::endl;

  double max_error = 1e-6;

  double substeps = 10;
  for (unsigned ii=1; ii<ordered_table.size(); ++ii)
  {
    double in1 = ordered_table[ii-1].first;
    double out1 = ordered_table[ii-1].second;
    double in2 = ordered_table[ii].first;
    double out2 = ordered_table[ii].second;
    EXPECT_NEAR(lkup.lookup(in1), out1, max_error);
    EXPECT_NEAR(lkup.lookup(in2), out2, max_error);
    double scale = 0.0;
    double stop = 1.0;
    if (ii==1)
    {
      // test extrapolate at begining by starting scale at value < 0.0
      scale = -1.0;
    }
    else if (ii==(ordered_table.size()-1))
    {
      // test extrapolate at end by stopping scale at value > 1.0
      stop = 2.0;
    }
    while (scale <= stop)
    {
      double in = scale*(in2-in1) + in1;
      double out = scale*(out2-out1) + out1;
      EXPECT_NEAR(lkup.lookup(in), out, max_error);
      //std::cerr << in << '\t' << lkup.lookup(in) << std::endl;
      scale += 1.0/substeps;
    }
  }
}

TEST(test_linear_lookup_table, interpolate_tests)
{
  // Test ordered table
  std::vector<std::pair<double,double> > table;
  table.push_back(std::pair<double,double>(-10,0));
  table.push_back(std::pair<double,double>(0,-20));
  table.push_back(std::pair<double,double>(1,-2));
  table.push_back(std::pair<double,double>(10,3));
  table.push_back(std::pair<double,double>(12,3));
  table.push_back(std::pair<double,double>(13,-3));
  interpolate_tests_helper(table, table);

  // Create unordered table and test that
  std::vector<std::pair<double,double> > ordered_table(table);
  // Unordered table has same elements as ordered table, but in different order
  std::swap(table[1],table[3]);
  std::swap(table[0],table[5]);

  interpolate_tests_helper(ordered_table, table);
}


// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
