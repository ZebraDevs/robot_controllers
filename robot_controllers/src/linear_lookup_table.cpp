// Copyright (c) 2015, Fetch Robotics Inc.
// Author: Derek King

#include <robot_controllers/linear_lookup_table.h>
#include <algorithm>
#include <cmath>

namespace robot_controllers
{

LinearLookupTable::LinearLookupTable()
{
  off_table_ = ReturnNaN;
}

bool LinearLookupTable::init(ros::NodeHandle& nh)
{
  std::string off_table_str;
  if (!nh.getParam("off_table", off_table_str))
  {
    ROS_ERROR_NAMED("LinearLookupTable", "off_table not defined in namespace %s", nh.getNamespace().c_str());
    return false;
  }
  else if (off_table_str == "return_nan")
  {
    off_table_ = ReturnNaN;
  }
  else if (off_table_str == "return_closest")
  {
    off_table_ = ReturnClosest;
  }
  else if (off_table_str == "extrapolate")
  {
    off_table_ = Extrapolate;
  }
  else
  {
    ROS_ERROR_NAMED("LinearLookupTable", "off_table value (%s) is not recognised.  Use 'return_nan', 'return_closest', or 'extrapolate'", off_table_str.c_str());
    return false;
  }

  XmlRpc::XmlRpcValue table;
  if (!nh.getParam("table", table))
  {
    ROS_ERROR_NAMED("LinearLookupTable", "table not defined in namespace %s", nh.getNamespace().c_str());
    return false;
  }
  else if (table.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    ROS_ERROR_NAMED("LinearLookupTable", "table type must be array");
    return false;
  }
  else
  {
    table_.clear();
    for (int ii=0; ii<table.size(); ++ii)
    {
      XmlRpc::XmlRpcValue const &element = table[ii];
      if ((element.getType() != XmlRpc::XmlRpcValue::TypeArray) || (element.size() != 2))
      {
        ROS_ERROR_NAMED("LinearLookupTable", "table element %d must be array with two elements [in,out]", ii);
        return false;
      }
      if ((element[0].getType() != XmlRpc::XmlRpcValue::TypeDouble) ||
          (element[1].getType() != XmlRpc::XmlRpcValue::TypeDouble) )
      {
        ROS_ERROR_NAMED("LinearLookupTable", "Lookup table elements must be doubles for table index %d", ii);
        return false;
      }
      XmlRpc::XmlRpcValue in = element[0];
      XmlRpc::XmlRpcValue out = element[1];
      table_.push_back(std::pair<double,double>(double(in),double(out)));
    }
  }

  return orderTable();
}

bool LinearLookupTable::init(const std::vector<std::pair<double,double> > &table, OffTableEnum off_table)
{
  table_ = table;
  off_table_ = off_table;
  return orderTable();
}

double LinearLookupTable::lookup(double value) const
{
  // Maybe init was not called
  if (table_.size() == 0)
  {
    ROS_ERROR_NAMED("LinearLookupTable", "Table needs at least 1 entry");
    return NAN;
  }

  // Find two nearest table entries
  unsigned low;
  unsigned high;

  // Check if value is off
  if (value < table_.front().first)
  {
    if (off_table_ == ReturnNaN)
    {
      return NAN;
    }
    else if (off_table_ == ReturnClosest)
    {
      return table_.front().second;
    }
    else
    {
      // Two closest values are larger than input
      low = 0;
      high = 1;
    }
  }
  else if (value > table_.back().first)
  {
    if (off_table_ == ReturnNaN)
    {
      return NAN;
    }
    else if (off_table_ == ReturnClosest)
    {
      return table_.back().second;
    }
    else
    {
      // Two closest values are smaller than input
      high = table_.size()-1;
      low = high-1;
    }
  }
  else
  {
    // Do binary search to find 2 closest table values
    low = 0;
    high = table_.size()-1;
    do
    {
      unsigned index = (low+high)>>1;
      if (value >= table_[index].first)
      {
        low = index;
      }
      else
      {
        high = index;
      }
    } while ((low+1) < high);
  }

  // Now low+1 == high, use linear interpolation to determine output value
  double in1 = table_[low].first;
  double out1 = table_[low].second;
  double in2 = table_[high].first;
  double out2 = table_[high].second;
  return (value-in1)*(out2-out1)/(in2-in1) + out1;
}

bool LinearLookupTable::orderTable(void)
{
  // Sort table
  std::sort(table_.begin(), table_.end());

  // Table needs at least one entry
  if (table_.size() == 0)
  {
    ROS_ERROR_NAMED("LinearLookupTable", "Table needs at least 1 entry");
    return false;
  }

  // Need at least 2 table entries to use extrapolate mode
  if ((off_table_ == Extrapolate ) && (table_.size() < 2))
  {
    ROS_ERROR_NAMED("LinearLookupTable", "Table needs at least 2 entries to use extrapolate mode");
    return false;
  }

  // Check for invalid values
  for (unsigned ii=0; ii<table_.size(); ++ii)
  {
    if (!std::isfinite(table_[ii].first))
    {
      ROS_ERROR_NAMED("LinearLookupTable", "Found invalid input with value %f",
                      table_[ii].first);
      return false;
    }
    if (!std::isfinite(table_[ii].second))
    {
      ROS_ERROR_NAMED("LinearLookupTable", "Found invalid output with value %f",
                      table_[ii].second);
      return false;
    }
  }

  // Check for duplicated or very close entries
  for (unsigned ii=1; ii<table_.size(); ++ii)
  {
    double v1 = table_[ii].first;
    double v2 = table_[ii-1].first;
    if (v1 == v2)
    {
      ROS_ERROR_NAMED("LinearLookupTable", "Found duplicate lookup table inputs with input value %f",
                      table_[ii].first);
      return false;
    }
    double absdiff = fabs(v1-v2);
    double abssum = fabs(v1)+fabs(v2);
    // Numerical instability can occur with both very small differences
    // or very large numbers with relatively small differences  (compared to their absolute values)
    if ((absdiff < 1e-6) || ((abssum>1e-6) && ((absdiff/abssum) < 1e-6)))
    {
      ROS_ERROR_NAMED("LinearLookupTable",
                      "Table has two values that are too close to each other and may cause numerical instability : %f and %f. absdiff=%f abssum=%f", v1, v2, absdiff, abssum);
      return false;
    }
  }

  return true;
}


std::ostream &operator<<(std::ostream &out, const LinearLookupTable &lkup)
{
  out << "off_table: " <<
    ((lkup.off_table_ == LinearLookupTable::ReturnNaN) ? "return NaN" :
     (lkup.off_table_ == LinearLookupTable::ReturnClosest) ? "return closest" :
     (lkup.off_table_ == LinearLookupTable::Extrapolate) ? "extrapolate" : "??") << std::endl;

  out << "table:" << std::endl;
  for (unsigned ii=0; ii<lkup.table_.size(); ++ii)
  {
    out << lkup.table_[ii].first << '\t' << lkup.table_[ii].second << std::endl;
  }
  return out;
}


}  // namespace robot_controllers
