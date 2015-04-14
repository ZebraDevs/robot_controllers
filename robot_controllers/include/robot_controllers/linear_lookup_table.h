// Copyright (c) 2015, Fetch Robotics Inc.
// Author: Derek King

#ifndef ROBOT_CONTROLLERS_LINEAR_LOOKUP_TABLE_H
#define ROBOT_CONTROLLERS_LINEAR_LOOKUP_TABLE_H

#include <vector>
#include <utility>
#include <ros/ros.h>

namespace robot_controllers
{

/**
 * @brief Provides function that map input values to output values using lookup table.
 * When input value does not match entery in table exactly, this finds two nearest
 * values and does linear interpolation between them to determine output value.
 * When input is beyond end or table, there is a option to do either three things:
 *  1. Return NaN
 *  2. Return value of closest neighbor.
 *  3. Use two nearest neighbors and extrapolate out
 */
class LinearLookupTable
{
 public:
  LinearLookupTable();

  /**
   * @brief Enum used to specify what to do for input values that are off one of the
   * end of the lookup table
   */
  enum OffTableEnum {ReturnNaN=0, ReturnClosest=1, Extrapolate=2};


  /**
   * @brief Initialize the lookup table values used interpolation using rosparams
   * @param nh Node handle for getting parameters
   * @returns true if succesfully configured, false for errors
   *
   * The node handle must have the following parameters defined in its namespace:
   *   off_table : determines what to do with inputs that are beyond end of table
   *      options : "return_nan", "return_closest", "extrapolate"
   *        return_nan : returns NaN for input value outside range of table
   *        return_closest : return value of closest end of table
   *        extrapolate : use two points at closest end of table to extrapolate outwards
   *   table : should be array of 2-element arrays with input - output table mapping
   *        for input output array, the input is the first element, and the output is the
   *        second element.  Both the input and output should be double values.
   */
  bool init(ros::NodeHandle& nh);

  /**
   * @brief Initialize the lookup table values used for interpolation using a vector of pairs
   * @param table vector of input,output pairs used for loouptable
   * @param what to do when given values that are off end of table
   * @returns true if succesfully configured, false for errors
   */
  bool init(const std::vector<std::pair<double,double> > &table, OffTableEnum off_table);

  /**
   * @brief Finds two closest neighbors in lookup table and return linear interpolation
   * @param nh Node handle for getting parameters
   * @returns true if succesfully configured, false for errors
   */
  double lookup(double value) const;

  /**
   * @brief Returns current off table setting
   */
  inline OffTableEnum getOffTable() const {return off_table_;}

  friend std::ostream &operator<<(std::ostream &out, const LinearLookupTable &lkup);

 protected:
  /**
   * @brief Puts lookup table in order of lowest to highest.  Also checks for duplicate input values
   * @returns True if table is ok, false if table is invalid (duplicated input values, NaN, infinity)
   */
  bool orderTable(void);

  std::vector< std::pair<double, double> > table_;
  OffTableEnum off_table_;
};

/**
 * @brief Prints table data to stream
 */
std::ostream &operator<<(std::ostream &out, const LinearLookupTable &lkup);

} // namespace robot_controllers

#endif // ROBOT_CONTROLLERS_LINEAR_LOOKUP_TABLE_H
