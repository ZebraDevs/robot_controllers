/*********************************************************************
 *  Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, Fetch Robotics Inc.
 *  Copyright (c) 2013, Unbounded Robotics Inc.
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Unbounded Robotics nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/*
 * Derived a bit from control_toolbox/pid.cpp
 */

#include <cmath>
#include <limits>
#include <algorithm>
#include <string>

#include "robot_controllers/pid.h"

namespace robot_controllers
{

PID::PID(double p, double i, double d, double i_max, double i_min) :
  p_gain_(p),
  i_gain_(i),
  d_gain_(d),
  i_max_(i_max),
  i_min_(i_min)
{
  reset();
  checkGains();
}

PID::PID() :
  p_gain_(0.0),
  i_gain_(0.0),
  d_gain_(0.0),
  i_max_(0.0),
  i_min_(0.0)
{
  reset();
}

bool PID::init(const std::string& name, rclcpp::Node::SharedPtr node)
{
  node_ = node;

  p_gain_ = node_->declare_parameter<double>(name + ".p", std::numeric_limits<double>::quiet_NaN());
  i_gain_ = node_->declare_parameter<double>(name + ".i", std::numeric_limits<double>::quiet_NaN());
  d_gain_ = node_->declare_parameter<double>(name + ".d", std::numeric_limits<double>::quiet_NaN());

  double i_clamp = node_->declare_parameter<double>(name + ".i_clamp", 0.0);
  i_max_ = std::abs(i_clamp);
  i_min_ = -std::abs(i_clamp);

  if (std::isnan(p_gain_))
  {
    // If P-gain is not specified, this often indicates wrong namespace was used
    RCLCPP_ERROR(node_->get_logger(), "No P gain sepcified. Parameter namespace %s", name.c_str());
    return false;
  }

  return checkGains();
}

bool PID::checkGains()
{
  bool pass = true;
  if (!std::isfinite(p_gain_))
  {
    RCLCPP_WARN(node_->get_logger(), "Proportional gain is not finite");
    p_gain_ = 0.0;
    pass = false;
  }
  if (!std::isfinite(i_gain_))
  {
    RCLCPP_WARN(node_->get_logger(), "Integral gain is not finite");
    i_gain_ = 0.0;
    pass = false;
  }
  if (!std::isfinite(d_gain_))
  {
    RCLCPP_WARN(node_->get_logger(), "Derivative gain is not finite");
    d_gain_ = 0.0;
    pass = false;
  }
  if (!std::isfinite(i_max_) || !std::isfinite(i_min_))
  {
    RCLCPP_WARN(node_->get_logger(), "Integral wind-up limit is not finite");
    i_max_ = 0.0;
    i_min_ = 0.0;
    pass = false;
  }
  if (i_max_ < i_min_)
  {
    RCLCPP_WARN(node_->get_logger(), "Integral max windup value is smaller than minimum value");
    double tmp = i_max_;
    i_max_ = i_min_;
    i_min_ = tmp;
    pass = false;
  }
  if ((i_min_ == 0) && (i_max_ == 0) && (i_gain_ != 0))
  {
    // It is easy to forgot to set a wind-up limit
    RCLCPP_WARN(node_->get_logger(),
                "Integral gain is non-zero, but integral wind-up limit is zero");
  }
  if ( ((i_min_ != 0) || (i_max_ != 0)) && (i_gain_ == 0) )
  {
    RCLCPP_WARN(node_->get_logger(), "Integral gain is zero, but wind-yup limit is zero");
  }
  return pass;
}

void PID::reset()
{
  i_term_ = 0.0;
  error_last_ = 0.0;
}

double PID::update(double error, double dt)
{
  double error_dot;
  if (dt <= 0.0)
  {
    RCLCPP_ERROR(node_->get_logger(), "PID::update : dt value is less than or equal to zero");
    // if dt is zero is not possible to perform division
    // in this case assume error_dot is zero and perform reset of calculation
    error_dot = 0.0;
  }
  else
  {
    error_dot = (error-error_last_)/dt;
  }
  error_last_ = error;
  return update(error, error_dot, dt);
}

double PID::update(double error, double error_dot, double dt)
{
  if (!std::isfinite(error) || !std::isfinite(error_dot) || !std::isfinite(dt))
  {
    RCLCPP_ERROR(node_->get_logger(), "PID::update : input value is NaN or infinity");
    return 0.0;
  }

  if (dt <= 0.0)
  {
    RCLCPP_ERROR(node_->get_logger(), "PID::update : dt value is less than or equal to zero");
    dt = 0.0;
  }

  double p_term = p_gain_*error;

  i_term_ += i_gain_ * error * dt;

  // apply wind-up limits to i_term
  i_term_ = std::max(i_min_, std::min(i_term_, i_max_));

  double d_term = d_gain_ * error_dot;

  return p_term + i_term_ + d_term;
}

}  // namespace robot_controllers
