/*********************************************************************
 *  Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, Fetch Robotics Inc.
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

// Author: Derek King

#include <ros/console.h>

#include <algorithm>
#include <limits>

#include <robot_controllers/diff_drive_limiter.h>

namespace robot_controllers
{


DiffDriveLimiter::DiffDriveLimiter()
{
  setParams(getDefaultParams());
}


DiffDriveLimiter::DiffDriveLimiter(const robot_controllers_msgs::DiffDriveLimiterParams &params)
{
  setParams(params);
}


/**
 * @brief saturates value between +/- limit
 * @param value value to be limited
 * @param limit limit to enforce to value
 * @returns value that can be limited between +/- limit
 */
inline double saturate(double value, double limit)
{
  return std::max(-limit, std::min(limit, value));
}


/**
 * @brief determines value needed to scale value between +/- limit
 * @param value value to be limited
 * @param limit limit to enforce to value (must be >= 0)
 * @returns scale that can be multipled by +/- limit to keep it between +/- limit
 *          abs(value * scale) <= limit
 */
inline double scaleToLimit(double value, double limit)
{
  if ((value > limit) || (value < -limit))
  {
    return std::abs(limit/value);
  }
  return 1.0;
}


/**
 * @brief limits acceleration without dividing by dt
 * @param desired_velocity desired velocity
 * @param last_velocity previous velocity value
 * @param dt time delta since previous cycle
 * @param max_acceleration acceleration limit
 * @returns velocity that has been limited so it does not violate
 *          acceleration limit
 */
inline double limitAccel(double desired_velocity, double last_velocity,
                         double dt, double max_acceleration)
{
  // re-arrange equation to avoid dividing by dt
  // because it could be zero
  double velocity_delta = desired_velocity - last_velocity;
  double max_velocity_delta = max_acceleration * dt;
  return last_velocity + saturate(velocity_delta, max_velocity_delta);
}


void DiffDriveLimiter::limit(double *limited_linear_velocity,
                                 double *limited_angular_velocity,
                                 double desired_linear_velocity,
                                 double desired_angular_velocity,
                                 double last_linear_velocity,
                                 double last_angular_velocity,
                                 double safety_scaling,
                                 double dt)
{
  // Make sure dt is ever negative, and warn if it is 0.0 (since it really shouldn't be)
  if (dt <= 0.0)
  {
    ROS_WARN("DiffDriveLimiter : bad dt=%f", dt);
    // use dt = 0.0 as special value, with current code it won't cause
    // issues if it shows up once in a while.
    // however velocities can't change if dt is always zero
    dt = 0.0;
  }

  if (!std::isfinite(desired_linear_velocity) ||
      !std::isfinite(desired_angular_velocity) )
  {
    ROS_ERROR_THROTTLE(1.0, "DiffDriveLimiter : Invalid Range on inputs"
                       "linear=%f, angular=%f",
                       desired_linear_velocity, desired_angular_velocity);
    desired_linear_velocity = 0.0;
    desired_angular_velocity = 0.0;
    // Allow acceleration limits to bring robot to a stop
  }

  if (std::isnan(safety_scaling) ||
      (safety_scaling < 0.0) ||
      (safety_scaling > 1.0) )
  {
    ROS_ERROR_THROTTLE(1.0, "DiffDriveLimiter : Invalid Range on safety scaling %f",
                       safety_scaling);
    if (std::isnan(safety_scaling))
    {
      // min/max doesn't always work on NaN, assign to zero
      safety_scaling = 0.0;
    }
    safety_scaling = std::max(0.0, std::min(1.0, safety_scaling));
  }

  // Use safety scaling to modify max linear velocity
  double max_linear_velocity = params_.max_linear_velocity * safety_scaling;

  // First limit linear and angular velocities
  double linear_scale = scaleToLimit(desired_linear_velocity, max_linear_velocity);

  double angular_scale = scaleToLimit(desired_angular_velocity, params_.max_angular_velocity);
  angular_scale = std::min(angular_scale, linear_scale);

  desired_angular_velocity *= angular_scale;

  if (params_.angular_velocity_limits_linear_velocity)
  {
    // Curvature is maintained this by scaling linear and angular velocity by
    // the same factor
    desired_linear_velocity *= angular_scale;
  }
  else
  {
    // Do not scale back linear velocity when angular velocity reaches limit.
    // This is how the previous code worked, but it does not maintain curvature
    desired_linear_velocity *= linear_scale;
  }

  // Calculate right wheel velocity and left wheel velocity from linear velocities
  double left, right;
  calcWheelVelocities(&left, &right, desired_linear_velocity, desired_angular_velocity);

  // Limit right and left wheel velocites
  if (params_.scale_to_wheel_velocity_limits)
  {
    // Scale wheel velocities together, to maintain curvature
    double wheel_velocity = std::max(fabs(left), fabs(right));
    double wheel_scale = scaleToLimit(wheel_velocity, params_.max_wheel_velocity);
    left *= wheel_scale;
    right *= wheel_scale;
  }
  else
  {
    // Scale wheel velocities separately.
    // This is how the previous code worked, but it does not maintain curvative
    left = saturate(left, params_.max_wheel_velocity);
    right = saturate(right, params_.max_wheel_velocity);
  }

  // Convert left/right velocities back to angular and linear commands
  desired_linear_velocity = 0.5*(right+left);
  desired_angular_velocity = (right-left)/params_.track_width;

  // Limit accelerations, for now don't try to maintain curvature
  // because it can't always be done if robot is already moving
  // However, if robot starts from stop and doesn't change commands
  // having acceleration limit matching would maintain curvature of robot
  *limited_linear_velocity = limitAccel(desired_linear_velocity, last_linear_velocity,
                                        dt, params_.max_linear_acceleration);
  *limited_angular_velocity = limitAccel(desired_angular_velocity, last_angular_velocity,
                                         dt, params_.max_angular_acceleration);
}


/**
 * @brief Get default parameters
 */
robot_controllers_msgs::DiffDriveLimiterParams DiffDriveLimiter::getDefaultParams()
{
  robot_controllers_msgs::DiffDriveLimiterParams params;
  params.max_linear_velocity = 1.0;
  params.max_angular_velocity = 4.5;
  params.max_linear_acceleration = 0.75;
  params.max_angular_acceleration = 3.0;
  params.max_wheel_velocity = 1.1;
  params.track_width = 0.37476;
  params.angular_velocity_limits_linear_velocity = false;
  params.scale_to_wheel_velocity_limits = false;
  return params;
}


/**
 * @brief Verifies that value is greater than zero (+infinity is ok), NaN is not ok
 */
void validatePositive(double value, const char* name)
{
  if (std::isnan(value) || (value <= 0.0))
  {
    std::stringstream ss;
    ss << "Invalid value for " << name << " : " << value;
    throw std::out_of_range(ss.str());
  }
}


void DiffDriveLimiter::validateParams(const robot_controllers_msgs::DiffDriveLimiterParams &params)
{
  validatePositive(params.max_linear_velocity, "max_linear_velocity");
  validatePositive(params.max_angular_velocity, "max_angular_velocity");
  validatePositive(params.max_wheel_velocity, "max_wheel_velocity");
  validatePositive(params.max_linear_acceleration, "max_linear_acceleration");
  validatePositive(params.max_angular_acceleration, "max_angular_acceleration");
  validatePositive(params.track_width, "track_width");
}


void DiffDriveLimiter::setParams(const robot_controllers_msgs::DiffDriveLimiterParams &new_params)
{
  validateParams(new_params);
  params_ = new_params;
}

} // namespace robot_controllers
