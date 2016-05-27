/*********************************************************************
 *  Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014-2016, Fetch Robotics Inc.
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

#ifndef ROBOT_CONTROLLERS_DIFF_DRIVE_LIMITER_H
#define ROBOT_CONTROLLERS_DIFF_DRIVE_LIMITER_H

#include <robot_controllers_msgs/DiffDriveLimiterParams.h>

namespace robot_controllers
{

class DiffDriveLimiter
{
 public:
  /**
   * @brief Default constructor variables will be set to class has not limits
   */
  DiffDriveLimiter();

  /**
   * @brief Constructor
   */
  DiffDriveLimiter(const robot_controllers_msgs::DiffDriveLimiterParams &params);

  /**
   * @brief Enforces acceleration and velocity limits on desired angular and
   *     linear velocities, in such a way the that the original desired trajectory
   *     is maintained.
   * @param limited_linear_velocity where limited angular velocity will be stored
   * @param limited_angular_velocity where limited angular velocity will be stored
   * @param desired_linear_velocity desired linear velocity
   * @param desired_angular_velocity desired angular velocity
   * @param last_linear_velocity last linear velocity, used to compute acceleration
   * @param last_angular_velocity last angular velocity,  used to compute acceleration
   * @param safety_scaling value between 0.0 and 1.0 used scale back max linear velocity
   *        for example if safety_scaling was 0.5, then effective max_linear_velocity
   *        would be half of what the parameter specified
   * @param dt time delta between now and previous time. used for computing acceleration
   *
   * Note: For this to work properly, none of the values should be NaN, and dt > 0.0
   */
  void limit(double *limited_linear_velocity,
             double *limited_angular_velocity,
             double desired_linear_velocity,
             double desired_angular_velocity,
             double last_linear_velocity,
             double last_angular_velocity,
             double safety_scaling,
             double dt);

  /**
   * @brief Calculates linear and angular velocites from wheel velocities
   * @param left_velocity where left wheel velocity will be stored
   * @param right_velocity where right wheel velocity will be stored
   * @param linear_velocity desired linear velocity
   * @param angular_velocity desired angular velocity
   */
  inline void calcWheelVelocities(double *left_velocity,
                                  double *right_velocity,
                                  double linear_velocity,
                                  double angular_velocity) const
  {
    *left_velocity  = linear_velocity - (0.5 * angular_velocity * params_.track_width);
    *right_velocity = linear_velocity + (0.5 * angular_velocity * params_.track_width);
  }

  /**
   * @brief Gets track_width (distance between wheels)
   */
  inline double getTrackWidth() const
  {
    return params_.track_width;
  }

  /**
   * @brief Sets new parameters for diff drive base
   */
  void setParams(const robot_controllers_msgs::DiffDriveLimiterParams &new_params);

  /**
   * @brief Get parameter from diff drive base
   */
  inline robot_controllers_msgs::DiffDriveLimiterParams getParams() const
  {
    return params_;
  }

  /**
   * @brief Get default parameters
   */
  static robot_controllers_msgs::DiffDriveLimiterParams getDefaultParams();

  /**
   * @brief Validate diff drive base parameters, throws exception if they don't make sense
   */
  static void validateParams(const robot_controllers_msgs::DiffDriveLimiterParams &params);

 protected:
  robot_controllers_msgs::DiffDriveLimiterParams params_;
};

} // namespace robot_controllers

#endif // ROBOT_CONTROLLERS_DIFF_DRIVE_LIMITER_H
