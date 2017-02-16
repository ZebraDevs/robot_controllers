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
   * @brief Object used for passing feedback info to the limiter
   */
  struct Feedback
  {
    double left_wheel_velocity;   ///< Rolling velocity of left wheel (m/s)
    double right_wheel_velocity;  ///< Rolling velocity of right wheel (m/s)
  };

  /**
   * @brief Contains parameters for velocity backoff behavior

   *        Instantaneous backoff is computed by the following equation (EQ 1.):
   * \n
   *        \f[\beta = e^(-|\omega_{d} - \omega_|^{2} / \sigma_{\omega}^{2})
   *                   e^(-|\v_{d} - \v_|^{2} / \sigma_{\v}^{2})
   *                   e^(-|\omega_{d} - \omega_| |\v_{d} - \v_| / \sigma_{v, \omega}^{2})
               \in [0, 1]
   *        \f]
   * \n
   *        The value <code>beta</code> is filtered using nonlinear first-order lowpass filter
   *        with a piecewise gain. The gain takes on a values as follows:
   *        - <code>BackoffConfig::ramp_up_gain</code> when <code>beta >= 0.0</code>
   *        - <code>BackoffConfig::ramp_down_gain</code> when <code>beta < 0.0</code>
   *
   * @todo  Add constructor which loads parameters from param server?
   */
  struct BackoffConfig
  {
    double min_value;  ///< Minimum value backoff factor can take on
    double ramp_down_gain;  ///< First-order update gain for decreasing backoff
    double ramp_up_gain;  ///< First-order update gain for increasing backoff
    double linear_velocity_error_deadband;  ///< Minimum difference between desired and actual linear velocity
    double angular_velocity_error_deadband;  ///< Minimum difference between desired and actual angular velocity
    double linear_velocity_variance;  ///< Holds value of \f[\sigma_{\v}^{2}\f] in EQ 1.
    double angular_velocity_variance;  ///< Holds value of \f[\sigma_{\omega}^{2}\f] in EQ 1.
    double cross_variance;  ///< Holds value of \f[\sigma_{v, \omega}^{2}\f] in EQ 1.

    /**
     * @brief Default constructor
     */
    BackoffConfig() { setDefaults();}

    /**
     * @brief Sets default values for BackoffConfig
     */
    inline void setDefaults()
    {
      min_value = 0.1;
      ramp_down_gain = 5e-2;
      ramp_up_gain = 1e-0;
      linear_velocity_error_deadband = 0.01;  // m/s
      angular_velocity_error_deadband = 0.01;  // rad/s
      linear_velocity_variance = 2.5e-3;  // 0.015 m/s variation
      angular_velocity_variance = 7.5e-5;  // 0.5 degree variation
      angular_velocity_variance = 1.0;
    }
  };

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
   * @param feeback optional param for suppling feedback to the limiting operation
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
             double dt,
             Feedback *feeback = NULL);

  /**
   * @brief Calculates wheel velocities from linear and angular velocities
   * @param left_velocity[out] where left wheel velocity will be stored
   * @param right_velocity[out] where right wheel velocity will be stored
   * @param linear_velocity desired linear velocity
   * @param angular_velocity desired angular velocity
   */
  inline void calcWheelVelocities(double& left_velocity,
                                  double& right_velocity,
                                  double linear_velocity,
                                  double angular_velocity) const
  {
    left_velocity  = linear_velocity - (0.5 * angular_velocity * params_.track_width);
    right_velocity = linear_velocity + (0.5 * angular_velocity * params_.track_width);
  }

  /**
   * @brief Calculates linear and angular velocities from wheel velocities
   * @param left_velocity desired left wheel velocity
   * @param right_velocity desired right wheel velocity
   * @param linear_velocity[out] where desired linear velocity will be stored
   * @param angular_velocity[out] where desired angular velocity will be stored
   */
  inline void calcDriveVelocities(double left_velocity,
                                  double right_velocity,
                                  double& linear_velocity,
                                  double& angular_velocity) const
  {
    linear_velocity  = 0.5 * (right_velocity + left_velocity);
    angular_velocity = (right_velocity - left_velocity) / params_.track_width;
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

  /**
   * @brief Exposes velocity scaling ("backoff") factor
   */
  inline double getVelocityBackoffFactor() const
  {
    return backoff_factor_;
  }

  /**
   * @brief Sets velocity backoff behavior configuration
   */
  inline void setBackoffConfig(BackoffConfig config)
  {
    backoff_config_ = config;
  }

 protected:
  robot_controllers_msgs::DiffDriveLimiterParams params_;

 private:
  /**
   * @brief Holds configuration for velocity-backoff behavior
   */
  BackoffConfig backoff_config_;

  /**
   * @brief Holds the backoff state
   *
   *        This values is always between [backoff_config_.min_value, 1] and is used to scale
   *        both desired linear and angular velocities (to maintain curvature) when current
   *        desired velocities exceed actual robot velocities (from feedback)
   * \n
   *        Backoff behavior can be enabled/disabled using
   *        <code>robot_controllers_msgs::DiffDriveLimiterParams::scale_to_wheel_velocity_limits</code>
   *
   * @see Feedback
   * @see BackoffConfig
   */
  double backoff_factor_;
};

} // namespace robot_controllers

#endif // ROBOT_CONTROLLERS_DIFF_DRIVE_LIMITER_H
