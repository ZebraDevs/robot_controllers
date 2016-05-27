/*
 * Copyright (c) 2016, Fetch Robotics Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Fetch Robotics Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL FETCH ROBOTICS INC. BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

// Author: Derek King

#include <gtest/gtest.h>
#include <robot_controllers/diff_drive_limiter.h>
#include <limits>

using robot_controllers::DiffDriveLimiter;
using robot_controllers_msgs::DiffDriveLimiterParams;

TEST(DiffDriveLimiterTests, 1_test_velocity_limits)
{
  DiffDriveLimiterParams params;
  params.max_linear_velocity  = 2.0;
  params.max_linear_acceleration = std::numeric_limits<double>::max();
  params.max_angular_velocity = 4.0;
  params.max_angular_acceleration = std::numeric_limits<double>::max();
  params.max_wheel_velocity = std::numeric_limits<double>::max();
  params.track_width = 0.5;
  params.angular_velocity_limits_linear_velocity = false;
  params.scale_to_wheel_velocity_limits = false;
  DiffDriveLimiter limiter(params);

  double limited_linear_velocity, limited_angular_velocity;
  double desired_linear_velocity, desired_angular_velocity;
  double safety_scaling = 1.0;
  double dt = 1.0;


  // Make sure linear or angular velocity is not limited when under limits
  desired_linear_velocity = 1.0;
  desired_angular_velocity = 1.0;
  limiter.limit(&limited_linear_velocity, &limited_angular_velocity,
                desired_linear_velocity, desired_angular_velocity,
                desired_linear_velocity, desired_angular_velocity,
                safety_scaling, dt);

  EXPECT_EQ(limited_linear_velocity, desired_linear_velocity);
  EXPECT_EQ(limited_angular_velocity, desired_angular_velocity);

  // Make sure linear velocity hits positive limit
  desired_linear_velocity = 10.0;
  desired_angular_velocity = 0.0;
  limiter.limit(&limited_linear_velocity, &limited_angular_velocity,
                desired_linear_velocity, desired_angular_velocity,
                desired_linear_velocity, desired_angular_velocity,
                safety_scaling, dt);
  EXPECT_NEAR(limited_linear_velocity, params.max_linear_velocity, 1e-6);

  // Make sure linear velocity hits negative limit
  desired_linear_velocity = -10.0;
  desired_angular_velocity = 0.0;
  limiter.limit(&limited_linear_velocity, &limited_angular_velocity,
                desired_linear_velocity, desired_angular_velocity,
                desired_linear_velocity, desired_angular_velocity,
                safety_scaling, dt);
  EXPECT_NEAR(limited_linear_velocity, -params.max_linear_velocity, 1e-6);


  // Make sure angular velocity hits positive limit
  desired_linear_velocity = 0.0;
  desired_angular_velocity = 10.0;
  limiter.limit(&limited_linear_velocity, &limited_angular_velocity,
                desired_linear_velocity, desired_angular_velocity,
                desired_linear_velocity, desired_angular_velocity,
                safety_scaling, dt);
  EXPECT_NEAR(limited_angular_velocity, params.max_angular_velocity, 1e-6);

  // Make sure angular velocity hits negative limit
  desired_linear_velocity = 0.0;
  desired_angular_velocity = -10.0;
  limiter.limit(&limited_linear_velocity, &limited_angular_velocity,
                desired_linear_velocity, desired_angular_velocity,
                desired_linear_velocity, desired_angular_velocity,
                safety_scaling, dt);
  EXPECT_NEAR(limited_angular_velocity, -params.max_angular_velocity, 1e-6);


  // Make sure linear velocity positive limit is reduced by safety scaling
  desired_linear_velocity = 10.0;
  desired_angular_velocity = 0.0;
  safety_scaling = 0.5;
  limiter.limit(&limited_linear_velocity, &limited_angular_velocity,
                desired_linear_velocity, desired_angular_velocity,
                desired_linear_velocity, desired_angular_velocity,
                safety_scaling, dt);
  EXPECT_NEAR(limited_linear_velocity, params.max_linear_velocity*safety_scaling, 1e-6);

  // Make sure angular velocity hits negative limit
  desired_linear_velocity = -10.0;
  desired_angular_velocity = 0.0;
  safety_scaling = 0.25;
  limiter.limit(&limited_linear_velocity, &limited_angular_velocity,
                desired_linear_velocity, desired_angular_velocity,
                desired_linear_velocity, desired_angular_velocity,
                safety_scaling, dt);
  EXPECT_NEAR(limited_linear_velocity, -params.max_linear_velocity*safety_scaling, 1e-6);


  // Make sure angular velocity positive is not limited by safety scaling when linear velocity is 0
  desired_linear_velocity = 0.0;
  desired_angular_velocity = 1.0;
  safety_scaling = 0.0; // This would force angular velocity to be zero
  limiter.limit(&limited_linear_velocity, &limited_angular_velocity,
                desired_linear_velocity, desired_angular_velocity,
                desired_linear_velocity, desired_angular_velocity,
                safety_scaling, dt);
  EXPECT_NEAR(limited_angular_velocity, desired_angular_velocity, 1e-6);
}


TEST(DiffDriveLimiterTests, 2_test_wheel_velocity_limits)
{
  DiffDriveLimiterParams params;
  params.max_linear_velocity = std::numeric_limits<double>::max();;
  params.max_linear_acceleration = std::numeric_limits<double>::max();
  params.max_angular_velocity = std::numeric_limits<double>::max();;
  params.max_angular_acceleration = std::numeric_limits<double>::max();
  params.max_wheel_velocity = 2.0;
  params.track_width = 1.0;
  params.angular_velocity_limits_linear_velocity = false;
  params.scale_to_wheel_velocity_limits = false;
  DiffDriveLimiter limiter(params);

  double limited_linear_velocity, limited_angular_velocity;
  double desired_linear_velocity, desired_angular_velocity;
  double safety_scaling = 1.0;
  double dt = 1.0;

  // Make sure linear or angular velocity is not limited when wheel velocity is less than limits
  desired_linear_velocity = 1.0;
  desired_angular_velocity = 0.0;
  limiter.limit(&limited_linear_velocity, &limited_angular_velocity,
                desired_linear_velocity, desired_angular_velocity,
                desired_linear_velocity, desired_angular_velocity,
                safety_scaling, dt);
  EXPECT_NEAR(limited_linear_velocity, desired_linear_velocity, 1e-6);
  EXPECT_NEAR(limited_angular_velocity, desired_angular_velocity, 1e-6);

  // Make sure linear or angular velocity is not limited when wheel velocity is less than limits
  desired_linear_velocity = 0.0;
  desired_angular_velocity = 1.0;
  limiter.limit(&limited_linear_velocity, &limited_angular_velocity,
                desired_linear_velocity, desired_angular_velocity,
                desired_linear_velocity, desired_angular_velocity,
                safety_scaling, dt);
  EXPECT_NEAR(limited_linear_velocity, desired_linear_velocity, 1e-6);
  EXPECT_NEAR(limited_angular_velocity, desired_angular_velocity, 1e-6);

  // When linear velocity is too large,
  // Make sure angular velocity stays zero,
  // and wheel velocites are equal to limits
  desired_linear_velocity = 10.0;
  desired_angular_velocity = 0.0;
  limiter.limit(&limited_linear_velocity, &limited_angular_velocity,
                desired_linear_velocity, desired_angular_velocity,
                desired_linear_velocity, desired_angular_velocity,
                safety_scaling, dt);
  // Angular velocity should stay near zero
  EXPECT_NEAR(limited_angular_velocity, desired_angular_velocity, 1e-6);
  // Angular velocity limit is same as wheel velocity limits
  EXPECT_NEAR(limited_linear_velocity, params.max_wheel_velocity, 1e-6);

  // When linear velocity is too large,
  // Make sure linear velocity stays zero,
  // and wheel velocites are equal to limits
  desired_linear_velocity = 0.0;
  desired_angular_velocity = 10.0;
  limiter.limit(&limited_linear_velocity, &limited_angular_velocity,
                desired_linear_velocity, desired_angular_velocity,
                desired_linear_velocity, desired_angular_velocity,
                safety_scaling, dt);
  // linear velocity should stay near zero
  EXPECT_NEAR(limited_linear_velocity, desired_linear_velocity, 1e-6);

  double left_velocity, right_velocity;
  limiter.calcWheelVelocities(&left_velocity, &right_velocity,
                              limited_linear_velocity, limited_angular_velocity);
  EXPECT_NEAR(left_velocity, -params.max_wheel_velocity, 1e-6);
  EXPECT_NEAR(right_velocity, params.max_wheel_velocity, 1e-6);

  // When linear velocity is too large,
  // Make sure linear velocity stays zero,
  // and wheel velocites are equal to limits
  desired_linear_velocity = 0.0;
  desired_angular_velocity = -10.0;
  limiter.limit(&limited_linear_velocity, &limited_angular_velocity,
                desired_linear_velocity, desired_angular_velocity,
                desired_linear_velocity, desired_angular_velocity,
                safety_scaling, dt);
  // linear velocity should stay near zero
  EXPECT_NEAR(limited_linear_velocity, desired_linear_velocity, 1e-6);

  limiter.calcWheelVelocities(&left_velocity, &right_velocity,
                              limited_linear_velocity, limited_angular_velocity);
  EXPECT_NEAR(left_velocity, params.max_wheel_velocity, 1e-6);
  EXPECT_NEAR(right_velocity, -params.max_wheel_velocity, 1e-6);
}



TEST(DiffDriveLimiterTests, 3_test_curvature_matching)
{
  DiffDriveLimiterParams params;
  params.max_linear_velocity = 2.0;
  params.max_linear_acceleration = std::numeric_limits<double>::max();
  params.max_angular_velocity = 4.0;
  params.max_angular_acceleration = std::numeric_limits<double>::max();;
  params.max_wheel_velocity = std::numeric_limits<double>::max();
  params.track_width = 1.0;
  params.angular_velocity_limits_linear_velocity = false;
  params.scale_to_wheel_velocity_limits = false;
  DiffDriveLimiter limiter(params);

  double limited_linear_velocity, limited_angular_velocity;
  double desired_linear_velocity, desired_angular_velocity;
  double safety_scaling = 1.0;
  double dt = 1.0;

  // Make sure curvature matches when linear velocity goes over limit
  desired_linear_velocity = 3.0;
  desired_angular_velocity = 3.0;
  limiter.limit(&limited_linear_velocity, &limited_angular_velocity,
                desired_linear_velocity, desired_angular_velocity,
                desired_linear_velocity, desired_angular_velocity,
                safety_scaling, dt);
  EXPECT_NEAR(limited_linear_velocity/limited_angular_velocity,
              desired_linear_velocity/desired_angular_velocity, 1e-6);
  EXPECT_NEAR(limited_linear_velocity, params.max_linear_velocity, 1e-6);


  // Make sure curvature matches when linear velocity goes over limit
  desired_linear_velocity = -3.0;
  desired_angular_velocity = 3.0;
  limiter.limit(&limited_linear_velocity, &limited_angular_velocity,
                desired_linear_velocity, desired_angular_velocity,
                desired_linear_velocity, desired_angular_velocity,
                safety_scaling, dt);
  EXPECT_NEAR(limited_linear_velocity/limited_angular_velocity,
              desired_linear_velocity/desired_angular_velocity, 1e-6);
  EXPECT_NEAR(limited_linear_velocity, -params.max_linear_velocity, 1e-6);

  // when angular_velocity_limits_linear_velocity flag is not set,
  // make sure angular limits do not effect linear velocity
  desired_linear_velocity = 1.0;
  desired_angular_velocity = 9.0;
  limiter.limit(&limited_linear_velocity, &limited_angular_velocity,
                desired_linear_velocity, desired_angular_velocity,
                desired_linear_velocity, desired_angular_velocity,
                safety_scaling, dt);
  EXPECT_NEAR(limited_linear_velocity, desired_linear_velocity, 1e-6);


  // when angular_velocity_limits_linear_velocity flag is set,
  // Make sure curvature matches when angular velocity hits limit
  params.angular_velocity_limits_linear_velocity = true;
  limiter.setParams(params);
  desired_linear_velocity = 1.0;
  desired_angular_velocity = 9.0;
  limiter.limit(&limited_linear_velocity, &limited_angular_velocity,
                desired_linear_velocity, desired_angular_velocity,
                desired_linear_velocity, desired_angular_velocity,
                safety_scaling, dt);
  EXPECT_NEAR(limited_linear_velocity/limited_angular_velocity,
              desired_linear_velocity/desired_angular_velocity, 1e-6);
  EXPECT_NEAR(limited_angular_velocity, params.max_angular_velocity, 1e-6);

  // Make sure curvature matches when safety scaling is non-zero
  params.angular_velocity_limits_linear_velocity = true;
  limiter.setParams(params);
  desired_linear_velocity = 2.0;
  desired_angular_velocity = 3.0;
  safety_scaling = 0.25;
  limiter.limit(&limited_linear_velocity, &limited_angular_velocity,
                desired_linear_velocity, desired_angular_velocity,
                desired_linear_velocity, desired_angular_velocity,
                safety_scaling, dt);
  EXPECT_NEAR(limited_linear_velocity/limited_angular_velocity,
              desired_linear_velocity/desired_angular_velocity, 1e-6);
  EXPECT_NEAR(limited_linear_velocity, desired_linear_velocity*safety_scaling, 1e-6);
  safety_scaling = 1.0;


  // when angular_velocity_limits_linear_velocity flag is set,
  // Make sure curvature matches when angular velocity hits limit
  params.angular_velocity_limits_linear_velocity = true;
  limiter.setParams(params);
  desired_linear_velocity = 1.0;
  desired_angular_velocity = -9.0;
  limiter.limit(&limited_linear_velocity, &limited_angular_velocity,
                desired_linear_velocity, desired_angular_velocity,
                desired_linear_velocity, desired_angular_velocity,
                safety_scaling, dt);
  EXPECT_NEAR(limited_linear_velocity/limited_angular_velocity,
              desired_linear_velocity/desired_angular_velocity, 1e-6);
  EXPECT_NEAR(limited_angular_velocity, -params.max_angular_velocity, 1e-6);
}


TEST(DiffDriveLimiterTests, 4_test_wheel_curvature_matching)
{
  DiffDriveLimiterParams params;
  params.max_linear_velocity = std::numeric_limits<double>::max();
  params.max_linear_acceleration = std::numeric_limits<double>::max();
  params.max_angular_velocity = std::numeric_limits<double>::max();
  params.max_angular_acceleration = std::numeric_limits<double>::max();;
  params.max_wheel_velocity = 2.0;
  params.track_width = 1.0;
  params.angular_velocity_limits_linear_velocity = false;
  params.scale_to_wheel_velocity_limits = true;
  DiffDriveLimiter limiter(params);

  double limited_linear_velocity, limited_angular_velocity;
  double desired_linear_velocity, desired_angular_velocity;
  double left_velocity, right_velocity;
  double safety_scaling = 1.0;
  double dt = 1.0;

  // Make sure curvature matches when linear velocity goes over limit
  desired_linear_velocity = 10.0;
  desired_angular_velocity = 1.0;
  limiter.limit(&limited_linear_velocity, &limited_angular_velocity,
                desired_linear_velocity, desired_angular_velocity,
                desired_linear_velocity, desired_angular_velocity,
                safety_scaling, dt);
  EXPECT_NEAR(limited_linear_velocity/limited_angular_velocity,
              desired_linear_velocity/desired_angular_velocity, 1e-6);
  limiter.calcWheelVelocities(&left_velocity, &right_velocity,
                              limited_linear_velocity, limited_angular_velocity);
  EXPECT_LE(left_velocity,  params.max_wheel_velocity+1e-6);
  EXPECT_LE(right_velocity, params.max_wheel_velocity+1e-6);
  EXPECT_GE(left_velocity,  -params.max_wheel_velocity-1e-6);
  EXPECT_GE(right_velocity, -params.max_wheel_velocity-1e-6);

  // Make sure curvature matches when linear velocity goes over negative limit
  desired_linear_velocity = -10.0;
  desired_angular_velocity = 1.0;
  limiter.limit(&limited_linear_velocity, &limited_angular_velocity,
                desired_linear_velocity, desired_angular_velocity,
                desired_linear_velocity, desired_angular_velocity,
                safety_scaling, dt);
  EXPECT_NEAR(limited_linear_velocity/limited_angular_velocity,
              desired_linear_velocity/desired_angular_velocity, 1e-6);
  limiter.calcWheelVelocities(&left_velocity, &right_velocity,
                              limited_linear_velocity, limited_angular_velocity);
  EXPECT_LE(left_velocity,  params.max_wheel_velocity+1e-6);
  EXPECT_LE(right_velocity, params.max_wheel_velocity+1e-6);
  EXPECT_GE(left_velocity,  -params.max_wheel_velocity-1e-6);
  EXPECT_GE(right_velocity, -params.max_wheel_velocity-1e-6);


  // Make sure curvature matches when angular velocity goes over limit
  desired_linear_velocity = 1.0;
  desired_angular_velocity = 10.0;
  limiter.limit(&limited_linear_velocity, &limited_angular_velocity,
                desired_linear_velocity, desired_angular_velocity,
                desired_linear_velocity, desired_angular_velocity,
                safety_scaling, dt);
  EXPECT_NEAR(limited_linear_velocity/limited_angular_velocity,
              desired_linear_velocity/desired_angular_velocity, 1e-6);
  limiter.calcWheelVelocities(&left_velocity, &right_velocity,
                              limited_linear_velocity, limited_angular_velocity);
  EXPECT_LE(left_velocity,  params.max_wheel_velocity+1e-6);
  EXPECT_LE(right_velocity, params.max_wheel_velocity+1e-6);
  EXPECT_GE(left_velocity,  -params.max_wheel_velocity-1e-6);
  EXPECT_GE(right_velocity, -params.max_wheel_velocity-1e-6);

  // Make sure curvature matches when linear velocity goes over negative limit
  desired_linear_velocity = 1.0;
  desired_angular_velocity = -10.0;
  limiter.limit(&limited_linear_velocity, &limited_angular_velocity,
                desired_linear_velocity, desired_angular_velocity,
                desired_linear_velocity, desired_angular_velocity,
                safety_scaling, dt);
  EXPECT_NEAR(limited_linear_velocity/limited_angular_velocity,
              desired_linear_velocity/desired_angular_velocity, 1e-6);
  limiter.calcWheelVelocities(&left_velocity, &right_velocity,
                              limited_linear_velocity, limited_angular_velocity);
  EXPECT_LE(left_velocity,  params.max_wheel_velocity+1e-6);
  EXPECT_LE(right_velocity, params.max_wheel_velocity+1e-6);
  EXPECT_GE(left_velocity,  -params.max_wheel_velocity-1e-6);
  EXPECT_GE(right_velocity, -params.max_wheel_velocity-1e-6);
}



TEST(DiffDriveLimiterTests, 5_test_acceleration_limits)
{
  DiffDriveLimiterParams params;
  params.max_linear_velocity = std::numeric_limits<double>::max();
  params.max_linear_acceleration = 2.0;
  params.max_angular_velocity = std::numeric_limits<double>::max();
  params.max_angular_acceleration = 4.0;
  params.max_wheel_velocity = std::numeric_limits<double>::max();
  params.track_width = 1.0;
  params.angular_velocity_limits_linear_velocity = false;
  params.scale_to_wheel_velocity_limits = false;
  DiffDriveLimiter limiter(params);

  double limited_linear_velocity, limited_angular_velocity;
  double desired_linear_velocity, desired_angular_velocity;
  double last_linear_velocity, last_angular_velocity;
  double safety_scaling = 1.0;
  double dt = 0.5;
  double desired_linear_accel, desired_angular_accel;
  double actual_linear_accel, actual_angular_accel;

  // Make sure accelerations are not limited when they are below limites
  desired_linear_accel = 1.0;
  desired_angular_accel = -1.0;
  last_linear_velocity = 9.0;
  last_angular_velocity = -4.0;
  desired_linear_velocity = desired_linear_accel*dt + last_linear_velocity;
  desired_angular_velocity = desired_angular_accel*dt + last_angular_velocity;
  limiter.limit(&limited_linear_velocity, &limited_angular_velocity,
                desired_linear_velocity, desired_angular_velocity,
                last_linear_velocity, last_angular_velocity,
                safety_scaling, dt);
  // If acceleration is below limits, then limited veloicty with match desired velocity
  EXPECT_NEAR(desired_linear_velocity, limited_linear_velocity, 1e-6);
  EXPECT_NEAR(desired_angular_velocity, limited_angular_velocity, 1e-6);


  // Make sure linear acceleration is limited to max
  desired_linear_accel = 10.0;
  desired_angular_accel = 1.0;
  last_linear_velocity = 3.0;
  last_angular_velocity = -8.0;
  desired_linear_velocity = desired_linear_accel*dt + last_linear_velocity;
  desired_angular_velocity = desired_angular_accel*dt + last_angular_velocity;
  limiter.limit(&limited_linear_velocity, &limited_angular_velocity,
                desired_linear_velocity, desired_angular_velocity,
                last_linear_velocity, last_angular_velocity,
                safety_scaling, dt);
  actual_linear_accel = (limited_linear_velocity - last_linear_velocity)/dt;
  actual_angular_accel = (limited_angular_velocity - last_angular_velocity)/dt;
  EXPECT_NEAR(actual_linear_accel, params.max_linear_acceleration, 1e-6);
  EXPECT_NEAR(actual_angular_accel, desired_angular_accel, 1e-6);

  // Make sure linear acceleration is limited to negative max
  desired_linear_accel = -10.0;
  desired_angular_accel = 1.0;
  last_linear_velocity = 0.0;
  last_angular_velocity = 4.0;
  desired_linear_velocity = desired_linear_accel*dt + last_linear_velocity;
  desired_angular_velocity = desired_angular_accel*dt + last_angular_velocity;
  limiter.limit(&limited_linear_velocity, &limited_angular_velocity,
                desired_linear_velocity, desired_angular_velocity,
                last_linear_velocity, last_angular_velocity,
                safety_scaling, dt);
  actual_linear_accel = (limited_linear_velocity - last_linear_velocity)/dt;
  actual_angular_accel = (limited_angular_velocity - last_angular_velocity)/dt;
  EXPECT_NEAR(actual_linear_accel, -params.max_linear_acceleration, 1e-6);
  EXPECT_NEAR(actual_angular_accel, desired_angular_accel, 1e-6);



  // Make sure angular acceleration is limited to max
  desired_linear_accel = 1.0;
  desired_angular_accel = 13.5;
  last_linear_velocity = 6.0;
  last_angular_velocity = -51.0;
  desired_linear_velocity = desired_linear_accel*dt + last_linear_velocity;
  desired_angular_velocity = desired_angular_accel*dt + last_angular_velocity;
  limiter.limit(&limited_linear_velocity, &limited_angular_velocity,
                desired_linear_velocity, desired_angular_velocity,
                last_linear_velocity, last_angular_velocity,
                safety_scaling, dt);
  actual_linear_accel = (limited_linear_velocity - last_linear_velocity)/dt;
  actual_angular_accel = (limited_angular_velocity - last_angular_velocity)/dt;
  EXPECT_NEAR(actual_linear_accel, desired_linear_accel, 1e-6);
  EXPECT_NEAR(actual_angular_accel, params.max_angular_acceleration, 1e-6);

  // Make sure angular acceleration is limited to negative max
  desired_linear_accel = -0.5;
  desired_angular_accel = -15.0;
  last_linear_velocity = 3.0;
  last_angular_velocity = -2.0;
  desired_linear_velocity = desired_linear_accel*dt + last_linear_velocity;
  desired_angular_velocity = desired_angular_accel*dt + last_angular_velocity;
  limiter.limit(&limited_linear_velocity, &limited_angular_velocity,
                desired_linear_velocity, desired_angular_velocity,
                last_linear_velocity, last_angular_velocity,
                safety_scaling, dt);
  actual_linear_accel = (limited_linear_velocity - last_linear_velocity)/dt;
  actual_angular_accel = (limited_angular_velocity - last_angular_velocity)/dt;
  EXPECT_NEAR(actual_linear_accel, desired_linear_accel, 1e-6);
  EXPECT_NEAR(actual_angular_accel, -params.max_angular_acceleration, 1e-6);

  // Make sure both accelerations are limited between -max and max
  for (desired_linear_accel = -6.0; desired_linear_accel<6.0; desired_linear_accel+=0.5)
  {
    for (desired_angular_accel = -12.0; desired_linear_accel<12.0; desired_linear_accel+=0.5)
    {
      last_linear_velocity = 3.0;
      last_angular_velocity = -2.0;
      desired_linear_velocity = desired_linear_accel*dt + last_linear_velocity;
      desired_angular_velocity = desired_angular_accel*dt + last_angular_velocity;
      limiter.limit(&limited_linear_velocity, &limited_angular_velocity,
                    desired_linear_velocity, desired_angular_velocity,
                    last_linear_velocity, last_angular_velocity,
                    safety_scaling, dt);
      actual_linear_accel = (limited_linear_velocity - last_linear_velocity)/dt;
      actual_angular_accel = (limited_angular_velocity - last_angular_velocity)/dt;
      // Assert instead of Expect, because loop could create a lot of failing test cases
      ASSERT_LE(actual_linear_accel,  params.max_linear_acceleration+1e-6);
      ASSERT_GE(actual_linear_accel, -params.max_linear_acceleration-1e-6);
      ASSERT_LE(actual_angular_accel,  params.max_angular_acceleration+1e-6);
      ASSERT_GE(actual_angular_accel, -params.max_angular_acceleration-1e-6);
    }
  }


  // Make sure acceleration limits still work when dt is zero
  dt = 0.0;
  desired_linear_accel = 1.0;
  desired_angular_accel = -1.0;
  last_linear_velocity = 9.0;
  last_angular_velocity = -4.0;
  desired_linear_velocity = 12.0;
  desired_angular_velocity = 3.0;
  limiter.limit(&limited_linear_velocity, &limited_angular_velocity,
                desired_linear_velocity, desired_angular_velocity,
                last_linear_velocity, last_angular_velocity,
                safety_scaling, dt);
  // If dt is zero, then veloicties can't change
  EXPECT_NEAR(limited_linear_velocity, last_linear_velocity, 1e-6);
  EXPECT_NEAR(limited_angular_velocity, last_angular_velocity, 1e-6);
}


TEST(DiffDriveLimiterTests, 6_validate_params)
{
  // Make sure default params are ok
  DiffDriveLimiterParams params;

  params = DiffDriveLimiter::getDefaultParams();
  EXPECT_NO_THROW(DiffDriveLimiter::validateParams(params));


  params = DiffDriveLimiter::getDefaultParams();
  params.max_linear_velocity = 0.0;
  EXPECT_THROW(DiffDriveLimiter::validateParams(params), std::exception);
  params = DiffDriveLimiter::getDefaultParams();
  params.max_linear_velocity = NAN;
  EXPECT_THROW(DiffDriveLimiter::validateParams(params), std::exception);
  params = DiffDriveLimiter::getDefaultParams();
  params.max_linear_velocity = -1.0;
  EXPECT_THROW(DiffDriveLimiter::validateParams(params), std::exception);


  params = DiffDriveLimiter::getDefaultParams();
  params.max_angular_velocity = 0.0;
  EXPECT_THROW(DiffDriveLimiter::validateParams(params), std::exception);
  params = DiffDriveLimiter::getDefaultParams();
  params.max_angular_velocity = NAN;
  EXPECT_THROW(DiffDriveLimiter::validateParams(params), std::exception);
  params = DiffDriveLimiter::getDefaultParams();
  params.max_angular_velocity = -1.0;
  EXPECT_THROW(DiffDriveLimiter::validateParams(params), std::exception);


  params = DiffDriveLimiter::getDefaultParams();
  params.max_wheel_velocity = 0.0;
  EXPECT_THROW(DiffDriveLimiter::validateParams(params), std::exception);
  params = DiffDriveLimiter::getDefaultParams();
  params.max_wheel_velocity = NAN;
  EXPECT_THROW(DiffDriveLimiter::validateParams(params), std::exception);
  params = DiffDriveLimiter::getDefaultParams();
  params.max_wheel_velocity = -1.0;
  EXPECT_THROW(DiffDriveLimiter::validateParams(params), std::exception);


  params = DiffDriveLimiter::getDefaultParams();
  params.max_linear_acceleration = 0.0;
  EXPECT_THROW(DiffDriveLimiter::validateParams(params), std::exception);
  params = DiffDriveLimiter::getDefaultParams();
  params.max_linear_acceleration = NAN;
  EXPECT_THROW(DiffDriveLimiter::validateParams(params), std::exception);
  params = DiffDriveLimiter::getDefaultParams();
  params.max_linear_acceleration = -1.0;
  EXPECT_THROW(DiffDriveLimiter::validateParams(params), std::exception);


  params = DiffDriveLimiter::getDefaultParams();
  params.max_angular_acceleration = 0.0;
  EXPECT_THROW(DiffDriveLimiter::validateParams(params), std::exception);
  params = DiffDriveLimiter::getDefaultParams();
  params.max_angular_acceleration = NAN;
  EXPECT_THROW(DiffDriveLimiter::validateParams(params), std::exception);
  params = DiffDriveLimiter::getDefaultParams();
  params.max_angular_acceleration = -1.0;
  EXPECT_THROW(DiffDriveLimiter::validateParams(params), std::exception);


  params = DiffDriveLimiter::getDefaultParams();
  params.track_width = 0.0;
  EXPECT_THROW(DiffDriveLimiter::validateParams(params), std::exception);
  params = DiffDriveLimiter::getDefaultParams();
  params.track_width = NAN;
  EXPECT_THROW(DiffDriveLimiter::validateParams(params), std::exception);
  params = DiffDriveLimiter::getDefaultParams();
  params.track_width = -1.0;
  EXPECT_THROW(DiffDriveLimiter::validateParams(params), std::exception);
}


// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
