/*********************************************************************
 *  Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, FYS Systems Inc.
 *  Copyright (c) 2013, Unbounded Robotics Inc.
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

// Author: Michael Ferguson

#ifndef ROBOT_CONTROLLERS_DIFF_DRIVE_BASE_H
#define ROBOT_CONTROLLERS_DIFF_DRIVE_BASE_H

#include <string>
#include <boost/shared_ptr.hpp>

#include <ros/ros.h>
#include <robot_controllers_interface/controller.h>
#include <robot_controllers_interface/controller_manager.h>
#include <robot_controllers_interface/joint_handle.h>
#include <tf/transform_broadcaster.h>

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

namespace robot_controllers
{

/**
 *  \brief ROS-aware controller to manage a differential drive mobile base. This
 *         subcribes to cmd_vel topic, publishes odom and tf, and manages the two
 *         wheel joints.
 */
class DiffDriveBaseController : public Controller
{
public:
  DiffDriveBaseController();
  virtual ~DiffDriveBaseController() {}

  /**
   * @brief Initialize the controller and any required data structures.
   * @param nh Node handle for this controller.
   * @param manager The controller manager instance, this is needed for the
   *        controller to get information about joints, etc.
   * @returns 0 if succesfully configured, negative values are error codes.
   */
  virtual int init(ros::NodeHandle& nh, ControllerManager* manager);

  /**
   * @brief Attempt to start the controller. This should be called only by the
   *        ControllerManager instance.
   * @returns True if successfully started, false otherwise.
   */
  virtual bool start();

  /**
   * @brief Attempt to stop the controller. This should be called only by the
   *        ControllerManager instance.
   * @param force Should we force the controller to stop? Some controllers
   *        may wish to continue running until they absolutely have to stop.
   * @returns True if successfully stopped, false otherwise.
   */
  virtual bool stop(bool force);

  /**
   * @brief This is the update loop for the controller.
   * @param time The system time.
   * @param dt The timestep since last call to update.
   */
  virtual void update(const ros::Time& now, const ros::Duration& dt);

  /** @brief Get the names of joints/controllers which this controller commands. */
  virtual std::vector<std::string> getCommandedNames();

  /** @brief Get the names of joints/controllers which this controller exclusively claims. */
  virtual std::vector<std::string> getClaimedNames();

  /** \brief Command callback from either a ROS topic, or a higher controller. */
  void command(const geometry_msgs::TwistConstPtr& msg);

  /** \brief Publish odom, possibly tf */
  bool publish(ros::Time time);

private:
  bool initialized_;
  ControllerManager* manager_;

  void updateCallback(const ros::WallTimerEvent& event);

  // Set base wheel speeds in m/s
  void setCommand(float left, float right);

  JointHandlePtr left_;
  JointHandlePtr right_;

  double track_width_;
  double radians_per_meter_;
  double theta_;

  double moving_threshold_;

  double max_velocity_x_;
  double max_velocity_r_;
  double max_acceleration_x_;
  double max_acceleration_r_;

  // These are the inputs from the ROS topic
  float desired_x_;
  float desired_r_;

  // These are from controller update
  float last_sent_x_;
  float last_sent_r_;

  float left_last_position_;
  float right_last_position_;
  double left_last_timestamp_;
  double right_last_timestamp_;

  ros::Time last_command_;
  ros::Time last_update_;
  ros::Duration timeout_;

  nav_msgs::Odometry odom_;
  ros::Publisher odom_pub_;
  ros::Subscriber cmd_sub_;

  boost::shared_ptr<tf::TransformBroadcaster> broadcaster_;
  bool publish_tf_;

  std::string odometry_frame_;
  std::string base_frame_;

  bool enabled_;
  bool ready_;
};

}  // namespace robot_controllers

#endif  // ROBOT_CONTROLLERS_DIFF_DRIVE_BASE_H
