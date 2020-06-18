/*********************************************************************
 *  Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, Michael Ferguson
 *  Copyright (c) 2014, Fetch Robotics Inc.
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

#ifndef ROBOT_CONTROLLERS__POINT_HEAD_H_
#define ROBOT_CONTROLLERS__POINT_HEAD_H_

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robot_controllers_interface/controller.h"
#include "robot_controllers_interface/joint_handle.h"
#include "robot_controllers_interface/controller_manager.h"
#include "tf2_ros/transform_listener.h"
#include "control_msgs/action/point_head.hpp"

#include "robot_controllers/trajectory.h"
#include "robot_controllers/trajectory_spline_sampler.h"

#include "kdl/tree.hpp"

namespace robot_controllers
{

class PointHeadController : public robot_controllers_interface::Controller
{
  using PointHeadAction = control_msgs::action::PointHead;
  using PointHeadGoal = rclcpp_action::ServerGoalHandle<PointHeadAction>;

public:
  PointHeadController() {}
  virtual ~PointHeadController() {}

  /**
   * @brief Initialize the controller and any required data structures.
   * @param nh Node handle for this controller.
   * @param manager The controller manager instance, this is needed for the
   *        controller to get information about joints, etc.
   * @returns 0 if succesfully configured, negative values are error codes.
   */
  virtual int init(const std::string& name,
                   rclcpp::Node::SharedPtr node,
                   robot_controllers_interface::ControllerManagerPtr manager);

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
   * @brief Cleanly reset the controller to it's initial state. Some controllers
   *        may choose to stop themselves. This is mainly used in the case of the
   *        the robot exiting some fault condition.
   * @returns True if successfully reset, false otherwise.
   */
  virtual bool reset();

  /**
   * @brief This is the update loop for the controller.
   * @param time The system time.
   * @param dt The timestep since last call to update.
   */
  virtual void update(const rclcpp::Time& now, const rclcpp::Duration& dt);

  /** @brief Get the type of this controller. */
  virtual std::string getType()
  {
    return "robot_controllers/PointHeadController";
  }

  /** @brief Get the names of joints/controllers which this controller commands. */
  virtual std::vector<std::string> getCommandedNames();

  /** @brief Get the names of joints/controllers which this controller exclusively claims. */
  virtual std::vector<std::string> getClaimedNames();

private:
  // rclcpp callbacks
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const PointHeadAction::Goal> goal_handle);
  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<PointHeadGoal> goal_handle);
  void handle_accepted(const std::shared_ptr<PointHeadGoal> goal_handle);

  // Handles to node, manager
  rclcpp::Node::SharedPtr node_;
  robot_controllers_interface::ControllerManagerPtr manager_;

  // Parameters
  std::string pan_parent_link_;
  std::string tilt_parent_link_;
  bool stop_with_action_;  /// should we stop this controller when the
                           /// action has terminated (or hold position)?

  // Joint handles
  robot_controllers_interface::JointHandlePtr head_pan_;
  robot_controllers_interface::JointHandlePtr head_tilt_;

  // Trajectory generation
  std::shared_ptr<TrajectorySampler> sampler_;
  std::mutex sampler_mutex_;

  /*
   * In certain cases, we want to start a trajectory at our last sample,
   * for instance if we were pre-empted (as is often the case with teleop)
   * we need to use the velocity and position of the last sample as a
   * starting point.
   */
  TrajectoryPoint last_sample_;

  // RCLCPP action server
  rclcpp_action::Server<PointHeadAction>::SharedPtr server_;
  std::shared_ptr<PointHeadGoal> active_goal_;

  KDL::Tree kdl_tree_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace robot_controllers

#endif  // ROBOT_CONTROLLERS__POINT_HEAD_H_
