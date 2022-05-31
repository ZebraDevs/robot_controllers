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
 * Derived a bit from pr2_controllers/cartesian_pose_controller.cpp
 * Author: Michael Ferguson, Wim Meeussen
 */

#ifndef ROBOT_CONTROLLERS__CARTESIAN_POSE_H_
#define ROBOT_CONTROLLERS__CARTESIAN_POSE_H_

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "robot_controllers/pid.h"
#include "robot_controllers_interface/controller.h"
#include "robot_controllers_interface/joint_handle.h"
#include "robot_controllers_interface/controller_manager.h"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

#include "kdl/chain.hpp"
#include "kdl/chainjnttojacsolver.hpp"
#include "kdl/chainfksolverpos_recursive.hpp"
#include "kdl/frames.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace robot_controllers
{

class CartesianPoseController : public robot_controllers_interface::Controller
{
public:
  CartesianPoseController();
  virtual ~CartesianPoseController() {}

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
    return "robot_controllers/CartesianPoseController";
  }

  /** @brief Get the names of joints/controllers which this controller commands. */
  virtual std::vector<std::string> getCommandedNames();

  /** @brief Get the names of joints/controllers which this controller exclusively claims. */
  virtual std::vector<std::string> getClaimedNames();

  /** @brief Controller command. */
  void command(const geometry_msgs::msg::PoseStamped::SharedPtr goal);

private:
  KDL::Frame getPose();

  bool initialized_;
  rclcpp::Node::SharedPtr node_;
  robot_controllers_interface::ControllerManagerPtr manager_;

  bool enabled_;
  std::string root_link_;
  rclcpp::Time last_command_;

  KDL::Frame desired_pose_;
  KDL::Frame actual_pose_;

  KDL::Twist twist_error_;

  KDL::Chain kdl_chain_;
  std::shared_ptr<KDL::ChainFkSolverPos> jnt_to_pose_solver_;
  std::shared_ptr<KDL::ChainJntToJacSolver> jac_solver_;
  KDL::JntArray jnt_pos_;
  KDL::JntArray jnt_delta_;
  KDL::Jacobian jacobian_;

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr feedback_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr command_sub_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::vector<robot_controllers_interface::JointHandlePtr> joints_;
  std::vector<robot_controllers::PID> pid_;
};

}  // namespace robot_controllers

#endif  // ROBOT_CONTROLLERS__CARTESIAN_POSE_H_
