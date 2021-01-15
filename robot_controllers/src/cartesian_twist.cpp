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
 * Author: Michael Ferguson, Wim Meeussen, Hanjun Song
 */

#include <algorithm>
#include <string>
#include <vector>

#include "pluginlib/class_list_macros.hpp"
#include "robot_controllers/cartesian_twist.h"
#include "robot_controllers_interface/utils.h"

#include "urdf/model.h"
#include "kdl_parser/kdl_parser.hpp"

PLUGINLIB_EXPORT_CLASS(robot_controllers::CartesianTwistController,
                       robot_controllers_interface::Controller)

namespace robot_controllers
{

using namespace std::placeholders;
using robot_controllers_interface::declare_parameter_once;
using robot_controllers_interface::get_safe_topic_name;
using robot_controllers_interface::to_sec;

CartesianTwistController::CartesianTwistController() :
  initialized_(false),
  is_active_(false)
{
}

int CartesianTwistController::init(const std::string& name,
                                   rclcpp::Node::SharedPtr node,
                                   robot_controllers_interface::ControllerManagerPtr manager)
{
  // We absolutely need access to the controller manager
  if (!manager)
  {
    initialized_ = false;
    return -1;
  }

  Controller::init(name, node, manager);
  node_ = node;
  manager_ = manager;

  // Initialize KDL structures
  std::string root = node_->declare_parameter<std::string>(getName() + ".root", "torso_lift_link");
  std::string tip = node_->declare_parameter<std::string>(getName() + ".tip", "wrist_roll_link");

  // Load URDF
  urdf::Model model;
  std::string robot_description = declare_parameter_once<std::string>("robot_description",
                                                                      "", node);
  if (!model.initString(robot_description))
  {
    RCLCPP_ERROR(rclcpp::get_logger(getName()),
                 "Failed to parse URDF");
    return -1;
  }

  // Load the tree
  KDL::Tree kdl_tree;
  if (!kdl_parser::treeFromUrdfModel(model, kdl_tree))
  {
    RCLCPP_ERROR(rclcpp::get_logger(getName()),
                 "Could not construct tree from URDF");
    return -1;
  }

  // Populate the Chain
  if (!kdl_tree.getChain(root, tip, kdl_chain_))
  {
    RCLCPP_ERROR(rclcpp::get_logger(getName()),
                 "Could not construct chain from URDF");
    return -1;
  }

  solver_.reset(new KDL::ChainIkSolverVel_wdls(kdl_chain_));
  fksolver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));
  unsigned num_joints = kdl_chain_.getNrOfJoints();
  tgt_jnt_pos_.resize(num_joints);
  tgt_jnt_vel_.resize(num_joints);
  last_tgt_jnt_vel_.resize(num_joints);

  // Init Joint Handles
  joints_.clear();
  for (size_t i = 0; i < kdl_chain_.getNrOfSegments(); ++i)
    if (kdl_chain_.getSegment(i).getJoint().getType() != KDL::Joint::None)
      joints_.push_back(manager_->getJointHandle(kdl_chain_.getSegment(i).getJoint().getName()));

  if (joints_.size() != num_joints)
  {
    RCLCPP_ERROR(rclcpp::get_logger(getName()),
                 "Inconsistant joint count %d, %d", num_joints, static_cast<int>(joints_.size()));
    return -1;
  }

  for (unsigned ii = 0; ii < num_joints; ++ii)
  {
    last_tgt_jnt_vel_(ii) = 0.0;
  }

  // Subscribe to command
  std::string topic_name = get_safe_topic_name(name) + "/command";
  command_sub_ = node_->create_subscription<geometry_msgs::msg::TwistStamped>(topic_name, 1,
                    std::bind(&CartesianTwistController::command, this, _1));
  last_command_time_ = node_->now();

  initialized_ = true;
  return 0;
}

bool CartesianTwistController::start()
{
  if (!initialized_)
  {
    RCLCPP_ERROR(rclcpp::get_logger(getName()),
                 "Unable to start, not initialized.");
    is_active_ = false;
    return false;
  }

  for (unsigned ii = 0; ii < joints_.size(); ++ii)
  {
    last_tgt_jnt_vel_(ii) = joints_[ii]->getVelocity();
    tgt_jnt_pos_(ii) = joints_[ii]->getPosition();
  }
  is_active_ = true;
  return true;
}

bool CartesianTwistController::stop(bool force)
{
  (void) force;
  is_active_ = false;
  return true;
}

bool CartesianTwistController::reset()
{
  // Simply stop
  return (manager_->requestStop(getName()) == 0);
}

void CartesianTwistController::update(const rclcpp::Time& now, const rclcpp::Duration& dt)
{
  // Need to initialize KDL structs
  if (!initialized_)
    return;  // Should never really hit this

  KDL::Frame cart_pose;
  // Copy desired twist and update time to local var to reduce lock contention
  KDL::Twist twist;
  rclcpp::Time last_command_time;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    // FK is used to transform the twist command seen from end-effector
    // frame to the one seen from body frame.
    if (fksolver_->JntToCart(tgt_jnt_pos_, cart_pose) < 0)
    {
      twist.Zero();
      RCLCPP_ERROR(rclcpp::get_logger(getName()),
                   "CartesianTwistController: FKsolver solver failed");
    }
    else
    {
      if (twist_command_frame_ == "end_effector_frame")
      {
        twist = cart_pose.M * twist_command_;
      }
      else
      {
        twist = twist_command_;
      }
    }
    last_command_time = last_command_time_;
  }

  unsigned num_joints = joints_.size();

  if ((now - last_command_time) > rclcpp::Duration::from_seconds(0.5))
  {
    manager_->requestStop(getName());
  }

  // change the twist here
  if (solver_->CartToJnt(tgt_jnt_pos_, twist, tgt_jnt_vel_) < 0)
  {
    for (unsigned ii = 0; ii < num_joints; ++ii)
    {
      tgt_jnt_vel_(ii) = 0.0;
    }
  }

  // Limit joint velocities by scaling all target velocities equally
  // so resulting movement is in same direction
  double max_vel = 0.0;
  for (unsigned ii = 0; ii < num_joints; ++ii)
  {
    max_vel = std::max(std::abs(tgt_jnt_vel_(ii)), max_vel);
  }

  double joint_velocity_limit = 0.5;
  double scale = 1.0;
  if (max_vel > joint_velocity_limit)
  {
    double scale = joint_velocity_limit / max_vel;
    for (unsigned ii = 0; ii < num_joints; ++ii)
    {
      tgt_jnt_vel_(ii) *= scale;
    }
    RCLCPP_DEBUG(rclcpp::get_logger(getName()),
                 "Joint velocity limit reached.");
  }

  // Make sure solver didn't generate any NaNs.
  for (unsigned ii = 0; ii < num_joints; ++ii)
  {
    if (!std::isfinite(tgt_jnt_vel_(ii)))
    {
      RCLCPP_ERROR(rclcpp::get_logger(getName()),
                   "Target joint velocity (%d) is not finite : %f", ii, tgt_jnt_vel_(ii));
      tgt_jnt_vel_(ii) = 1.0;
    }
  }

  // Limit accelerations while trying to keep same resulting direction
  // somewhere between previous and current value
  scale = 1.0;
  double accel_limit = 1.0;
  double vel_delta_limit = accel_limit * to_sec(dt);
  for (unsigned ii = 0; ii < num_joints; ++ii)
  {
    double vel_delta = std::abs(tgt_jnt_vel_(ii) - last_tgt_jnt_vel_(ii));
    if (vel_delta > vel_delta_limit)
    {
      scale = std::min(scale, vel_delta_limit/vel_delta);
    }
  }

  if (scale <= 0.0)
  {
    RCLCPP_ERROR(rclcpp::get_logger(getName()),
                 "Acceleration limit produces non-positive scale %f", scale);
    scale = 0.0;
  }

  // Linear interpolate betwen previous velocity and new target velocity using scale.
  // scale = 1.0  final velocity = new target velocity
  // scale = 0.0  final velocity = previous velocity
  for (unsigned ii = 0; ii < num_joints; ++ii)
  {
    tgt_jnt_vel_(ii) = (tgt_jnt_vel_(ii) - last_tgt_jnt_vel_(ii))*scale + last_tgt_jnt_vel_(ii);
  }

  // Calculate new target position of joint.  Put target position a few timesteps into the future
  double dt_sec = to_sec(dt);
  for (unsigned ii = 0; ii < num_joints; ++ii)
  {
    tgt_jnt_pos_(ii) += tgt_jnt_vel_(ii) * dt_sec;
  }

  // Limit target position of joint
  for (unsigned ii = 0; ii < num_joints; ++ii)
  {
    if (!joints_[ii]->isContinuous())
    {
      if (tgt_jnt_pos_(ii) > joints_[ii]->getPositionMax())
      {
        tgt_jnt_pos_(ii) = joints_[ii]->getPositionMax();
      }
      else if (tgt_jnt_pos_(ii) < joints_[ii]->getPositionMin())
      {
        tgt_jnt_pos_(ii) = joints_[ii]->getPositionMin();
      }
    }
  }


  for (size_t ii = 0; ii < joints_.size(); ++ii)
  {
    joints_[ii]->setPosition(tgt_jnt_pos_(ii), tgt_jnt_vel_(ii), 0.0);
    last_tgt_jnt_vel_(ii) = tgt_jnt_vel_(ii);
  }
}

void CartesianTwistController::command(const geometry_msgs::msg::TwistStamped::SharedPtr goal)
{
  // Need to initialize KDL structs
  if (!initialized_)
  {
    RCLCPP_ERROR(rclcpp::get_logger(getName()),
                 "Cannot accept goal, controller is not initialized.");
    return;
  }

  if (goal->header.frame_id.empty())
  {
    manager_->requestStop(getName());
    return;
  }

  KDL::Twist twist;
  twist(0) = goal->twist.linear.x;
  twist(1) = goal->twist.linear.y;
  twist(2) = goal->twist.linear.z;
  twist(3) = goal->twist.angular.x;
  twist(4) = goal->twist.angular.y;
  twist(5) = goal->twist.angular.z;

  for (int ii=0; ii < 6; ++ii)
  {
    if (!std::isfinite(twist(ii)))
    {
      RCLCPP_ERROR(rclcpp::get_logger(getName()),
                   "Twist command value (%d) is not finite : %f", ii, twist(ii));
      twist(ii) = 0.0;
    }
  }

  rclcpp::Time now = node_->now();

  {
    std::lock_guard<std::mutex> lock(mutex_);
    twist_command_frame_ = goal->header.frame_id;
    twist_command_ = twist;
    last_command_time_ = now;
  }

  // Try to start up
  if (!is_active_ && manager_->requestStart(getName()) != 0)
  {
    RCLCPP_ERROR(rclcpp::get_logger(getName()),
                 "Cannot start, blocked by another controller.");
    return;
  }
}

std::vector<std::string> CartesianTwistController::getCommandedNames()
{
  std::vector<std::string> names;
  if (initialized_)
  {
    for (size_t i = 0; i < kdl_chain_.getNrOfSegments(); ++i)
      if (kdl_chain_.getSegment(i).getJoint().getType() != KDL::Joint::None)
        names.push_back(kdl_chain_.getSegment(i).getJoint().getName());
  }
  return names;
}

std::vector<std::string> CartesianTwistController::getClaimedNames()
{
  return getCommandedNames();
}

}  // namespace robot_controllers
