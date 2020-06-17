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

#include <pluginlib/class_list_macros.hpp>
#include <robot_controllers/cartesian_pose.h>
#include <robot_controllers_interface/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>

PLUGINLIB_EXPORT_CLASS(robot_controllers::CartesianPoseController, robot_controllers_interface::Controller)

namespace robot_controllers
{

using namespace std::placeholders;
using robot_controllers_interface::declare_parameter_once;
using robot_controllers_interface::get_safe_topic_name;
using robot_controllers_interface::to_sec;

CartesianPoseController::CartesianPoseController() :
    initialized_(false)
{
}

int CartesianPoseController::init(const std::string& name,
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
  root_link_ = node_->declare_parameter<std::string>(getName() + ".root", "torso_lift_link");
  std::string tip = node_->declare_parameter<std::string>(getName() + ".tip", "wrist_roll_link");

  // Load URDF
  urdf::Model model;
  std::string robot_description = declare_parameter_once<std::string>("robot_description", "", node);
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

  // Populate the chain
  if(!kdl_tree.getChain(root_link_, tip, kdl_chain_))
  {
    RCLCPP_ERROR(rclcpp::get_logger(getName()),
                 "Could not construct chain from URDF");
    return -1;
  }

  jnt_to_pose_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));
  jac_solver_.reset(new KDL::ChainJntToJacSolver(kdl_chain_));
  jnt_pos_.resize(kdl_chain_.getNrOfJoints());
  jnt_delta_.resize(kdl_chain_.getNrOfJoints());
  jacobian_.resize(kdl_chain_.getNrOfJoints());

  // Initialize controllers
  robot_controllers::PID pid_controller;
  if (!pid_controller.init(name + ".fb_trans", node_)) return false;
  for (unsigned int i = 0; i < 3; i++)
    pid_.push_back(pid_controller);
  if (!pid_controller.init(name + ".fb_rot", node_)) return false;
  for (unsigned int i = 0; i < 3; i++)
    pid_.push_back(pid_controller);

  // Init joint handles
  joints_.clear();
  for (size_t i = 0; i < kdl_chain_.getNrOfSegments(); ++i)
    if (kdl_chain_.getSegment(i).getJoint().getType() != KDL::Joint::None)
      joints_.push_back(manager_->getJointHandle(kdl_chain_.getSegment(i).getJoint().getName()));

  // Setup transform listener
  tf_buffer_.reset(new tf2_ros::Buffer(node_->get_clock()));
  tf_listener_.reset(new tf2_ros::TransformListener(*tf_buffer_));

  // Subscribe to command
  std::string topic_name = get_safe_topic_name(name) + "/command";
  command_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(topic_name, 1,
                    std::bind(&CartesianPoseController::command, this, _1));
  last_command_ = node_->now();

  // Feedback of twist
  topic_name = get_safe_topic_name(name) + "/feedback";
  feedback_pub_ = node->create_publisher<geometry_msgs::msg::TwistStamped>(topic_name, 10);

  initialized_ = true;
  return 0;
}

bool CartesianPoseController::start()
{
  if (!initialized_)
  {
    RCLCPP_ERROR(rclcpp::get_logger(getName()),
                 "Unable to start, not initialized.");
    return false;
  }

  if (node_->now() - last_command_ > rclcpp::Duration(3, 0))
  {
    RCLCPP_ERROR(rclcpp::get_logger(getName()),
                 "Unable to start, no goal.");
    return false;
  }

  return true;
}

bool CartesianPoseController::stop(bool force)
{
  // Always stop
  return true;
}

bool CartesianPoseController::reset()
{
  // Simply stop
  return (manager_->requestStop(getName()) == 0);
}

void CartesianPoseController::update(const rclcpp::Time& now, const rclcpp::Duration& dt)
{
  // Need to initialize KDL structs
  if (!initialized_)
    return;  // Should never actually hit this

  // Get current pose
  actual_pose_ = getPose();

  // Pose feedback
  twist_error_ = KDL::diff(actual_pose_, desired_pose_);
  geometry_msgs::msg::TwistStamped t;
  t.header.stamp = now;
  t.twist.linear.x = twist_error_(0);
  t.twist.linear.y = twist_error_(1);
  t.twist.linear.z = twist_error_(2);
  t.twist.angular.x = twist_error_(3);
  t.twist.angular.y = twist_error_(4);
  t.twist.angular.z = twist_error_(5);
  feedback_pub_->publish(t);

  // Update PID
  for (size_t i = 0; i < 6; ++i)
    twist_error_(i) = pid_[i].update(twist_error_(i), to_sec(dt));

  // Get jacobian
  jac_solver_->JntToJac(jnt_pos_, jacobian_);

  // Convert wrench to joint efforts
  for (size_t i = 0; i < kdl_chain_.getNrOfJoints(); ++i)
  {
    jnt_delta_(i) = 0.0;
    for (unsigned int j = 0; j < 6; ++j)
      jnt_delta_(i) += (jacobian_(j,i) * twist_error_(j));
  }

  // Actually update joints
  for (size_t j = 0; j < joints_.size(); ++j)
    joints_[j]->setPosition(jnt_delta_(j) + joints_[j]->getPosition(), 0.0, 0.0);
}

KDL::Frame CartesianPoseController::getPose()
{
  for (size_t i = 0; i < joints_.size(); ++i)
    jnt_pos_(i) = joints_[i]->getPosition();

  KDL::Frame result;
  jnt_to_pose_solver_->JntToCart(jnt_pos_, result);

  return result;
}

void CartesianPoseController::command(const geometry_msgs::msg::PoseStamped::SharedPtr goal)
{
  // Need to initialize KDL structs
  if (!initialized_)
  {
    RCLCPP_ERROR(rclcpp::get_logger(getName()),
                 "Cannot accept goal, controller is not initialized.");
    return;
  }

  try
  {
    geometry_msgs::msg::PoseStamped ps;
    tf_buffer_->transform(*goal, ps, root_link_);
    desired_pose_ = KDL::Frame(KDL::Rotation::Quaternion(ps.pose.orientation.x,
                                                         ps.pose.orientation.y,
                                                         ps.pose.orientation.z,
                                                         ps.pose.orientation.w),
                               KDL::Vector(ps.pose.position.x,
                                           ps.pose.position.y,
                                           ps.pose.position.z));
  }
  catch (const tf2::TransformException& ex)
  {
    RCLCPP_ERROR(rclcpp::get_logger(getName()),
                 "Could not transform goal.");
    return;
  }

  // Update last command time before trying to start controller
  last_command_ = node_->now();

  // Try to start up
  if (manager_->requestStart(getName()) != 0)
  {
    RCLCPP_ERROR(rclcpp::get_logger(getName()),
                 "Cannot start, blocked by another controller.");
    return;
  }
}

std::vector<std::string> CartesianPoseController::getCommandedNames()
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

std::vector<std::string> CartesianPoseController::getClaimedNames()
{
  // Commanded == claimed
  return getCommandedNames();
}


}  // namespace robot_controllers
