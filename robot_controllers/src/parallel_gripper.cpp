/*********************************************************************
 *  Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, Fetch Robotics Inc.
 *  Copyright (c) 2014, Unbounded Robotics Inc.
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
 *   * Neither the name of Fetch Robotics Inc. nor the names of its
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

#include <cmath>
#include <pluginlib/class_list_macros.hpp>
#include <robot_controllers/parallel_gripper.h>
#include <robot_controllers_interface/utils.h>

PLUGINLIB_EXPORT_CLASS(robot_controllers::ParallelGripperController, robot_controllers_interface::Controller)

namespace robot_controllers
{

using namespace std::placeholders;
using robot_controllers_interface::get_safe_topic_name;
using robot_controllers_interface::to_sec;

int ParallelGripperController::init(const std::string& name,
                                    rclcpp::Node::SharedPtr node,
                                    robot_controllers_interface::ControllerManagerPtr manager)
{
  // We absolutely need access to the controller manager
  if (!manager)
  {
    server_.reset();
    return -1;
  }

  Controller::init(name, node, manager);
  node_ = node;
  manager_ = manager;

  // Get parameters
  max_position_ = node_->declare_parameter<double>(name + ".max_position", 0.1);
  max_effort_ = node_->declare_parameter<double>(name + ".max_effort", 10.0);
  std::string l_name = node_->declare_parameter<std::string>(name + ".l_gripper_joint", "l_gripper_finger_joint");
  std::string r_name = node_->declare_parameter<std::string>(name + ".r_gripper_joint", "r_gripper_finger_joint");
  use_centering_controller_ = node_->declare_parameter<bool>(name + ".use_centering", false);

  // Setup joints
  left_ = manager_->getJointHandle(l_name);
  right_ = manager_->getJointHandle(r_name);

  // Checking to see if Joint Handles exists
  if (!left_)
  {
    RCLCPP_ERROR(rclcpp::get_logger(getName()),
                 "Unable to retrieve joint (%s), Namespace: %s/l_gripper_joint",
                 l_name.c_str(), name.c_str());
    return -1;  
  }
  
  if (!right_)
  {
    RCLCPP_ERROR(rclcpp::get_logger(getName()),
                 "Unable to retrieve joint (%s), Namespace: %s/r_gripper_joint",
                 r_name.c_str(), name.c_str());
    return -1;
  } 

  // Timer to publish feedback
  publish_timer_ = node->create_wall_timer(std::chrono::milliseconds(20),
                     std::bind(&ParallelGripperController::publishCallback, this));

  // Start action server
  active_goal_.reset();
  server_ = rclcpp_action::create_server<GripperCommandAction>(
    node_->get_node_base_interface(),
    node_->get_node_clock_interface(),
    node_->get_node_logging_interface(),
    node_->get_node_waitables_interface(),
    robot_controllers_interface::get_safe_topic_name(name),
    std::bind(&ParallelGripperController::handle_goal, this, _1, _2),
    std::bind(&ParallelGripperController::handle_cancel, this, _1),
    std::bind(&ParallelGripperController::handle_accepted, this, _1)
  );

  // Set default to max
  goal_ = max_position_;
  effort_ = max_effort_;

  return 0;
}

bool ParallelGripperController::start()
{
  if (!server_)
  {
    // No logger available yet
    return false;
  }

  if (!active_goal_)
  {
    RCLCPP_ERROR(rclcpp::get_logger(getName()),
                 "Unable to start, action server has no goal.");
    return false;
  }

  return true;
}

bool ParallelGripperController::stop(bool force)
{
  if (!server_)
    return true;

  if (active_goal_)
  {
    if (force)
    {
      // Shut down the action
      auto result = std::make_shared<GripperCommandAction::Result>();
      result->position = feedback_->position;
      result->effort = feedback_->effort;
      result->reached_goal = false;
      result->stalled = false;
      RCLCPP_DEBUG(rclcpp::get_logger(getName()),
                   "Goal preempted.");
      active_goal_->abort(result);
      active_goal_.reset();
      return true;
    }

    // Do not abort unless forced
    return false;
  }

  // Just holding position, go ahead and stop us
  return true;
}

bool ParallelGripperController::reset()
{
  // Nothing to do here
  return true;
}

void ParallelGripperController::update(const rclcpp::Time& now, const rclcpp::Duration& dt)
{
  if (!server_)
    return;

  if (use_centering_controller_)
  {
    double position = left_->getPosition() + right_->getPosition();
    double effort = std::fabs(effort_);
    if (goal_ < position)
    {
      effort = -effort;
    }

    double offset = centering_pid_.update(left_->getPosition() - right_->getPosition(), to_sec(dt));

    left_->setEffort(effort - offset);
    right_->setEffort(effort + offset);
  }
  else
  {
    left_->setPosition(goal_/2.0, 0, effort_);
    right_->setPosition(goal_/2.0, 0, effort_);
  }

  feedback_->position = left_->getPosition() + right_->getPosition();
  feedback_->effort = left_->getEffort() + right_->getEffort();
  feedback_->reached_goal = false;
  feedback_->stalled = false;

  // Goal detection
  if (std::fabs(feedback_->position - active_goal_->get_goal()->command.position) < 0.002)
  {
    auto result = std::make_shared<GripperCommandAction::Result>();
    result->position = feedback_->position;
    result->effort = feedback_->effort;
    result->reached_goal = true;
    result->stalled = false;
    RCLCPP_DEBUG(rclcpp::get_logger(getName()),
                 "Goal succeeded.");
    active_goal_->succeed(result);
    active_goal_.reset();
    return;
  }

  // Stall detection
  if (std::fabs(feedback_->position - last_position_) > 0.005)
  {
    last_position_ = feedback_->position;
    last_position_time_ = node_->now();
  }
  else
  {
    if (node_->now() - last_position_time_ > rclcpp::Duration(2, 0))
    {
      auto result = std::make_shared<GripperCommandAction::Result>();
      result->position = feedback_->position;
      result->effort = feedback_->effort;
      result->reached_goal = false;
      result->stalled = true;
      RCLCPP_DEBUG(rclcpp::get_logger(getName()),
                   "ParallelGripperController sstalled, but succeeding.");
      active_goal_->succeed(result);
      active_goal_.reset();
      return;
    }
  }
}

rclcpp_action::GoalResponse ParallelGripperController::handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const GripperCommandAction::Goal> goal_handle)
{
  if (!server_)
  {
    // Can't even log here - we aren't initialized
    return rclcpp_action::GoalResponse::REJECT;
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse ParallelGripperController::handle_cancel(
    const std::shared_ptr<GripperCommandGoal> goal_handle)
{
  // Always accept
  if (active_goal_ && active_goal_->get_goal_id() == goal_handle->get_goal_id())
  {
    RCLCPP_INFO(rclcpp::get_logger(getName()),
                "Goal cancelled.");
    active_goal_.reset();
  }

  return rclcpp_action::CancelResponse::ACCEPT;
}

void ParallelGripperController::handle_accepted(const std::shared_ptr<GripperCommandGoal> goal_handle)
{
  auto result = std::make_shared<GripperCommandAction::Result>();
  feedback_ = std::make_shared<GripperCommandAction::Feedback>();

  if (active_goal_)
  {
    // TODO: if rclcpp_action ever supports preempted, note it here
    //       https://github.com/ros2/rclcpp/issues/1104
    active_goal_->abort(result);
    active_goal_.reset();
    RCLCPP_INFO(node_->get_logger(), "ParallelGripper goal preempted.");
  }

  const auto goal = goal_handle->get_goal();

  // Effort of 0.0 means use max
  if (goal->command.max_effort <= 0.0 || goal->command.max_effort > max_effort_)
  {
    effort_ = max_effort_;
  }
  else
  {
    effort_ = goal->command.max_effort;
  }

  // Set goal position
  if (goal->command.position > max_position_)
  {
    goal_ = max_position_;
  }
  else if (goal->command.position < 0.0)
  {
    goal_ = 0.0;
  }
  else
  {
    goal_ = goal->command.position;
  }

  // Track position/time for stall detection
  last_position_time_ = node_->now();
  // It's probably not safe to read joints in this thread
  // valid position ranges from 0->max, so this will absolutely
  // not trigger a stall detection on the next cycle.
  last_position_ = -1.0;

  active_goal_ = goal_handle;
  if (manager_->requestStart(getName()) != 0)
  {
    active_goal_->abort(result);
    active_goal_.reset();
    RCLCPP_ERROR(node_->get_logger(), "Cannot execute, unable to start controller.");
    return;
  }

  RCLCPP_INFO(node_->get_logger(), "ParallelGripper goal started.");
}

void ParallelGripperController::publishCallback()
{
  if (active_goal_)
  {
    active_goal_->publish_feedback(feedback_);
  }
}

std::vector<std::string> ParallelGripperController::getCommandedNames()
{
  std::vector<std::string> names;
  names.push_back(left_->getName());
  names.push_back(right_->getName());
  return names;
}

std::vector<std::string> ParallelGripperController::getClaimedNames()
{
  // Claimed == commanded.
  return getCommandedNames();
}

}  // namespace robot_controllers
