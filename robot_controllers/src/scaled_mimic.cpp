/*********************************************************************
 *  Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, Fetch Robotics Inc.
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

#include <string>
#include <vector>

#include "pluginlib/class_list_macros.hpp"
#include "robot_controllers/scaled_mimic.h"
#include "robot_controllers/mimic_joint_handle.h"

PLUGINLIB_EXPORT_CLASS(robot_controllers::ScaledMimicController,
                       robot_controllers_interface::Controller)

namespace robot_controllers
{

int ScaledMimicController::init(const std::string& name,
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

  // Setup Joints, Params
  std::string mimic = node->declare_parameter<std::string>(getName() + ".mimic_joint",
                                                           "torso_lift_joint");
  joint_to_mimic_ = manager->getJointHandle(mimic);
  if (joint_to_mimic_ == nullptr)
  {
    RCLCPP_ERROR(rclcpp::get_logger(getName()), "Could not get %s", mimic.c_str());
    initialized_ = false;
    return -1;
  }
  std::string controlled = node->declare_parameter<std::string>(getName() + ".controlled_joint", "bellows_joint");
  joint_to_control_ = manager->getJointHandle(controlled);
  if (joint_to_control_ == nullptr)
  {
    RCLCPP_WARN(rclcpp::get_logger(getName()), "Could not get %s, automatically creating it", controlled.c_str());
    robot_controllers_interface::JointHandlePtr joint =
      std::make_shared<MimicJointHandle>(controlled, joint_to_mimic_);
    manager->addJointHandle(joint);
    joint_to_control_ = joint;
  }
  scale_ = node->declare_parameter<double>(getName() + ".mimic_scale", 1.0);

  initialized_ = true;

  // Should we autostart?
  bool autostart = node->declare_parameter<bool>(getName() + ".autostart", false);
  if (autostart)
    manager->requestStart(getName());

  return 0;
}

bool ScaledMimicController::start()
{
  if (!initialized_)
  {
    // Can't log, no logger available
    return false;
  }
  return true;
}

bool ScaledMimicController::stop(bool force)
{
  (void) force;

  // Stop me anytime
  return true;
}

bool ScaledMimicController::reset()
{
  // Nothing to do here
  return true;
}

void ScaledMimicController::update(const rclcpp::Time& now, const rclcpp::Duration& dt)
{
  (void) now;
  (void) dt;

  if (!initialized_)
    return;

  joint_to_control_->setPosition(scale_ * joint_to_mimic_->getPosition(), 0, 0);
}

std::vector<std::string> ScaledMimicController::getCommandedNames()
{
  std::vector<std::string> names;
  names.push_back(joint_to_control_->getName());
  return names;
}

std::vector<std::string> ScaledMimicController::getClaimedNames()
{
  // Claimed == commanded.
  return getCommandedNames();
}

}  // namespace robot_controllers
