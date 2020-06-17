/*
 * Copyright (c) 2020, Michael Ferguson
 * Copyright (c) 2014-2015, Fetch Robotics Inc.
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

// Author: Michael Ferguson

#include <memory>
#include <string>
#include <sstream>
#include <vector>

#include "robot_controllers_interface/controller_manager.h"

namespace robot_controllers_interface
{

using namespace std::placeholders;

ControllerManager::ControllerManager()
{
}

int ControllerManager::init(std::shared_ptr<rclcpp::Node> node)
{
  node_ = node;

  // Find and load default controllers
  std::vector<std::string> controller_names =
    node_->declare_parameter<std::vector<std::string>>("default_controllers",
                                                       std::vector<std::string>());
  if (controller_names.empty())
  {
    RCLCPP_WARN(node_->get_logger(), "No controllers loaded.");
    return -1;
  }

  // Load each controller
  for (auto controller_name : controller_names)
  {
    RCLCPP_INFO(node->get_logger(), "Loading %s", controller_name.c_str());
    load(controller_name);
  }

  // Setup actionlib server
  server_ = node_->create_service<robot_controllers_msgs::srv::QueryControllerStates>(
    "query_controller_states",
    std::bind(&ControllerManager::callback, this, _1, _2)
  );

  return 0;
}

int ControllerManager::requestStart(const std::string& name)
{
  // Find requested controller
  ControllerLoaderPtr controller;
  for (const auto& c : controllers_)
  {
    if (c->getController()->getName() == name)
    {
      controller = c;
      break;
    }
  }

  // Does controller exist?
  if (!controller)
  {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "Unable to start " <<
                        name.c_str() << ": no such controller.");
    return -1;
  }

  // Is controller already running?
  if (controller->isActive())
  {
    RCLCPP_DEBUG_STREAM(node_->get_logger(), "Unable to start " <<
                        name.c_str() << ": already running.");
    return 0;
  }

  // Check for conflicts
  std::vector<std::string> names = controller->getController()->getCommandedNames();
  for (const auto& c : controllers_)
  {
    // Only care about active controllers
    if (!c->isActive())
      continue;

    std::vector<std::string> names2 = c->getController()->getClaimedNames();
    bool conflict = false;
    for (size_t i = 0; i < names.size(); i++)
    {
      for (size_t i2 = 0; i2 < names2.size(); i2++)
      {
        if (names[i] == names2[i2])
        {
          conflict = true;
          break;
        }
      }
      if (conflict) break;
    }
    // Have conflict, try to stop controller (without force)
    if (conflict)
    {
      if (c->stop(false))
      {
        RCLCPP_INFO_STREAM(node_->get_logger(), "Stopped " <<
                           c->getController()->getName().c_str());
      }
      else
      {
        // Unable to stop c, cannot start controller
        RCLCPP_ERROR_STREAM(node_->get_logger(), "Unable to stop " <<
                            c->getController()->getName().c_str() <<
                            " when trying to start " << name.c_str());
        return -1;
      }
    }
  }

  // Start controller
  if (controller->start())
  {
    RCLCPP_INFO_STREAM(node_->get_logger(), "Started " <<
                       controller->getController()->getName().c_str());
    return 0;
  }

  return -1;  // Unable to start
}

int ControllerManager::requestStop(const std::string& name)
{
  // Find controller
  for (const auto& c : controllers_)
  {
    if (c->getController()->getName() == name)
    {
      // Stop controller (with force)
      if (c->stop(true))
      {
        RCLCPP_INFO_STREAM(node_->get_logger(), "Stopped " <<
                           c->getController()->getName().c_str());
        return 0;
      }
      else
      {
        return -1;  // controller decided not to stop
      }
    }
  }
  return -1;  // no such controller
}

void ControllerManager::update(const rclcpp::Time& time, const rclcpp::Duration& dt)
{
  // Reset handles
  for (const auto& joint_handle : joints_)
    joint_handle->reset();

  // Update controllers
  for (const auto& c : controllers_)
  {
    c->update(time, dt);
  }
}

void ControllerManager::reset()
{
  // Update controllers
  for (const auto& c : controllers_)
  {
    c->reset();
  }
}

bool ControllerManager::addJointHandle(JointHandlePtr& joint_handle_ptr)
{
  // If we don't already have a handle with
  // a duplicate name, and it is not null we add it
  if (!joint_handle_ptr)
  {
    return false;
  }
  for (const auto& joint_handle : joints_)
  {
    if (joint_handle->getName() == joint_handle_ptr->getName())
    return false;
  }

  joints_.push_back(joint_handle_ptr);
  return true;
}

bool ControllerManager::addGyroHandle(GyroHandlePtr gyro_handle_ptr)
{
  // If we don't already have a handle with
  // a duplicate name, and it is not null we add it
  if (!gyro_handle_ptr)
  {
    return false;
  }
  for (const auto& gyro_handle : gyros_)
  {
    if (gyro_handle->getName() == gyro_handle_ptr->getName())
    return false;
  }

  gyros_.push_back(gyro_handle_ptr);
  return true;
}

HandlePtr ControllerManager::getHandle(const std::string& name)
{
  // Try joints first
  for (const auto& j : joints_)
  {
    if (j->getName() == name)
    {
      return j;
    }
  }

  // Then gyros
  for (const auto& gyro_handle_pointer : gyros_)
  {
    if (gyro_handle_pointer->getName() == name)
    {
      return gyro_handle_pointer;
    }
  }

  // Then controllers
  for (const auto& controller : controllers_)
  {
    if (controller->getController()->getName() == name)
    {
      return controller->getController();
    }
  }

  // Not found
  return HandlePtr{};
}

JointHandlePtr ControllerManager::getJointHandle(const std::string& name)
{
  // Try joints first
  for (const auto& j : joints_)
  {
    if (j->getName() == name)
    {
      return j;
    }
  }

  // Not found
  return JointHandlePtr();
}

GyroHandlePtr ControllerManager::getGyroHandle(const std::string& name)
{
  for (const auto& gyro_handle_pointer : gyros_)
  {
    if (gyro_handle_pointer->getName() == name)
    {
      return gyro_handle_pointer;
    }
  }

  // Not found
  return GyroHandlePtr();
}

void ControllerManager::callback(
    const std::shared_ptr<robot_controllers_msgs::srv::QueryControllerStates::Request> request,
    std::shared_ptr<robot_controllers_msgs::srv::QueryControllerStates::Response> response)
{
  for (auto state : request->updates)
  {
    // Make sure controller exists
    bool in_controller_list = false;
    for (const auto& c : controllers_)
    {
      if (c->getController()->getName() == state.name)
      {
        if (state.type != "")
        {
          if (state.type == c->getController()->getType())
          {
            in_controller_list = true;
            break;
          }
          else
          {
            RCLCPP_ERROR(node_->get_logger(), "Controller %s is of type %s not %s",
                                              state.name,
                                              c->getController()->getType(),
                                              state.type);
            return;
          }
        }
        in_controller_list = true;
        break;
      }
    }
    if (!in_controller_list)
    {
      // Create controller (in a loader)
      if (!load(static_cast<std::string>(state.name)))
      {
        RCLCPP_ERROR(node_->get_logger(), "Failed to load controller %s", state.name.c_str());
        return;
      }
    }

    // Update state
    if (state.state == state.STOPPED)
    {
      if (requestStop(state.name) != 0)
      {
        RCLCPP_ERROR(node_->get_logger(), "Unable to stop %s", state.name.c_str());
        return;
      }
    }
    else if (state.state == state.RUNNING)
    {
      if (requestStart(state.name) != 0)
      {
        RCLCPP_ERROR(node_->get_logger(), "Unable to start %s", state.name.c_str());
        return;
      }
    }
    else
    {
      RCLCPP_ERROR(node_->get_logger(), "Invalid state for controller %s: %d",
                                        state.name,
                                        static_cast<int>(state.state));
      return;
    }
  }

  // Send result
  getState(response->state);
}

void ControllerManager::getState(
    std::vector<robot_controllers_msgs::msg::ControllerState>& states)
{
  states.clear();
  for (auto& c : controllers_)
  {
    robot_controllers_msgs::msg::ControllerState state;
    state.name = c->getController()->getName();
    state.type = c->getController()->getType();
    if (c->isActive())
    {
      state.state = state.RUNNING;
    }
    else
    {
      state.state = state.STOPPED;
    }
    states.push_back(state);
  }
}

// NOTE: this function should be called only by one thread
bool ControllerManager::load(const std::string& name)
{
  // Create controller (in a loader)
  ControllerLoaderPtr controller(new ControllerLoader());
  // Push back controller (so that autostart will work)
  controllers_.push_back(controller);
  // Now initialize controller
  if (!controller->init(name, node_, shared_from_this()))
  {
    // Remove if init fails
    controllers_.pop_back();
    return false;
  }
  return true;
}

}  // namespace robot_controllers_interface
