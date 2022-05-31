/*
 * Copyright (c) 2020-2022, Michael Ferguson
 * Copyright (c) 2014-2017, Fetch Robotics Inc.
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

#include "rclcpp/logging.hpp"
#include "robot_controllers_interface/controller_loader.h"

namespace robot_controllers_interface
{

ControllerLoader::ControllerLoader() :
    plugin_loader_("robot_controllers_interface", "robot_controllers_interface::Controller"),
    active_(false)
{
}

bool ControllerLoader::init(const std::string& name,
                            std::shared_ptr<rclcpp::Node> node,
                            std::shared_ptr<ControllerManager> manager)
{
  std::string controller_type = node->declare_parameter<std::string>(name + ".type", "");
  if (!controller_type.empty())
  {
    // If plugin is bad, catch pluginlib exception
    try
    {
      controller_ = plugin_loader_.createSharedInstance(controller_type);
      controller_->init(name, node, manager);
    }
    catch (pluginlib::LibraryLoadException& e)
    {
      RCLCPP_ERROR_STREAM(node->get_logger(),
                          "Plugin error while loading controller: " << e.what());
      return false;
    }
    return true;
  }

  RCLCPP_ERROR(node->get_logger(), "Unable to load controller %s", name.c_str());
  return false;
}

bool ControllerLoader::start()
{
  active_ = controller_->start();
  return active_;
}

bool ControllerLoader::stop(bool force)
{
  bool stopped = controller_->stop(force);
  if (stopped)
  {
    active_ = false;
  }
  return stopped;
}

bool ControllerLoader::reset()
{
  if (active_)
  {
    return controller_->reset();
  }
  return true;
}

bool ControllerLoader::isActive()
{
  return active_;
}

void ControllerLoader::update(const rclcpp::Time& time, const rclcpp::Duration& dt)
{
  if (active_)
  {
    controller_->update(time, dt);
  }
}

ControllerPtr ControllerLoader::getController()
{
  return controller_;
}

}  // namespace robot_controllers_interface
