/*
 * Copyright (c) 2014, FYS Systems Inc.
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
 *     * Neither the name of the FYS Systems Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL FYS SYSTMEMS INC. BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

// Author: Michael Ferguson

#include <robot_controllers_interface/controller_manager.h>

namespace robot_controllers
{

ControllerManager::ControllerManager()
{
}

int ControllerManager::init(ros::NodeHandle& nh)
{
  // Find and load default controllers
  XmlRpc::XmlRpcValue controller_params;
  if (nh.getParam("default_controllers", controller_params))
  {
    if (controller_params.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
      ROS_ERROR_NAMED("ControllerManager", "Parameter 'default_controllers' should be a list.");
      return -1;
    }
    else
    {
      // Load each controller
      for (int c = 0; c < controller_params.size(); c++)
      {
        // Make sure name is valid
        XmlRpc::XmlRpcValue &controller_name = controller_params[c];
        if (controller_name.getType() != XmlRpc::XmlRpcValue::TypeString)
        {
          ROS_WARN_NAMED("ControllerManager", "Controller name is not a string?");
          continue;
        }

        // Create controller (in a loader)
        load(static_cast<std::string>(controller_name));
      }
    }
  }
  else
  {
    ROS_WARN_NAMED("ControllerManager", "No controllers loaded.");
    return -1;
  }

  return 0;
}

int ControllerManager::requestStart(const std::string& name)
{
  // Find requested controller
  ControllerLoaderPtr controller;
  for (ControllerList::iterator c = controllers_.begin(); c != controllers_.end(); c++)
  {
    if ((*c)->getController()->getName() == name)
    {
      controller = *c;
      break;
    }
  }

  // Does controller exist?
  if (!controller)
    return -1;

  // Is controller already running?
  if (controller->isActive())
  {
    return 0;
  }

  // Check for conflicts
  std::vector<std::string> names = controller->getController()->getCommandedNames();
  for (ControllerList::iterator c = controllers_.begin(); c != controllers_.end(); c++)
  {
    // Only care about active controllers
    if (!(*c)->isActive())
      continue;

    std::vector<std::string> names2 = (*c)->getController()->getClaimedNames();
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
      if ((*c)->stop(false))
      {
        ROS_INFO_STREAM_NAMED("ControllerManager", "Stopped " << (*c)->getController()->getName().c_str());
      }
      else
      {
        // Unable to stop c, cannot start controller
        return -1;
      }
    }
  }

  // Start controller
  if (controller->start())
  {
    ROS_INFO_STREAM_NAMED("ControllerManager", "Started " << controller->getController()->getName().c_str());
    return 0;
  }

  return -1;  // Unable to start
}

int ControllerManager::requestStop(const std::string& name)
{
  // Find controller
  for (ControllerList::iterator c = controllers_.begin(); c != controllers_.end(); c++)
  {
    if ((*c)->getController()->getName() == name)
    {
      // Stop controller (with force)
      if ((*c)->stop(true))
      {
        ROS_INFO_STREAM_NAMED("ControllerManager", "Stopped " << (*c)->getController()->getName().c_str());
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

void ControllerManager::update(const ros::Time& time, const ros::Duration& dt)
{
  // Reset handles
  for (JointHandleList::iterator j = joints_.begin(); j != joints_.end(); j++)
    (*j)->reset();

  // Update controllers
  for (ControllerList::iterator c = controllers_.begin(); c != controllers_.end(); c++)
  {
    (*c)->update(time, dt);
  }
}

bool ControllerManager::addJointHandle(JointHandlePtr& j)
{
  // TODO: check for duplicate names?
  joints_.push_back(j);
}

HandlePtr ControllerManager::getHandle(const std::string& name)
{
  // Try joints first
  for (JointHandleList::iterator j = joints_.begin(); j != joints_.end(); j++)
  {
    if ((*j)->getName() == name)
      return *j;
  }

  // Then controllers
  for (ControllerList::iterator c = controllers_.begin(); c != controllers_.end(); c++)
  {
    if ((*c)->getController()->getName() == name)
      return (*c)->getController();
  }

  // Not found
  return HandlePtr();
}

JointHandlePtr ControllerManager::getJointHandle(const std::string& name)
{
  // Try joints first
  for (JointHandleList::iterator j = joints_.begin(); j != joints_.end(); j++)
  {
    if ((*j)->getName() == name)
      return *j;
  }

  // Not found
  return JointHandlePtr();
}

bool ControllerManager::load(const std::string& name)
{
  // Create controller (in a loader)
  ControllerLoaderPtr controller(new ControllerLoader());
  if (controller->init(name, this))
  {
    controllers_.push_back(controller);
    return true;
  }
  return false;
}

}  // namespace robot_controllers
