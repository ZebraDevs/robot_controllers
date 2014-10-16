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

#ifndef ROBOT_CONTROLLERS_INTERFACE_CONTROLLER_MANAGER_H
#define ROBOT_CONTROLLERS_INTERFACE_CONTROLLER_MANAGER_H

#include <string>
#include <ros/ros.h>
#include <pluginlib/class_loader.h>

#include <robot_controllers_interface/controller.h>
#include <robot_controllers_interface/joint_handle.h>

namespace robot_controllers
{

#include <ros/ros.h>

/** @brief Base class for a controller manager. */
class ControllerManager
{
  typedef pluginlib::ClassLoader<robot_controllers::Controller> ControllerLoader;
  typedef std::vector<ControllerPtr> ControllerList;
  typedef std::vector<JointHandlePtr> JointHandleList;

public:
  ControllerManager() :
    loader_("robot_controllers", "robot_controllers::Controller")
  {
  }

  /** @brief Ensure proper shutdown with virtual destructor. */
  virtual ~ControllerManager()
  {
  }

  /**
   * @brief Startup the controller manager, loading default controllers.
   * @param nh The proper node handle for finding parameters.
   * @returns 0 if success, negative values are error codes.
   */
  virtual int init(ros::NodeHandle& nh)
  {
    // Find and load default controllers
    XmlRpc::XmlRpcValue controller_params;
    if (nh.getParam("default_controllers", controller_params))
    {
      if (controller_params.getType() != XmlRpc::XmlRpcValue::TypeArray)
      {
        ROS_ERROR("Parameter 'default_controllers' should be a list.");
      }
      else
      {
        // Load each controller
        for (int c = 0; c < controller_params.size(); c++)
        {
          XmlRpc::XmlRpcValue &controller_name = controller_params[c];
          if (controller_name.getType() != XmlRpc::XmlRpcValue::TypeString)
            ROS_WARN("Controller name is not a string?");
          load(static_cast<std::string>(controller_name));
        }
      }
    }
    else
    {
      ROS_WARN("No controllers loaded.");
    }
    return 0;
  }

  /** @brief Start a controller. */
  virtual int requestStart(const std::string& name)
  {
    // Make sure controller is not running
    for (ControllerList::iterator c = active_.begin(); c != active_.end(); c++)
      if ((*c)->getName() == name)
        return 0;  // already running

    // Find requested controller
    size_t c = 0;
    for (c = 0; c < controllers_.size(); ++c)
      if (controllers_[c]->getName() == name)
        break;

    if (c == controllers_.size())
      return -1;  // No such controller

    // Check for conflicts
    std::vector<std::string> names = controllers_[c]->getCommandedNames();
    for (ControllerList::iterator a = active_.begin(); a != active_.end(); a++)
    {
      std::vector<std::string> names2 = (*a)->getClaimedNames();
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
        if ((*a)->stop(false))
        {
          ROS_INFO_STREAM("Stopped " << (*a)->getName().c_str());
          active_.erase(a);
        }
        else
        {
          // Unable to stop a, cannot start c
          return -1;
        }
      }
    }

    // Start controller
    if (controllers_[c]->start())
    {
      ROS_INFO_STREAM("Started " << controllers_[c]->getName().c_str());
      active_.push_back(controllers_[c]);  
      return 0;
    }

    return -1;  // Unable to start
  }

  /** @brief Stop a controller. */
  virtual int requestStop(const std::string& name)
  {
    // Find controller
    for (ControllerList::iterator c = active_.begin(); c != active_.end(); c++)
    {
      if ((*c)->getName() == name)
      {
        // Stop controller (with force)
        if ((*c)->stop(true))
        {
          ROS_INFO_STREAM("Stopped " << (*c)->getName().c_str());
          active_.erase(c);
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

  /** @brief Update active controllers. */
  virtual void update(const ros::Time& time, const ros::Duration& dt)
  {
    // Reset handles
    for (JointHandleList::iterator j = joints_.begin(); j != joints_.end(); j++)
      j->reset();

    // Update controllers
    for (ControllerList::iterator c = active_.begin(); c != active_.end(); c++)
    {
      (*c)->update(time, dt);
    }
  }

  /** @brief Add a joint handle. */
  bool addJointHandle(JointHandlePtr& j)
  {
    // TODO: check for duplicate names?
    joints_.push_back(j);
  }

  /**
   * @brief Get the JointHandle associated with a particular joint name.
   * @param name The name of the joint.
   */
  JointHandlePtr getJointHandle(const std::string& name)
  {
    for (size_t j = 0; j < joints_.size(); j++)
    {
      if (joints_[j]->getName() == name)
        return joints_[j];
    }
    return JointHandlePtr();
  }

protected:
  /** @brief Load a controller by name. */
  virtual bool load(const std::string& name)
  {
    ros::NodeHandle nh(name);
    std::string controller_type;

    if (nh.getParam("type", controller_type))
    {
      ControllerPtr controller = loader_.createInstance(controller_type);
      controller->init(nh, this);
      controllers_.push_back(controller);
      return true;
    }

    ROS_ERROR_STREAM("Unable to load controller " << name.c_str());
    return false;
  }

private:
  ControllerLoader loader_;
  ControllerList controllers_;
  ControllerList active_;
  JointHandleList joints_;
};

}  // namespace robot_controllers

#endif  // ROBOT_CONTROLLERS_INTERFACE_CONTROLLER_MANAGER_H
