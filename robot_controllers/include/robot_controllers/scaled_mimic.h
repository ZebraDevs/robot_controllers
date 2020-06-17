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

#ifndef ROBOT_CONTROLLERS__SCALED_MIMIC_H_
#define ROBOT_CONTROLLERS__SCALED_MIMIC_H_

#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "robot_controllers_interface/controller.h"
#include "robot_controllers_interface/joint_handle.h"
#include "robot_controllers_interface/controller_manager.h"

namespace robot_controllers
{

/**
 * @brief Controller for a having one joint be a (scaled) mimic of another.
 *        Primary design use case is making a fake bellows joint that moves
 *        in relation to a torso lift joint.
 */
class ScaledMimicController : public robot_controllers_interface::Controller
{
public:
  ScaledMimicController() : initialized_(false) {}
  virtual ~ScaledMimicController() {}

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
    return "robot_controllers/ScaledMimicController";
  }

  /** @brief Get the names of joints/controllers which this controller commands. */
  virtual std::vector<std::string> getCommandedNames();

  /** @brief Get the names of joints/controllers which this controller exclusively claims. */
  virtual std::vector<std::string> getClaimedNames();

private:
  bool initialized_;
  robot_controllers_interface::JointHandlePtr joint_to_mimic_;
  robot_controllers_interface::JointHandlePtr joint_to_control_;
  double scale_;
};

}  // namespace robot_controllers

#endif  // ROBOT_CONTROLLERS__SCALED_MIMIC_H_
