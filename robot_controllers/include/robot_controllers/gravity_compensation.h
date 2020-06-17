/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, Michael Ferguson
 *  Copyright (c) 2014, Fetch Robotics Inc.
 *  Copyright (c) 2013, Unbounded Robotics Inc.
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

// Author: Michael Ferguson

#ifndef ROBOT_CONTROLLERS__GRAVITY_COMPENSATION_H_
#define ROBOT_CONTROLLERS__GRAVITY_COMPENSATION_H_

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "robot_controllers_interface/controller.h"
#include "robot_controllers_interface/controller_manager.h"
#include "robot_controllers_interface/joint_handle.h"

#include "urdf/model.h"
#include "kdl_parser/kdl_parser.hpp"

#include "kdl/tree.hpp"
#include "kdl/chain.hpp"
#include "kdl/jntarray.hpp"
#include "kdl/jntarrayvel.hpp"
#include "kdl/chaindynparam.hpp"

namespace robot_controllers
{

/**
 *  @class GravityCompensation
 *  @brief Controller which uses KDL to compute torque needed for static
 *         holding of the chain at the current pose.
 */
class GravityCompensation : public robot_controllers_interface::Controller
{
public:
  GravityCompensation() : initialized_(false) {}
  virtual ~GravityCompensation() {}

  /**
   * @brief Initialize the controller and any required data structures.
   * @param node Node handle for this controller.
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
   *  @brief Stop this controller.
   *  @param force If true, this controller will be stopped regardless
   *         of return value.
   *  @returns true if controller preempted successfully.
   */
  virtual bool stop(bool force)
  {
    (void) force;
    // always allow preemption
    return true;
  }

  /**
   * @brief Cleanly reset the controller to it's initial state. Some controllers
   *        may choose to stop themselves. This is mainly used in the case of the
   *        the robot exiting some fault condition.
   * @returns True if successfully reset, false otherwise.
   */
  virtual bool reset()
  {
    // Do nothing
    return true;
  }

  /**
   * @brief This is the update loop for the controller.
   * @param now The system time.
   * @param dt The timestep since last call to update.
   */
  virtual void update(const rclcpp::Time& now, const rclcpp::Duration& dt);

  /** @brief Get the type of this controller. */
  virtual std::string getType()
  {
    return "robot_controllers/GravityCompensation";
  }

  /** @brief Get the names of joints/controllers which this controller commands. */
  virtual std::vector<std::string> getCommandedNames();

  /** @brief Get the names of joints/controllers which this controller exclusively claims. */
  virtual std::vector<std::string> getClaimedNames();

private:
  robot_controllers_interface::ControllerManagerPtr manager_;
  std::vector<robot_controllers_interface::JointHandlePtr> joints_;

  bool initialized_;  /// is KDL structure setup

  KDL::Chain kdl_chain_;
  KDL::JntArrayVel positions_;
  std::shared_ptr<KDL::ChainDynParam> kdl_chain_dynamics_;
};

}  // namespace robot_controllers

#endif  // ROBOT_CONTROLLERS__GRAVITY_COMPENSATION_H_
