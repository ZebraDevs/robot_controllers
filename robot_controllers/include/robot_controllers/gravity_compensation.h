/*********************************************************************
 * Software License Agreement (BSD License)
 *
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

#ifndef ROBOT_CONTROLLERS_GRAVITY_COMPENSATION_H_
#define ROBOT_CONTROLLERS_GRAVITY_COMPENSATION_H_

#include <string>
#include <vector>
#include <boost/shared_ptr.hpp>

#include <ros/ros.h>
#include <robot_controllers_interface/controller.h>
#include <robot_controllers_interface/controller_manager.h>
#include <robot_controllers_interface/joint_handle.h>

#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>

#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jntarrayvel.hpp>
#include <kdl/chaindynparam.hpp>

namespace robot_controllers
{

/**
 *  @class GravityCompensation
 *  @brief Controller which uses KDL to compute torque needed for static
 *         holding of the chain at the current pose.
 */
class GravityCompensation : public Controller
{
public:
  GravityCompensation() : initialized_(false) {}
  virtual ~GravityCompensation() {}

  /** @brief Initialize parameters, interfaces */
  virtual int init(ros::NodeHandle& nh, ControllerManager* manager);

  /** @brief Start the controller. */
  virtual bool start();

  /**
   *  @brief Stop this controller.
   *  @param force If true, this controller will be stopped regardless
   *         of return value.
   *  @returns true if controller preempted successfully.
   */
  virtual bool stop(bool force)
  {
    // always allow preemption
    return true;
  }

  /** @brief Update controller, called from controller_manager update */
  virtual void update(const ros::Time& time, const ros::Duration& dt);

  /** @brief Get the names of joints/controllers which this controller commands. */
  virtual std::vector<std::string> getCommandedNames();

  /** @brief Get the names of joints/controllers which this controller exclusively claims. */
  virtual std::vector<std::string> getClaimedNames();

private:
  ControllerManager* manager_;
  std::vector<JointHandlePtr> joints_;

  bool initialized_;  /// is KDL structure setup

  KDL::Chain kdl_chain_;
  KDL::JntArrayVel positions_;
  boost::shared_ptr<KDL::ChainDynParam> kdl_chain_dynamics_;
};

}  // namespace robot_controllers

#endif  // ROBOT_CONTROLLERS_GRAVITY_COMPENSATION_H_
