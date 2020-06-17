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

#ifndef ROBOT_CONTROLLERS__MIMIC_JOINT_HANDLE_H_
#define ROBOT_CONTROLLERS__MIMIC_JOINT_HANDLE_H_

#include <memory>
#include <string>

#include "robot_controllers_interface/joint_handle.h"

namespace robot_controllers
{

/**
 * @brief Joint handle for scaled mimic controller
 */
class MimicJointHandle : public robot_controllers_interface::JointHandle
{
public:
  MimicJointHandle(const std::string& name,
                   robot_controllers_interface::JointHandlePtr mimic);

  /** @brief Ensure proper cleanup with virtual destructor. */
  virtual ~MimicJointHandle() = default;

  /**
   * @brief Set a position command for this joint.
   * @param position Position command in radians or meters.
   * @param velocity Velocity command in rad/sec or meters/sec.
   * @param effort Effort command in Nm or N.
   */
  virtual void setPosition(double position, double velocity, double effort);

  /**
   * @brief Set a velocity command for this joint.
   * @param velocity Velocity command in rad/sec or meters/sec.
   * @param effort Effort command in Nm or N.
   */
  virtual void setVelocity(double velocity, double effort);

  /**
   * @brief Set an effort command for this joint.
   * @param effort Effort command in Nm or N.
   */
  virtual void setEffort(double effort);

  /** @brief Get the position of the joint in radians or meters. */
  virtual double getPosition();

  /** @brief Get the velocity of the joint in rad/sec or meters/sec. */
  virtual double getVelocity();

  /** @brief Get applied effort of a joint in Nm or N. */
  virtual double getEffort();

  /** @brief Is this joint continuous (has no position limits). */
  virtual bool isContinuous();

  /** @brief Get the minimum valid position command. */
  virtual double getPositionMin();

  /** @brief Get the maximum valid position command. */
  virtual double getPositionMax();

  /** @brief Get the maximum velocity command. */
  virtual double getVelocityMax();

  /** @brief Get the maximum effort command. */
  virtual double getEffortMax();

  /** @brief Get the name of this joint. */
  virtual std::string getName();

  /** @brief Reset the command. */
  virtual void reset();

private:
  double position_, velocity_, effort_;
  bool continuous_;
  double min_pos_, max_pos_, max_vel_, max_eff_;
  std::string name_;

  // No copy
  MimicJointHandle(const MimicJointHandle&) = delete;
  MimicJointHandle& operator=(const MimicJointHandle&) = delete;
};


}  // namespace robot_controllers

#endif  // ROBOT_CONTROLLERS__MIMIC_JOINT_HANDLE_H_
