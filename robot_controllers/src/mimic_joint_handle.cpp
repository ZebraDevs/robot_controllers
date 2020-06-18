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

#include "robot_controllers/mimic_joint_handle.h"

namespace robot_controllers
{

MimicJointHandle::MimicJointHandle(const std::string & name,
                                   robot_controllers_interface::JointHandlePtr mimic) :
  position_(0.0),
  velocity_(0.0),
  effort_(0.0),
  continuous_(mimic->isContinuous()),
  min_pos_(mimic->getPositionMin()),
  max_pos_(mimic->getPositionMax()),
  max_vel_(mimic->getVelocityMax()),
  max_eff_(mimic->getEffortMax()),
  name_(name)
{
}

void MimicJointHandle::setPosition(double position, double velocity, double effort)
{
  // Just copy (mimic controller calls this)
  position_ = position;
  velocity_ = velocity;
  effort_ = effort;
}

void MimicJointHandle::setVelocity(double velocity, double effort)
{
  velocity_ = velocity;
  effort_ = effort;
}

void MimicJointHandle::setEffort(double effort)
{
  effort_ = effort;
}

double MimicJointHandle::getPosition()
{
  return position_;
}

double MimicJointHandle::getVelocity()
{
  return velocity_;
}

double MimicJointHandle::getEffort()
{
  return effort_;
}

bool MimicJointHandle::isContinuous()
{
  return continuous_;
}

double MimicJointHandle::getPositionMin()
{
  return min_pos_;
}

double MimicJointHandle::getPositionMax()
{
  return max_pos_;
}

double MimicJointHandle::getVelocityMax()
{
  return max_vel_;
}

double MimicJointHandle::getEffortMax()
{
  return max_eff_;
}

std::string MimicJointHandle::getName()
{
  return name_;
}

void MimicJointHandle::reset()
{
  // no-op
}

}  // namespace robot_controllers
