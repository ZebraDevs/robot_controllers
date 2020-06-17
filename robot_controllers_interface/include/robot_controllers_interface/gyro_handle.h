/*
 * Copyright (c) 2020, Fetch Robotics Inc.
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

// Author: Vincenzo Barbato, Carl Saldanha

#ifndef ROBOT_CONTROLLERS_INTERFACE__GYRO_HANDLE_H_
#define ROBOT_CONTROLLERS_INTERFACE__GYRO_HANDLE_H_

#include <memory>
#include <string>

#include "robot_controllers_interface/handle.h"

namespace robot_controllers_interface
{

/**
 * @brief Base class for a gyro handle used to provide controller 
 *        direct access to gyro data though controller interface.
 */
class GyroHandle : public Handle
{
public:
  GyroHandle()
  {
  }

  /** @brief Ensure proper cleanup with virtual destructor. */
  virtual ~GyroHandle()
  {
  }

  /** @brief Get the type of this gyro */
  virtual std::string getType() = 0;

  /** @brief Get the name of this gyro */
  virtual std::string getName() = 0;

  /** @brief Get if the data from the Gyro is currently valid. */
  virtual bool isValid() = 0;

  /** @brief Get the angular velocity in X direction */
  virtual double getAngularVelocityX() = 0;

  /** @brief Get the angular velocity in Y direction */
  virtual double getAngularVelocityY() = 0;

  /** @brief Get the angular velocity in Z direction */
  virtual double getAngularVelocityZ() = 0;

private:
  // No copy
  GyroHandle(const GyroHandle&) = delete;
  GyroHandle& operator=(const GyroHandle&) = delete;
};

using GyroHandlePtr = std::shared_ptr<GyroHandle>;

}  // namespace robot_controllers_interface

#endif  // ROBOT_CONTROLLERS_INTERFACE__GYRO_HANDLE_H_
