/*********************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2020, Michael Ferguson
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *    * Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 * 
 *    * Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 * 
 *    * Neither the name of the copyright holder nor the names of its
 *      contributors may be used to endorse or promote products derived from
 *      this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#ifndef ROBOT_CONTROLLERS_INTERFACE__UTILS_H_
#define ROBOT_CONTROLLERS_INTERFACE__UTILS_H_

#include <algorithm>
#include <string>

#include "rclcpp/rclcpp.hpp"

namespace robot_controllers_interface
{

// Mainly used for robot_description
template <typename T>
T declare_parameter_once(const std::string& name, const T& default_value,
                         rclcpp::Node::SharedPtr node)
{
  if (!node->has_parameter(name))
  {
    node->declare_parameter<T>(name, default_value);
  }

  T value;
  node->get_parameter(name, value);
  return value;
}

// Parameters use ".", topics use "/"
inline std::string get_safe_topic_name(const std::string& name)
{
  std::string safe_name = name;
  std::replace(safe_name.begin(), safe_name.end(), '.', '/');
  return safe_name;
}

template <typename T>
double to_sec(const T& t)
{
  return (double) t.nanoseconds() / 1e9;
}

template <typename T>
double msg_to_sec(T timelike)
{
  return timelike.sec + (double) timelike.nanosec / 1e9;
}

}  // namespace robot_controllers_interface

#endif  // ROBOT_CONTROLLERS_INTERFACE__UTILS_H_
