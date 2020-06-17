/*********************************************************************
 *  Software License Agreement (BSD License)
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

#include <memory>
#include <string>
#include <vector>

#include "pluginlib/class_list_macros.hpp"
#include "robot_controllers_interface/utils.h"
#include "robot_controllers/point_head.h"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "urdf/model.h"
#include "kdl_parser/kdl_parser.hpp"

PLUGINLIB_EXPORT_CLASS(robot_controllers::PointHeadController,
                       robot_controllers_interface::Controller)

namespace robot_controllers
{

using namespace std::placeholders;
using robot_controllers_interface::to_sec;
using robot_controllers_interface::msg_to_sec;
using robot_controllers_interface::declare_parameter_once;

int PointHeadController::init(const std::string& name,
                              rclcpp::Node::SharedPtr node,
                              robot_controllers_interface::ControllerManagerPtr manager)
{
  // We absolutely need access to the controller manager
  if (!manager)
  {
    RCLCPP_ERROR(rclcpp::get_logger(getName()),
                 "No controller manager available.");
    server_.reset();
    return -1;
  }

  Controller::init(name, node, manager);
  node_ = node;
  manager_ = manager;

  // No initial sampler
  std::lock_guard<std::mutex> lock(sampler_mutex_);
  sampler_.reset();

  // Get parameters
  stop_with_action_ = node->declare_parameter<bool>(name + ".stop_with_action", false);

  // Setup Joints
  head_pan_ = manager_->getJointHandle("head_pan_joint");
  head_tilt_ = manager_->getJointHandle("head_tilt_joint");

  // Parse UDRF/KDL
  urdf::Model model;
  std::string robot_description = declare_parameter_once<std::string>("robot_description",
                                                                      "", node);
  if (!model.initString(robot_description))
  {
    RCLCPP_ERROR(rclcpp::get_logger(getName()),
      "Failed to parse URDF, is robot_description parameter set?");
    return -1;
  }

  if (!kdl_parser::treeFromUrdfModel(model, kdl_tree_))
  {
    RCLCPP_ERROR(rclcpp::get_logger(getName()),
                 "Failed to construct KDL tree");
    return -1;
  }

  // Find parent of pan joint
  KDL::SegmentMap segment_map = kdl_tree_.getSegments();
  for (KDL::SegmentMap::iterator it = segment_map.begin();
       it != segment_map.end();
       ++it)
  {
#ifdef KDL_USE_NEW_TREE_INTERFACE
     if (it->second->segment.getJoint().getName() == head_pan_->getName())
       root_link_ = it->second->parent->first;
#else
    if (it->second.segment.getJoint().getName() == head_pan_->getName())
      root_link_ = it->second.parent->first;
#endif
  }

  // Setup transform listener
  tf_buffer_.reset(new tf2_ros::Buffer(node_->get_clock()));
  tf_listener_.reset(new tf2_ros::TransformListener(*tf_buffer_));

  // Start action server
  active_goal_.reset();
  server_ = rclcpp_action::create_server<PointHeadAction>(
    node_->get_node_base_interface(),
    node_->get_node_clock_interface(),
    node_->get_node_logging_interface(),
    node_->get_node_waitables_interface(),
    robot_controllers_interface::get_safe_topic_name(name),
    std::bind(&PointHeadController::handle_goal, this, _1, _2),
    std::bind(&PointHeadController::handle_cancel, this, _1),
    std::bind(&PointHeadController::handle_accepted, this, _1)
  );

  return 0;
}

bool PointHeadController::start()
{
  if (!server_)
  {
    // Can't log (not intialized)
    return false;
  }

  if (!active_goal_)
  {
    RCLCPP_ERROR(rclcpp::get_logger(getName()),
                 "Unable to start, action server has no goal.");
    return false;
  }

  return true;
}

bool PointHeadController::stop(bool force)
{
  if (!server_)
    return true;

  if (active_goal_)
  {
    if (force)
    {
      // Shut down the action
      auto result = std::make_shared<PointHeadAction::Result>();
      active_goal_->abort(result);
      active_goal_.reset();
      return true;
    }
    // Do not abort unless forced
    return false;
  }

  // Just holding position, go ahead and stop us
  return true;
}

bool PointHeadController::reset()
{
  stop(true);  // force stop ourselves
  return (manager_->requestStop(getName()) == 0);
}

void PointHeadController::update(const rclcpp::Time& now, const rclcpp::Duration& dt)
{
  (void) dt;

  if (!server_)
    return;

  // We have a trajectory to execute?
  if (active_goal_ && sampler_)
  {
    std::lock_guard<std::mutex> lock(sampler_mutex_);

    // Interpolate trajectory
    TrajectoryPoint p = sampler_->sample(to_sec(now));
    last_sample_ = p;

    // Are we done?
    if (to_sec(now) > sampler_->end_time())
    {
      // Stop this controller if desired (and not preempted)
      if (stop_with_action_)
        manager_->requestStop(getName());

      auto result = std::make_shared<PointHeadAction::Result>();
      active_goal_->succeed(result);
      active_goal_.reset();
      RCLCPP_DEBUG(rclcpp::get_logger(getName()),
                   "PointHead goal succeeded");
    }

    // Send trajectory to joints
    if (p.q.size() == 2)
    {
      head_pan_->setPosition(p.q[0], p.qd[0], 0.0);
      head_tilt_->setPosition(p.q[1], p.qd[1], 0.0);
    }
  }
  else if (last_sample_.q.size() == 2)
  {
    // Hold Position
    head_pan_->setPosition(last_sample_.q[0], 0.0, 0.0);
    head_tilt_->setPosition(last_sample_.q[1], 0.0, 0.0);
  }
}

rclcpp_action::GoalResponse PointHeadController::handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const PointHeadAction::Goal> goal_handle)
{
  (void) uuid;
  (void) goal_handle;

  if (!server_)
  {
    RCLCPP_ERROR(rclcpp::get_logger(getName()),
                 "Unable to start, action server not initialized.");
    return rclcpp_action::GoalResponse::REJECT;
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse PointHeadController::handle_cancel(
    const std::shared_ptr<PointHeadGoal> goal_handle)
{
  // Always accept
  if (active_goal_ && active_goal_->get_goal_id() == goal_handle->get_goal_id())
  {
    RCLCPP_INFO(rclcpp::get_logger(getName()),
                "Goal cancelled.");
    auto result = std::make_shared<PointHeadAction::Result>();
    active_goal_->canceled(result);
    active_goal_.reset();
  }

  return rclcpp_action::CancelResponse::ACCEPT;
}

void PointHeadController::handle_accepted(const std::shared_ptr<PointHeadGoal> goal_handle)
{
  auto result = std::make_shared<PointHeadAction::Result>();

  bool preempted = false;
  if (active_goal_)
  {
    // TODO(anyone): if rclcpp_action ever supports preempted, note it here
    //               https://github.com/ros2/rclcpp/issues/1104
    active_goal_->abort(result);
    active_goal_.reset();
    RCLCPP_DEBUG(rclcpp::get_logger(getName()),
                 "Goal preempted.");
    preempted = true;
  }

  const auto goal = goal_handle->get_goal();

  float head_pan_goal_ = 0.0;
  float head_tilt_goal_ = 0.0;
  try
  {
    geometry_msgs::msg::PointStamped target_in_pan, target_in_tilt;
    tf_buffer_->transform(goal->target, target_in_pan, root_link_);
    tf_buffer_->transform(goal->target, target_in_tilt, "head_pan_link");
    head_pan_goal_ = atan2(target_in_pan.point.y, target_in_pan.point.x);
    head_tilt_goal_ = -atan2(target_in_tilt.point.z,
      sqrt(pow(target_in_tilt.point.x, 2) + pow(target_in_tilt.point.y, 2)));
  }
  catch (const tf2::TransformException& ex)
  {
    goal_handle->abort(result);
    RCLCPP_ERROR(rclcpp::get_logger(getName()),
                 "Could not transform goal using TF2.");
    return;
  }

  // Turn goal into a trajectory
  Trajectory t;
  t.points.resize(2);
  if (preempted)
  {
    // Starting point is last sample
    t.points[0] = last_sample_;
  }
  else
  {
    // Starting point is current position, assume not moving
    t.points[0].q.push_back(head_pan_->getPosition());
    t.points[0].q.push_back(head_tilt_->getPosition());
    t.points[0].qd.push_back(0.0);
    t.points[0].qd.push_back(0.0);
    t.points[0].qdd.push_back(0.0);
    t.points[0].qdd.push_back(0.0);
    t.points[0].time = to_sec(node_->now());
  }

  // Ending point is goal position, not moving
  t.points[1].q.push_back(head_pan_goal_);
  t.points[1].q.push_back(head_tilt_goal_);
  t.points[1].qd.push_back(0.0);
  t.points[1].qd.push_back(0.0);
  t.points[1].qdd.push_back(0.0);
  t.points[1].qdd.push_back(0.0);

  // Determine how long to take to execute this trajectory
  double max_pan_vel = head_pan_->getVelocityMax();
  double max_tilt_vel = head_tilt_->getVelocityMax();
  if (goal->max_velocity > 0.0)
  {
    max_pan_vel = fmin(goal->max_velocity, head_pan_->getVelocityMax());
    max_tilt_vel = fmin(goal->max_velocity, head_tilt_->getVelocityMax());
  }
  double pan_transit = fabs((t.points[1].q[0] - t.points[0].q[0]) / max_pan_vel);
  double tilt_transit = fabs((t.points[1].q[1] - t.points[0].q[1]) / max_tilt_vel);
  t.points[1].time = t.points[0].time + fmax(fmax(pan_transit, tilt_transit),
                                             msg_to_sec(goal->min_duration));

  {
    std::lock_guard<std::mutex> lock(sampler_mutex_);
    sampler_.reset(new SplineTrajectorySampler(t));
    active_goal_ = goal_handle;
  }

  if (manager_->requestStart(getName()) != 0)
  {
    active_goal_->abort(result);
    active_goal_.reset();
    RCLCPP_ERROR(rclcpp::get_logger(getName()),
                 "Cannot point head, unable to start controller.");
    return;
  }

  RCLCPP_DEBUG(rclcpp::get_logger(getName()), "PointHead goal started.");
}

std::vector<std::string> PointHeadController::getCommandedNames()
{
  std::vector<std::string> names;
  names.push_back(head_pan_->getName());
  names.push_back(head_tilt_->getName());
  return names;
}

std::vector<std::string> PointHeadController::getClaimedNames()
{
  // Claimed == commanded.
  return getCommandedNames();
}

}  // namespace robot_controllers
