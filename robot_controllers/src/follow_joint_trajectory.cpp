/*********************************************************************
 *  Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, Michael Ferguson
 *  Copyright (c) 2014-2015, Fetch Robotics Inc.
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

/* Author: Michael Ferguson */

#include <algorithm>
#include <pluginlib/class_list_macros.hpp>
#include <robot_controllers/follow_joint_trajectory.h>

using angles::shortest_angular_distance;

PLUGINLIB_EXPORT_CLASS(robot_controllers::FollowJointTrajectoryController, robot_controllers_interface::Controller)

namespace robot_controllers
{

FollowJointTrajectoryController::FollowJointTrajectoryController() :
    initialized_(false)
{
}

int FollowJointTrajectoryController::init(
    const std::string& name,
    rclcpp::Node::SharedPtr node,
    robot_controllers_interface::ControllerManagerPtr manager)
{
  using namespace std::placeholders;

  // We absolutely need access to the controller manager
  if (!manager)
  {
    initialized_ = false;
    return -1;
  }

  Controller::init(name, node, manager);
  node_ = node;
  manager_ = manager;

  // No initial sampler
  std::scoped_lock lock(sampler_mutex_);
  sampler_.reset();
  preempted_ = false;

  // Get Joint Names
  joint_names_ = node->declare_parameter<std::vector<std::string>>(name + ".joints", std::vector<std::string>());
  if (joint_names_.empty())
  {
    RCLCPP_ERROR(node->get_logger(), "No joints given for %s", name.c_str());
    return -1;
  }

  // Get parameters
  stop_with_action_ = node->declare_parameter<bool>(name + ".stop_with_action", false);
  stop_on_path_violation_ = node->declare_parameter<bool>(name + ".stop_on_path_violation", false);

  // Get Joint Handles, setup feedback
  joints_.clear();
  feedback_ = std::make_shared<FollowJointTrajectoryAction::Feedback>();
  for (size_t i = 0; i < joint_names_.size(); ++i)
  {
    robot_controllers_interface::JointHandlePtr j = manager_->getJointHandle(joint_names_[i]);
    feedback_->joint_names.push_back(j->getName());
    joints_.push_back(j);
    continuous_.push_back(j->isContinuous());
  }

  // Update feedback
  feedback_->desired.positions.resize(joints_.size());
  feedback_->desired.velocities.resize(joints_.size());
  feedback_->desired.accelerations.resize(joints_.size());
  feedback_->actual.positions.resize(joints_.size());
  feedback_->actual.velocities.resize(joints_.size());
  feedback_->actual.effort.resize(joints_.size());
  feedback_->error.positions.resize(joints_.size());
  feedback_->error.velocities.resize(joints_.size());

  // Update tolerances
  path_tolerance_.q.resize(joints_.size());
  path_tolerance_.qd.resize(joints_.size());
  path_tolerance_.qdd.resize(joints_.size());
  goal_tolerance_.q.resize(joints_.size());
  goal_tolerance_.qd.resize(joints_.size());
  goal_tolerance_.qdd.resize(joints_.size());

  // Parameters use ".", topics use "/"
  std::string action_name = name;
  std::replace(action_name.begin(), action_name.end(), '.', '/');

  // Setup actionlib server
  active_goal_.reset();
  server_ = rclcpp_action::create_server<FollowJointTrajectoryAction>(
    node_->get_node_base_interface(),
    node_->get_node_clock_interface(),
    node_->get_node_logging_interface(),
    node_->get_node_waitables_interface(),
    action_name,
    std::bind(&FollowJointTrajectoryController::handle_goal, this, _1, _2),
    std::bind(&FollowJointTrajectoryController::handle_cancel, this, _1),
    std::bind(&FollowJointTrajectoryController::handle_accepted, this, _1)
  );

  initialized_ = true;
  return 0;
}

bool FollowJointTrajectoryController::start()
{
  if (!initialized_)
  {
    // Can't log (not intialized)
    return false;
  }

  /*if (!server_->isActive())
  {
    RCLCPP_ERROR(node_->get_logger(), "Unable to start, action server is not active.");
    return false;
  }*/

  return true;
}

bool FollowJointTrajectoryController::stop(bool force)
{
  if (!initialized_)
    return true;

  if (active_goal_)
  {

  }

  if (active_goal_)
  {
    if (force)
    {
      // Shut down the action
      auto result = std::make_shared<FollowJointTrajectoryAction::Result>();
      result->error_code = result->GOAL_TOLERANCE_VIOLATED;
      result->error_string = "Controller manager forced preemption.";
      active_goal_->abort(result);
      return true;
    }
    // Do not abort unless forced
    return false;
  }

  // Just holding position, go ahead and stop us
  return true;
}

bool FollowJointTrajectoryController::reset()
{
  stop(true);  // force stop ourselves
  return (manager_->requestStop(getName()) == 0);
}

void FollowJointTrajectoryController::update(const rclcpp::Time& now, const rclcpp::Duration& dt)
{
  if (!initialized_)
    return;

  // Is trajectory active?
  if (active_goal_ && sampler_)
  {
    std::scoped_lock lock(sampler_mutex_);

    // Interpolate trajectory
    TrajectoryPoint p = sampler_->sample((double) now.nanoseconds() / 1e9);
    unwindTrajectoryPoint(continuous_, p);
    last_sample_ = p;

    // Update joints
    if (p.q.size() == joints_.size())
    {
      // Position is good
      for (size_t i = 0; i < joints_.size(); ++i)
      {
        feedback_->desired.positions[i] = p.q[i];
      }

      if (p.qd.size() == joints_.size())
      {
        // Velocity is good
        for (size_t i = 0; i < joints_.size(); ++i)
        {
          feedback_->desired.velocities[i] = p.qd[i];
        }

        if (p.qdd.size() == joints_.size())
        {
          // Acceleration is good
          for (size_t i = 0; i < joints_.size(); ++i)
          {
            feedback_->desired.accelerations[i] = p.qdd[i];
          }
        }
      }

      // Fill in actual
      for (size_t j = 0; j < joints_.size(); ++j)
      {
        feedback_->actual.positions[j] = joints_[j]->getPosition();
        feedback_->actual.velocities[j] = joints_[j]->getVelocity();
        feedback_->actual.effort[j] = joints_[j]->getEffort();
      }

      // Fill in error
      for (size_t j = 0; j < joints_.size(); ++j)
      {
        feedback_->error.positions[j] = shortest_angular_distance(feedback_->desired.positions[j],
                                                                 feedback_->actual.positions[j]);
        feedback_->error.velocities[j] = feedback_->actual.velocities[j] -
                                        feedback_->desired.velocities[j];
      }

      // Check that we are within path tolerance
      if (has_path_tolerance_)
      {
        for (size_t j = 0; j < joints_.size(); ++j)
        {
          if ((path_tolerance_.q[j] > 0) &&
              (fabs(feedback_->error.positions[j]) > path_tolerance_.q[j]))
          {
            auto result = std::make_shared<FollowJointTrajectoryAction::Result>();
            result->error_code = result->PATH_TOLERANCE_VIOLATED;
            active_goal_->abort(result);
            RCLCPP_ERROR(node_->get_logger(), "Trajectory path tolerances violated (position).");
            if (stop_on_path_violation_)
            {
              manager_->requestStop(getName());
            }
            break;
          }

          if ((path_tolerance_.qd[j] > 0) &&
              (fabs(feedback_->error.velocities[j]) > path_tolerance_.qd[j]))
          {
            auto result = std::make_shared<FollowJointTrajectoryAction::Result>();
            result->error_code = result->PATH_TOLERANCE_VIOLATED;
            active_goal_->abort(result);
            RCLCPP_ERROR(node_->get_logger(), "Trajectory path tolerances violated (velocity).");
            if (stop_on_path_violation_)
            {
              manager_->requestStop(getName());
            }
            break;
          }
        }
      }

      // Check that we are within goal tolerance
      double now_sec = (double) now.nanoseconds() / 1e9;
      if (now_sec >= sampler_->end_time())
      {
        bool inside_tolerances = true;
        for (size_t j = 0; j < joints_.size(); ++j)
        {
          if ((goal_tolerance_.q[j] > 0) &&
              (fabs(feedback_->error.positions[j]) > goal_tolerance_.q[j]))
          {
            inside_tolerances = false;
          }
        }

        if (inside_tolerances)
        {
          auto result = std::make_shared<FollowJointTrajectoryAction::Result>();
          active_goal_->succeed(result);
          active_goal_.reset();
          // Stop this controller if desired (and not preempted)
          if (stop_with_action_)
            manager_->requestStop(getName());
          RCLCPP_DEBUG(node_->get_logger(), "Trajectory succeeded");
        }
        else if (now_sec > (sampler_->end_time() + goal_time_tolerance_ + 0.6))  // 0.6s matches PR2
        {
          auto result = std::make_shared<FollowJointTrajectoryAction::Result>();
          result->error_code = result->GOAL_TOLERANCE_VIOLATED;
          active_goal_->abort(result);
          RCLCPP_ERROR(node_->get_logger(), "Trajectory not executed within time limits");
        }
      }

      // Update joints
      for (size_t j = 0; j < joints_.size(); ++j)
      {
        joints_[j]->setPosition(feedback_->desired.positions[j],
                                feedback_->desired.velocities[j],
                                0.0);
      }
    }
  }
  else if (last_sample_.q.size() == joints_.size())
  {
    // Hold Position Unless Path Tolerance Violated
    if (has_path_tolerance_ && stop_on_path_violation_)
    {
      for (size_t j = 0; j < joints_.size(); ++j)
      {
        if ((path_tolerance_.q[j] > 0) &&
            (fabs(joints_[j]->getPosition() - last_sample_.q[j]) > path_tolerance_.q[j]))
        {
          manager_->requestStop(getName());
          break;
        }
      }
    }

    for (size_t j = 0; j < joints_.size(); ++j)
    {
      joints_[j]->setPosition(last_sample_.q[j],
                              0.0,
                              0.0);
    }
  }
}

rclcpp_action::GoalResponse FollowJointTrajectoryController::handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const FollowJointTrajectoryAction::Goal> goal_handle)
{
  if (!initialized_)
  {
    // Can't even log here - we aren't initialized
    return rclcpp_action::GoalResponse::REJECT;
  }

  if (goal_handle->trajectory.joint_names.size() != joints_.size())
  {
    RCLCPP_ERROR(node_->get_logger(), "Trajectory goal size does not match controlled joints size.");
    return rclcpp_action::GoalResponse::REJECT;
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse FollowJointTrajectoryController::handle_cancel(
    const std::shared_ptr<FollowJointTrajectoryGoal> goal_handle)
{
  // Always accept
  if (active_goal_ && active_goal_->get_goal_id() == goal_handle->get_goal_id())
  {
    active_goal_.reset();
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  return rclcpp_action::CancelResponse::ACCEPT;
}

void FollowJointTrajectoryController::handle_accepted(const std::shared_ptr<FollowJointTrajectoryGoal> goal_handle)
{
  auto result = std::make_shared<FollowJointTrajectoryAction::Result>();
  
  if (active_goal_)
  {
    // TODO: if rclcpp_action ever supports preempted, note it here
    //       https://github.com/ros2/rclcpp/issues/1104
    result->error_code = -6;
    result->error_string = "preempted";
    active_goal_->abort(result);
    active_goal_.reset();
  }

  const auto goal = goal_handle->get_goal();
  
  if (goal->trajectory.points.empty())
  {
    // Stop
    manager_->requestStop(getName());
    result->error_code = result->SUCCESSFUL;
    result->error_string = "Controller stopped";
    goal_handle->succeed(result);
    return;
  }

  Trajectory new_trajectory;
  Trajectory executable_trajectory;
  goal_time = goal->trajectory.header.stamp;

  // Make a trajectory from our message
  if (!trajectoryFromMsg(goal->trajectory, joint_names_, node_->now(), &new_trajectory))
  {
    active_goal_.reset();
    result->error_code = result->INVALID_JOINTS;
    goal_handle->abort(result);
    RCLCPP_ERROR(node_->get_logger(), "Trajectory goal does not match controlled joints");
    return;
  }

  // If we are preempting a trajectory, need to splice on things together
  if (active_goal_)
  {
    // If the sampler had a large trajectory, we may just be cutting into it
    if (sampler_ && (sampler_->getTrajectory().size() > 2))
    {
      if (!spliceTrajectories(sampler_->getTrajectory(),
                              new_trajectory,
                              (double) node_->now().nanoseconds() / 1e9,
                              &executable_trajectory))
      {
        active_goal_.reset();
        result->error_code = result->INVALID_JOINTS;
        goal_handle->abort(result);
        RCLCPP_ERROR(node_->get_logger(), "Unable to splice trajectory");
        return;
      }
    }
    else
    {
      // Previous trajectory was only 2 points, use last_sample + new trajectory
      Trajectory t;
      t.points.push_back(last_sample_);
      if (!spliceTrajectories(t,
                              new_trajectory,
                              0.0, /* take all points */
                              &executable_trajectory))
      {
        active_goal_.reset();
        result->error_code = result->INVALID_JOINTS;
        goal_handle->abort(result);
        RCLCPP_ERROR(node_->get_logger(), "Unable to splice trajectory");
        return;
      }
    }
  }
  else
  {
    if (new_trajectory.size() > 1)
    {
      // use the generated trajectory
      executable_trajectory = new_trajectory;

      // if this hasn't started yet or if the header stamp is in the future,
      // need to insert current position
      if (time_utils::fromMsg(goal->trajectory.points[0].time_from_start) > 0 ||
          executable_trajectory.points[0].time > (double) node_->now().nanoseconds() / 1e9)
      {
        executable_trajectory.points.insert(
          executable_trajectory.points.begin(),
          getPointFromCurrent(new_trajectory.points[0].qd.size() > 0,
                              new_trajectory.points[0].qdd.size() > 0,
                              true));
      }
    }
    else
    {
      // A single point, with nothing in the queue!
      executable_trajectory.points.push_back(
          getPointFromCurrent(new_trajectory.points[0].qd.size() > 0,
                              new_trajectory.points[0].qdd.size() > 0,
                              true));
      executable_trajectory.points.push_back(new_trajectory.points[0]);
    }
  }

  // Windup executable trajectory so spline is smooth
  windupTrajectory(continuous_, executable_trajectory);

  // Create trajectory sampler
  {
    std::scoped_lock lock(sampler_mutex_);
    sampler_.reset(new SplineTrajectorySampler(executable_trajectory));

    // Convert the path tolerances into a more usable form
    if (goal->path_tolerance.size() == joints_.size())
    {
      has_path_tolerance_ = true;
      for (size_t j = 0; j < joints_.size(); ++j)
      {
        // Find corresponding indices between path_tolerance and joints
        int index = -1;
        for (size_t i = 0; i < goal->path_tolerance.size(); ++i)
        {
          if (joints_[j]->getName() == goal->path_tolerance[i].name)
          {
            index = i;
            break;
          }
        }
        if (index == -1)
        {
          // Every joint must have a tolerance and this one does not
          active_goal_.reset();
          result->error_code = result->INVALID_JOINTS;
          goal_handle->abort(result);
          RCLCPP_ERROR(node_->get_logger(), "Unable to convert path tolerances");
          return;
        }
        path_tolerance_.q[j] = goal->path_tolerance[index].position;
        path_tolerance_.qd[j] = goal->path_tolerance[index].velocity;
        path_tolerance_.qdd[j] = goal->path_tolerance[index].acceleration;
      }
    }
    else
    {
      has_path_tolerance_ = false;
    }

    // Convert the goal tolerances into a more usable form
    if (goal->goal_tolerance.size() == joints_.size())
    {
      for (size_t j = 0; j < joints_.size(); ++j)
      {
        // Find corresponding indices between goal_tolerance and joints
        int index = -1;
        for (size_t i = 0; i < goal->goal_tolerance.size(); ++i)
        {
          if (joints_[j]->getName() == goal->goal_tolerance[i].name)
          {
            index = i;
            break;
          }
        }
        if (index == -1)
        {
          // Every joint must have a tolerance and this one does not
          active_goal_.reset();
          result->error_code = result->INVALID_JOINTS;
          goal_handle->abort(result);
          RCLCPP_ERROR(node_->get_logger(), "Unable to convert goal tolerances");
          return;
        }
        goal_tolerance_.q[j] = goal->goal_tolerance[index].position;
        goal_tolerance_.qd[j] = goal->goal_tolerance[index].velocity;
        goal_tolerance_.qdd[j] = goal->goal_tolerance[index].acceleration;
      }
    }
    else
    {
      // Set defaults
      for (size_t j = 0; j < joints_.size(); ++j)
      {
        goal_tolerance_.q[j] = 0.02;  // tolerance is same as PR2
        goal_tolerance_.qd[j] = 0.02;
        goal_tolerance_.qdd[j] = 0.02;
      }
    }
    goal_time_tolerance_ = time_utils::fromMsg(goal->goal_time_tolerance);

    active_goal_ = goal_handle;
  }

  RCLCPP_DEBUG(node_->get_logger(), "Executing new trajectory");

  if (manager_->requestStart(getName()) != 0)
  {
    active_goal_.reset();
    RCLCPP_ERROR(node_->get_logger(), "Cannot execute trajectory, unable to start controller.");
    result->error_code = result->GOAL_TOLERANCE_VIOLATED;
    goal_handle->abort(result);
    return;
  }
}

/*
    // Publish feedback
    feedback_->header.stamp = node_->now();
    feedback_->desired.time_from_start = feedback_->header.stamp - goal_time;
    feedback_->actual.time_from_start = feedback_->header.stamp - goal_time;
    feedback_->error.time_from_start = feedback_->header.stamp - goal_time;
    server_->publishFeedback(feedback_);
    rclcpp::Duration(1/50.0).sleep();
 */

TrajectoryPoint FollowJointTrajectoryController::getPointFromCurrent(
  bool incl_vel, bool incl_acc, bool zero_vel)
{
  TrajectoryPoint point;

  point.q.resize(joints_.size());
  for (size_t j = 0; j < joints_.size(); ++j)
    point.q[j] = joints_[j]->getPosition();

  if (incl_vel && zero_vel)
  {
    point.qd.resize(joints_.size());
    for (size_t j = 0; j < joints_.size(); ++j)
      point.qd[j] = 0.0;
  }
  else if (incl_vel)
  {
    point.qd.resize(joints_.size());
    for (size_t j = 0; j < joints_.size(); ++j)
      point.qd[j] = joints_[j]->getVelocity();
  }

  if (incl_acc)
  {
    point.qdd.resize(joints_.size());
    // Currently no good measure of acceleration, assume none
    for (size_t j = 0; j < joints_.size(); ++j)
      point.qdd[j] = 0.0;
  }

  point.time = (double) node_->now().nanoseconds() / 1e9;

  return point;
}

std::vector<std::string> FollowJointTrajectoryController::getCommandedNames()
{
  return joint_names_;
}

std::vector<std::string> FollowJointTrajectoryController::getClaimedNames()
{
  return joint_names_;
}

}  // namespace robot_controllers
