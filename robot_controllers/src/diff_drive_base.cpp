/*********************************************************************
 *  Software License Agreement (BSD License)
 *
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

// Author: Michael Ferguson

#include <angles/angles.h>
#include <pluginlib/class_list_macros.h>
#include <robot_controllers/diff_drive_base.h>

PLUGINLIB_EXPORT_CLASS(robot_controllers::DiffDriveBaseController, robot_controllers::Controller)

namespace robot_controllers
{

DiffDriveBaseController::DiffDriveBaseController() :
    initialized_(false)
{
  theta_ = 0.0;

  odom_.pose.pose.orientation.z = 0.0;
  odom_.pose.pose.orientation.w = 1.0;

  last_sent_x_ = desired_x_ = 0.0;
  last_sent_r_ = desired_r_ = 0.0;

  left_last_timestamp_ = right_last_timestamp_ = 0.0;
  last_command_ = last_update_ = ros::Time(0.0);
}

int DiffDriveBaseController::init(ros::NodeHandle& nh, ControllerManager* manager)
{
  // We absolutely need access to the controller manager
  if (!manager)
  {
    ROS_ERROR_NAMED("BaseController", "No controller manager available.");
    initialized_ = false;
    return -1;
  }

  Controller::init(nh, manager);
  manager_ = manager;

  // Initialize joints
  std::string l_name, r_name;
  nh.param<std::string>("l_wheel_joint", l_name, "l_wheel_joint");
  nh.param<std::string>("r_wheel_joint", r_name, "r_wheel_joint");
  left_ = manager_->getJointHandle(l_name);
  right_ = manager_->getJointHandle(r_name);
  if (left_ == NULL || right_ == NULL)
  {
    ROS_ERROR_NAMED("BaseController", "Cannot get wheel joints.");
    initialized_ = false;
    return -1;
  }
  left_last_position_ = left_->getPosition();
  right_last_position_ = right_->getPosition();
  last_update_ = ros::Time::now();

  // Get base parameters
  nh.param<double>("track_width", track_width_, 0.37476);
  nh.param<double>("radians_per_meter", radians_per_meter_, 16.5289);

  // If using an external correction (such as robot_pose_ekf or graft)
  // we should not publish the TF frame from base->odom
  nh.param<bool>("publish_tf", publish_tf_, true);

  // The pose in the odometry message is specified in terms of the odometry frame
  nh.param<std::string>("odometry_frame", odom_.header.frame_id, "odom");

  // The twist in the odometry message is specified in the coordinate frame of the base
  nh.param<std::string>("base_frame", odom_.child_frame_id, "base_link");

  // Get various thresholds below which we supress noise
  nh.param<double>("wheel_rotating_threshold", wheel_rotating_threshold_, 0.001);
  nh.param<double>("rotating_threshold", rotating_threshold_, 0.05);
  nh.param<double>("moving_threshold", moving_threshold_, 0.05);

  double t;
  nh.param<double>("timeout", t, 0.25);
  timeout_ = ros::Duration(t);

  // Get limits of base controller
  nh.param<double>("max_velocity_x", max_velocity_x_, 1.0);
  nh.param<double>("max_velocity_r", max_velocity_r_, 4.5);
  x_accel_profile_.init(nh, "max_acceleration_x");
  r_accel_profile_.init(nh, "max_acceleration_r");

  // Subscribe to base commands
  cmd_sub_ = nh.subscribe<geometry_msgs::Twist>("command", 1,
                boost::bind(&DiffDriveBaseController::command, this, _1));

  // Publish odometry & tf
  ros::NodeHandle n;
  odom_pub_ = n.advertise<nav_msgs::Odometry>("odom", 10);
  if (publish_tf_)
    broadcaster_.reset(new tf::TransformBroadcaster());

  initialized_ = true;

  // Should we autostart?
  bool autostart;
  nh.param("autostart", autostart, false);
  if (autostart)
    manager->requestStart(getName());

  return 0;
}

void DiffDriveBaseController::command(const geometry_msgs::TwistConstPtr& msg)
{
  if (!initialized_)
  {
    ROS_ERROR_NAMED("BaseController", "Unable to accept command, not initialized.");
    return;
  }

  last_command_ = ros::Time::now();
  desired_x_ = msg->linear.x;
  desired_r_ = msg->angular.z;
  manager_->requestStart(getName());
}

bool DiffDriveBaseController::start()
{
  if (!initialized_)
  {
    ROS_ERROR_NAMED("BaseController", "Unable to start, not initialized.");
    return false;
  }

  return true;
}

bool DiffDriveBaseController::stop(bool force)
{
  // If we have timed out, certainly stop
  if (last_update_ - last_command_ >= timeout_)
    return true;

  // If not moving, stop
  if (last_sent_x_ == 0.0 && last_sent_r_ == 0.0)
    return true;

  // If forced, stop
  if (force)
    return true;

  return false;
}

void DiffDriveBaseController::update(const ros::Time& now, const ros::Duration& dt)
{
  if (!initialized_)
    return;  // should never really hit this

  // See if we have timed out and need to stop
  if (now - last_command_ >= timeout_)
  {
    ROS_DEBUG_THROTTLE_NAMED(5, "BaseController", "Command timed out.");
    desired_x_ = desired_r_ = 0.0;
  }

  // Do velocity acceleration/limiting
  last_sent_x_ = x_accel_profile_.interpolate(desired_x_, last_sent_x_, (now - last_update_).toSec());
  last_sent_r_ = r_accel_profile_.interpolate(desired_r_, last_sent_r_, (now - last_update_).toSec());

  double dx = 0.0;
  double dr = 0.0;

  double left_pos = left_->getPosition();
  double right_pos = right_->getPosition();
  double left_dx = angles::shortest_angular_distance(left_last_position_, left_pos)/radians_per_meter_;
  double right_dx = angles::shortest_angular_distance(right_last_position_, right_pos)/radians_per_meter_;
  double left_vel = static_cast<double>(left_->getVelocity())/radians_per_meter_;
  double right_vel = static_cast<double>(right_->getVelocity())/radians_per_meter_;

  // Threshold the odometry to avoid noise (especially in simulation)
  if (fabs(left_dx) > wheel_rotating_threshold_ ||
      fabs(right_dx) > wheel_rotating_threshold_ ||
      last_sent_x_ != 0.0 ||
      last_sent_r_ != 0.0)
  {
    // Above threshold, update last position
    left_last_position_ = left_pos;
    right_last_position_ = right_pos;
  }
  else
  {
    // Below threshold, zero delta/velocities
    left_dx = right_dx = 0.0;
    left_vel = right_vel = 0.0;
  }

  // Calculate forward and angular differences
  double d = (left_dx+right_dx)/2.0;
  double th = (right_dx-left_dx)/track_width_;

  // Calculate forward and angular velocities
  dx = (left_vel + right_vel)/2.0;
  dr = (right_vel - left_vel)/track_width_;

  // Update store odometry
  theta_ += th/2.0;
  odom_.pose.pose.position.x += d*cos(theta_);
  odom_.pose.pose.position.y += d*sin(theta_);
  theta_ += th/2.0;

  // Actually set command
  if (fabs(dx) > moving_threshold_ ||
      fabs(dr) > rotating_threshold_ ||
      last_sent_x_ != 0.0 ||
      last_sent_r_ != 0.0)
  {
    setCommand(last_sent_x_ - (last_sent_r_/2.0 * track_width_),
               last_sent_x_ + (last_sent_r_/2.0 * track_width_));
  }

  // Update odometry information
  odom_.pose.pose.orientation.z = sin(theta_/2.0);
  odom_.pose.pose.orientation.w = cos(theta_/2.0);
  odom_.twist.twist.linear.x = dx;
  odom_.twist.twist.angular.z = dr;

  last_update_ = now;
}

std::vector<std::string> DiffDriveBaseController::getCommandedNames()
{
  std::vector<std::string> names;
  if (left_)
    names.push_back(left_->getName());
  if (right_)
    names.push_back(right_->getName());
  return names;
}

std::vector<std::string> DiffDriveBaseController::getClaimedNames()
{
  // Claimed == Commanded
  return getCommandedNames();
}

bool DiffDriveBaseController::publish(ros::Time time)
{
  // Publish or perish
  odom_.header.stamp = time;
  odom_pub_.publish(odom_);

  if (publish_tf_)
  {
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(odom_.pose.pose.position.x, odom_.pose.pose.position.y, 0.0));
    transform.setRotation(tf::Quaternion(odom_.pose.pose.orientation.x,
                                         odom_.pose.pose.orientation.y,
                                         odom_.pose.pose.orientation.z,
                                         odom_.pose.pose.orientation.w) );
    /*
     * REP105 (http://ros.org/reps/rep-0105.html)
     *   says: map -> odom -> base_link
     */
    broadcaster_->sendTransform(tf::StampedTransform(transform, time, odom_.header.frame_id, odom_.child_frame_id));
  }

  return true;
}

void DiffDriveBaseController::setCommand(float left, float right)
{
  // Convert meters/sec into radians/sec
  left_->setVelocity(left * radians_per_meter_, 0.0);
  right_->setVelocity(right * radians_per_meter_, 0.0);
}

DiffDriveBaseController::PiecewiseAccelProfile::PiecewiseAccelProfile() :
  limit_interval(1.0)  // avoid div/0
{
}

int DiffDriveBaseController::PiecewiseAccelProfile::init(
  ros::NodeHandle& nh,
  const std::string& name)
{
  XmlRpc::XmlRpcValue params;
  if (nh.getParam(name, params))
  {
    if (params.getType() ==  XmlRpc::XmlRpcValue::TypeDouble)
    {
      ROS_WARN("Interpreting acceleration profile as single acceleration");
      accel_limits.push_back(static_cast<double>(params));
      decel_limits.push_back(static_cast<double>(params));
    }
    else
    {
      nh.getParam(name+"/interval", limit_interval);
      ros::NodeHandle n(nh, name);
      if (n.getParam("accel", params))
      {
        for (int i = 0; i < params.size(); i++)
        {
          accel_limits.push_back(static_cast<double>(params[i]));
        }
      }
      if (n.getParam("decel", params))
      {
        for (int i = 0; i < params.size(); i++)
        {
          decel_limits.push_back(static_cast<double>(params[i]));
        }
      }
    }
  }
  else
  {
    // Default to 1.0
    ROS_WARN("No acceleration profile provided");
    accel_limits.push_back(1.0);
    decel_limits.push_back(1.0);
  }

  ROS_INFO(" ** BASE CONTROLLER ** ");
  ROS_INFO_STREAM(limit_interval);
  ROS_INFO("Accel Limits");
  for (size_t i = 0; i < accel_limits.size(); i++)
    ROS_INFO_STREAM("  " << accel_limits[i]);
  ROS_INFO("Decel Limits");
  for (size_t i = 0; i < decel_limits.size(); i++)
    ROS_INFO_STREAM("  " << decel_limits[i]);
}

double DiffDriveBaseController::PiecewiseAccelProfile::interpolate(
  double desired,
  double present,
  double timestep)
{
  // Determine index to use within accel/decel curve
  int v_index = fabs(present) / limit_interval;
  v_index = std::min(v_index, static_cast<int>(accel_limits.size())-1);

  double a = 0.0;
  double v = 0.0;
  if (desired > present)
  {
    if (present >= 0.0)
    {
      a = accel_limits[v_index];
    }
    else
    {
      a = decel_limits[v_index];
    }
    v = present + a * timestep;
    if (v > desired)
    {
      return desired;
    }
    else
    {
      return v;
    }
  }
  else
  {
    if (desired < 0.0)
    {
      a = accel_limits[v_index];
    }
    else
    {
      a = decel_limits[v_index];
    }
    v = present - a * timestep;
    if (v < desired)
    {
      return desired;
    }
    else
    {
      return v;
    }
  }
}

}  // namespace robot_controllers
