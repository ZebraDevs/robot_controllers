/*********************************************************************
 *  Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014-2016, Fetch Robotics Inc.
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
    initialized_(false),
    safety_scaling_(1.0)
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

  // Get diff drive parameters
  robot_controllers_msgs::DiffDriveLimiterParams params = DiffDriveLimiter::getDefaultParams();
  nh.getParam("track_width", params.track_width);
  nh.getParam("max_velocity_x", params.max_linear_velocity);
  nh.getParam("max_velocity_r", params.max_angular_velocity);
  nh.getParam("max_acceleration_x", params.max_linear_acceleration);
  nh.getParam("max_acceleration_r", params.max_angular_acceleration);

  bool angular_velocity_limits_linear_velocity = params.angular_velocity_limits_linear_velocity;
  nh.getParam("angular_velocity_limits_linear_velocity", angular_velocity_limits_linear_velocity);
  params.angular_velocity_limits_linear_velocity = angular_velocity_limits_linear_velocity;

  bool scale_to_wheel_velocity_limits = params.scale_to_wheel_velocity_limits;
  nh.getParam("scale_to_wheel_velocity_limits", scale_to_wheel_velocity_limits);
  params.scale_to_wheel_velocity_limits = scale_to_wheel_velocity_limits;

  // Max wheel velocity parameter is imposted by joint, but limiter needs to
  // know about it to properly maintain path curvature when turning while driving fast
  if (left_->getVelocityMax() != right_->getVelocityMax())
  {
    ROS_WARN_NAMED("BaseController", "Right and left wheel velocities limits do not match %f %f",
             left_->getVelocityMax(), right_->getVelocityMax());
  }
  // Convert wheel velocity limit in (rad/s) of motor, to linear velocity of wheel
  double joint_max_wheel_velocity =
    std::min(left_->getVelocityMax(), right_->getVelocityMax()) / radians_per_meter_;

  // Allow rosparam to limit wheel velocity to lower value than joint
  double max_wheel_velocity = joint_max_wheel_velocity;
  if (nh.getParam("max_wheel_velocity", max_wheel_velocity))
  {
    if (max_wheel_velocity >= joint_max_wheel_velocity)
    {
      ROS_WARN_NAMED("BaseController", "Max wheel velocity param %f is greater than joint limit %f",
                     max_wheel_velocity, joint_max_wheel_velocity);
      max_wheel_velocity = joint_max_wheel_velocity;
    }
  }
  params.max_wheel_velocity = max_wheel_velocity;

  try
  {
    limiter_.setParams(params);
  }
  catch (std::exception &ex)
  {
    ROS_ERROR_STREAM_NAMED("BaseController", "Problem with parameters : " << ex.what());
    initialized_ = false;
    return -1;
  }

  // Publisher limiter parameters
  params_pub_ = nh.advertise<robot_controllers_msgs::DiffDriveLimiterParams>("params", 1, true);
  params_pub_.publish(limiter_.getParams());

  // Publish cmd values after they have been limited
  nh.param<bool>("publish_limited_cmd", publish_limited_cmd_, false);
  if (publish_limited_cmd_)
  {
    limited_cmd_pub_ = nh.advertise<geometry_msgs::Twist>("limited_cmd", 10);
  }

  // Subscribe to base commands
  cmd_sub_ = nh.subscribe<geometry_msgs::Twist>("command", 1,
                boost::bind(&DiffDriveBaseController::command, this, _1));

  // Publish odometry & tf
  ros::NodeHandle n;
  odom_pub_ = n.advertise<nav_msgs::Odometry>("odom", 10);
  if (publish_tf_)
    broadcaster_.reset(new tf::TransformBroadcaster());

  // Publish timer
  double publish_frequency;
  nh.param<double>("publish_frequency", publish_frequency, 100.0);
  odom_timer_ = n.createTimer(ros::Duration(1/publish_frequency),
                              &DiffDriveBaseController::publishCallback,
                              this);

  // Should we use the laser for safety limiting base velocity?
  nh.param<double>("laser_safety_dist", safety_scaling_distance_, 0.0);
  nh.param<double>("robot_safety_width", robot_width_, 0.7);
  if (safety_scaling_distance_ > 0.0)
  {
    scan_sub_ = n.subscribe<sensor_msgs::LaserScan>("base_scan", 1,
                  boost::bind(&DiffDriveBaseController::scanCallback, this, _1));
  }

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

  if (std::isfinite(msg->linear.x) && std::isfinite(msg->angular.z))
  {
    boost::mutex::scoped_lock lock(command_mutex_);
    last_command_ = ros::Time::now();
    desired_x_ = msg->linear.x;
    desired_r_ = msg->angular.z;
  }
  else
  {
    ROS_ERROR_NAMED("BaseController", "Commanded velocities not finite!");
    return;
  }

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

bool DiffDriveBaseController::reset()
{
  // Reset command
  last_command_ = ros::Time(0);
  return true;
}

void DiffDriveBaseController::update(const ros::Time& now, const ros::Duration& dt)
{
  if (!initialized_)
    return;  // should never really hit this

  // See if we have timed out and need to stop
  if (now - last_command_ >= timeout_)
  {
    ROS_DEBUG_THROTTLE_NAMED(5, "BaseController", "Command timed out.");
    boost::mutex::scoped_lock lock(command_mutex_);
    desired_x_ = desired_r_ = 0.0;
  }

  // Lock while copying desired velocities and safety scaling
  double desired_x, desired_r, safety_scaling;
  ros::Time last_laser_scan;
  {
    boost::mutex::scoped_lock lock(command_mutex_);
    desired_x = desired_x_;
    desired_r = desired_r_;
    safety_scaling = safety_scaling_;
    last_laser_scan = last_laser_scan_;
  }

  // Make sure laser has not timed out
  if ((safety_scaling_distance_ > 0.0) &&
      (ros::Time::now() - last_laser_scan > ros::Duration(0.5)))
  {
    safety_scaling = 0.1;
  }

  // Perform velocity and acceleration limiting
  double last_update_dt = (now-last_update_).toSec();
  double limited_x, limited_r;
  limiter_.limit(&limited_x, &limited_r,
                 desired_x, desired_r,
                 last_sent_x_, last_sent_r_,
                 safety_scaling, last_update_dt);

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
      limited_x != 0.0 ||
      limited_r != 0.0)
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

  double track_width = limiter_.getTrackWidth();

  // Calculate forward and angular differences
  double d = (left_dx+right_dx)/2.0;
  double th = (right_dx-left_dx)/track_width;

  // Calculate forward and angular velocities
  dx = (left_vel + right_vel)/2.0;
  dr = (right_vel - left_vel)/track_width;

  // Actually set command
  if (fabs(dx) > moving_threshold_ ||
      fabs(dr) > rotating_threshold_ ||
      limited_x != 0.0 ||
      limited_r != 0.0)
  {
    double left_velocity, right_velocity;
    limiter_.calcWheelVelocities(&left_velocity, &right_velocity,
                                 limited_x, limited_r);
    setCommand(left_velocity, right_velocity);
  }

  if (publish_limited_cmd_)
  {
    geometry_msgs::Twist limited_cmd;
    limited_cmd.linear.x = limited_x;
    limited_cmd.angular.z = limited_r;
    limited_cmd_pub_.publish(limited_cmd);
  }


  // Lock mutex before updating
  boost::mutex::scoped_lock lock(odom_mutex_);

  if (std::isfinite(left_vel) && std::isfinite(right_vel))
  {
    // Update stored odometry pose...
    theta_ += th/2.0;
    odom_.pose.pose.position.x += d*cos(theta_);
    odom_.pose.pose.position.y += d*sin(theta_);
    theta_ += th/2.0;
    odom_.pose.pose.orientation.z = sin(theta_/2.0);
    odom_.pose.pose.orientation.w = cos(theta_/2.0);
    // ...and twist
    odom_.twist.twist.linear.x = dx;
    odom_.twist.twist.angular.z = dr;
  }
  else
  {
    ROS_ERROR_THROTTLE_NAMED(1.0,
                             "BaseController",
                             "Ignoring non-finite base movement (%f,%f)",
                             left_vel,
                             right_vel);
  }

  last_update_ = now;
  last_sent_x_ = limited_x;
  last_sent_r_ = limited_r;
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

void DiffDriveBaseController::publishCallback(const ros::TimerEvent& event)
{
  // Copy message under lock of mutex
  nav_msgs::Odometry msg;
  {
    boost::mutex::scoped_lock lock(odom_mutex_);
    msg = odom_;
  }

  // Publish or perish
  msg.header.stamp = ros::Time::now();
  odom_pub_.publish(msg);

  if (publish_tf_)
  {
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(msg.pose.pose.position.x, msg.pose.pose.position.y, 0.0));
    transform.setRotation(tf::Quaternion(msg.pose.pose.orientation.x,
                                         msg.pose.pose.orientation.y,
                                         msg.pose.pose.orientation.z,
                                         msg.pose.pose.orientation.w) );
    /*
     * REP105 (http://ros.org/reps/rep-0105.html)
     *   says: map -> odom -> base_link
     */
    broadcaster_->sendTransform(tf::StampedTransform(transform, msg.header.stamp, msg.header.frame_id, msg.child_frame_id));
  }
}

void DiffDriveBaseController::scanCallback(
  const sensor_msgs::LaserScanConstPtr& scan)
{
  double angle = scan->angle_min;
  double min_dist = safety_scaling_distance_;

  for (size_t i = 0; i < scan->ranges.size(); ++i, angle += scan->angle_increment)
  {
    if (std::isfinite(scan->ranges[i]) &&
        scan->ranges[i] > scan->range_min &&
        scan->ranges[i] < min_dist)
    {
      // Only test points in the forward 180 degrees
      if (angle < -1.5 || angle > 1.5)
        continue;

      // Check if point is inside the width of the robot
      double py = sin(angle) * scan->ranges[i];
      if (fabs(py) < (robot_width_/2.0))
        min_dist = scan->ranges[i];
    }
  }

  boost::mutex::scoped_lock lock(command_mutex_);
  safety_scaling_ = std::max(0.1, min_dist / safety_scaling_distance_);
  last_laser_scan_ = ros::Time::now();
}

void DiffDriveBaseController::setCommand(float left, float right)
{
  // Convert meters/sec into radians/sec
  left_->setVelocity(left * radians_per_meter_, 0.0);
  right_->setVelocity(right * radians_per_meter_, 0.0);
}

}  // namespace robot_controllers
