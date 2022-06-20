# Robot Controllers

This is a robot control infrastructure, developed initially for Fetch and Freight, but
designed to be robot-agnostic. In comparison to ros_control, robot_controllers offers
the ability to "stack" controllers and avoids template-based hardware interfaces.
robot_controllers has not be designed with real-time constraints in mind, and is intended
primarily for robots where the real-time joint-level controllers are run in hardware,
as is the case for Fetch and Freight.

 * Devel Job Status: [![Build Status](https://build.ros2.org/buildStatus/icon?job=Fdev__robot_controllers__ubuntu_focal_amd64)](https://build.ros2.org/job/Fdev__robot_controllers__ubuntu_focal_amd64/)
 * AMD64 Debian Job Status:
   * robot_controllers_msgs: [![Build Status](https://build.ros2.org/buildStatus/icon?job=Fbin_uF64__robot_controllers_msgs__ubuntu_focal_amd64__binary)](https://build.ros2.org/job/Fbin_uF64__robot_controllers_msgs__ubuntu_focal_amd64__binary/)
   * robot_controllers_interface: [![Build Status](https://build.ros2.org/buildStatus/icon?job=Fbin_uF64__robot_controllers_interface__ubuntu_focal_amd64__binary)](https://build.ros2.org/job/Fbin_uF64__robot_controllers_interface__ubuntu_focal_amd64__binary/)
   * robot_controllers: [![Build Status](https://build.ros2.org/buildStatus/icon?job=Fbin_uF64__robot_controllers__ubuntu_focal_amd64__binary)](https://build.ros2.org/job/Fbin_uF64__robot_controllers__ubuntu_focal_amd64__binary/)

## ROS1 -> ROS2 Migration

 * ControllerManager:
   * The ControllerManager class is now part of the robot_controllers_interface namespace.
   * The ControllerManager is now passed to controllers by std::shared_ptr. It therefore derives
     from std::enable_shared_from_this. You MUST create your ControllerManager instance as a
     shared pointer - we recommend using std::make_shared.
   * The action query interface has been converted to a service interface. ROS2 allows asynchronous
     services and so the overhead of an action interface is no longer warranted. See the scripts
     in robot_controllers_interface package for examples of using this interface.
   * The ControllerManager has a constructor that takes a tf2_ros::Buffer input, when this is
     used, no additional TransformListeners will be created.
 * Controllers:
   * All controllers have migrated to using TF2.
   * The CartesianPose controller now takes a a TwistStamped (previously, it used an unstamped
     message).
   * The diff drive controller now takes a list of joint names for the left/right side, allowing
     use with 4wd robots.
   * ParallelGripper: a new parameter "use_centering_pid" must be set to true to use the centering
     PID (with all parameters needing to be declared, ROS2 doesn't handle our older style
     interface).
 * Custom controllers:
   * The Controller class is now in the robot_controllers_interface namespace. This was a bug in
     ROS1 which caused all custom controller packages to need a dependency on robot_controllers,
     so we took the opportunity to fix that dependency in ROS2.
   * The init() call signature has changed. The first parameter is now the string name of the
     controller. This is due to the changes in namespacing in ROS2 and some existing bugs in
     "subnodes". Simply pass this name string onto Controller::init() and everything will continue
     working. The ros::NodeHandle has been replaced by a std::shared_ptr to the rclcpp::Node. Most
     controllers will need to keep a reference to this Node in order to access time and logging.
     The controller manager is now passed as a std::shared_ptr as well.
   * The calls to update() now take rclcpp::Time and rclcpp::Duration instances.
   * ROS2 time is very different from ROS1 time - more analogous to std::chrono than ROS1 time.
     If your controllers used toSec() significantly, you will want to check out
     robot_controllers_interface/utils.h which includes some functions to quickly convert
     rclcpp::Time and Duration to and from floating-point seconds.
