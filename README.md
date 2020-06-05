# Robot Controllers

This is a robot control infrastructure, developed initially for Fetch and Freight, but
designed to be robot-agnostic. In comparison to ros_control, robot_controllers offers
the ability to "stack" controllers and avoids template-based hardware interfaces.
robot_controllers has not be designed with real-time constraints in mind, and is intended
primarily for robots where the real-time joint-level controllers are run in hardware,
as is the case for Fetch and Freight.

 * Devel Job Status: [![Build Status](http://build.ros.org/buildStatus/icon?job=Mdev__robot_controllers__ubuntu_bionic_amd64)](http://build.ros.org/job/Mdev__robot_controllers__ubuntu_bionic_amd64/)
 * AMD64 Debian Job Status: [![Build Status](http://build.ros.org/buildStatus/icon?job=Mbin_uB64__robot_controllers__ubuntu_bionic_amd64__binary)](http://build.ros.org/view/Mbin_uB64/job/Mbin_uB64__robot_controllers__ubuntu_bionic_amd64__binary/)
