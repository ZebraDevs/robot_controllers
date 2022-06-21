^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package robot_controllers_interface
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* fix a bug in the older constructor (`#78 <https://github.com/mikeferguson/robot_controllers/issues/78>`_)
* Contributors: Michael Ferguson

0.9.1 (2022-06-20)
------------------
* move transform listener to manager (`#77 <https://github.com/fetchrobotics/robot_controllers/issues/77>`_)
  each transform listener incurs an extra DDS node, which can
  have a significant effect in a large system. this change
  allows our robot driverrs to have a single shared transform
  listener across all controllers (and even the whole node,
  since the buffer can be passed in with the new constructor)
* Contributors: Michael Ferguson

0.9.0 (2022-06-08)
------------------
* update logging messages for galactic and later (`#75 <https://github.com/fetchrobotics/robot_controllers/issues/75>`_)
  https://docs.ros.org/en/humble/Releases/Release-Galactic-Geochelone.html#change-in-rclcpp-s-logging-macros
  also add a few build cleanups, fails on humble otherwise
* Contributors: Michael Ferguson

0.8.1 (2020-12-02)
------------------
* add Fergs as maintainer (`#65 <https://github.com/fetchrobotics/robot_controllers/issues/65>`_)
* additional boost disabling (`#64 <https://github.com/fetchrobotics/robot_controllers/issues/64>`_)
* Contributors: Michael Ferguson

0.8.0 (2020-11-25)
------------------
* Add initial support for ROS2
* implement getJointNames/getControllerNames (`#51 <https://github.com/fetchrobotics/robot_controllers/issues/51>`_)
  this was discussed in `#39 <https://github.com/fetchrobotics/robot_controllers/issues/39>`_
* Merge pull request `#47 <https://github.com/fetchrobotics/robot_controllers/issues/47>`_ from mikeferguson/ros2
* improve logging
* switch to service interface
* gravity compensation controller working in ros2
* Various bug and API fixes
* Added gyro interface to robot controllers (`#43 <https://github.com/fetchrobotics/robot_controllers/issues/43>`_)
* Updates maintainers
* Contributors: Alex Moriarty, Carl Saldanha, Michael Ferguson, Russell Toris

0.6.0 (2018-07-11)
------------------
* updates ownership
* Contributors: Russell Toris

0.5.3 (2017-06-11)
------------------
* add error message when pluginlib fails
* fix cmake warnings on kinetic (`#28 <https://github.com/fetchrobotics/robot_controllers/issues/28>`_)
* Contributors: Michael Ferguson

0.5.2 (2016-07-18)
------------------

0.5.1 (2016-07-18)
------------------
* Dynamically load controllers (`#23 <https://github.com/fetchrobotics/robot_controllers/issues/23>`_)
  * When requested controller not in default list, check parameter server for controller
  * Controller loader catches pluginlib exception when trying to load bad controller
* Return controller states even if update fails
* Contributors: Michael Ferguson, Sarah Elliott

0.5.0 (2016-02-21)
------------------

0.4.3 (2015-12-05)
------------------
* add mainpage to doxygen
* Contributors: Michael Ferguson

0.4.2 (2015-10-22)
------------------
* add -Wall to interface package
* Contributors: Michael Ferguson

0.4.1 (2015-06-12)
------------------

0.4.0 (2015-05-23)
------------------

0.3.4 (2015-05-22)
------------------
* add ability to reset controllers
* Contributors: Michael Ferguson

0.3.3 (2015-05-03)
------------------

0.3.2 (2015-04-09)
------------------
* install scripts
* Contributors: Michael Ferguson

0.3.1 (2015-03-28)
------------------

0.3.0 (2015-03-23)
------------------
* improve windup support
* fix potential build issue
* Contributors: Michael Ferguson

0.1.4 (2015-03-13)
------------------
* add action server for get/set of controller state
* make autostart actually work
* Contributors: Michael Ferguson

0.1.3 (2015-01-28)
------------------
* additional logging when requestStart() fails
* Contributors: Michael Ferguson

0.1.2 (2015-01-06)
------------------
* install robot_controllers_interface
* Contributors: Michael Ferguson

0.1.1 (2015-01-05)
------------------
* initial release
* Contributors: Michael Ferguson
