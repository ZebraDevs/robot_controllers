^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package robot_controllers
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------

0.9.2 (2022-06-21)
------------------

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
* use from_seconds for readability (`#70 <https://github.com/fetchrobotics/robot_controllers/issues/70>`_)
* Contributors: Michael Ferguson

0.8.1 (2020-12-02)
------------------
* add Fergs as maintainer (`#65 <https://github.com/fetchrobotics/robot_controllers/issues/65>`_)
* additional boost disabling (`#64 <https://github.com/fetchrobotics/robot_controllers/issues/64>`_)
* disable boost components of pluginlib (`#63 <https://github.com/fetchrobotics/robot_controllers/issues/63>`_)
* Contributors: Michael Ferguson

0.8.0 (2020-11-25)
------------------
* Add initial support for ROS2
* fix threading bug in point head controller (`#62 <https://github.com/fetchrobotics/robot_controllers/issues/62>`_)
* get latest transform (`#60 <https://github.com/fetchrobotics/robot_controllers/issues/60>`_)
* add linter, fix errors (`#48 <https://github.com/fetchrobotics/robot_controllers/issues/48>`_)
* parallel gripper tested (`#56 <https://github.com/fetchrobotics/robot_controllers/issues/56>`_)
* remove hard coded frame in point_head (`#55 <https://github.com/fetchrobotics/robot_controllers/issues/55>`_)
* build fixes on eloquent (`#54 <https://github.com/fetchrobotics/robot_controllers/issues/54>`_)
* add support for 4wd bases (`#53 <https://github.com/fetchrobotics/robot_controllers/issues/53>`_)
* test, fix and improve scaled_mimic (`#52 <https://github.com/fetchrobotics/robot_controllers/issues/52>`_)
* fix bug in laser checking: robot would always go slow if laser safety was on.
* Uses C++14
* make preemption work
* fix uninitialized time that would occasionally cause std::runtime_error* improve logging
* switch to service interface
* port all controllers and tested
* add util/declare_parameter_once
* Add time_from_start information to feedback topic (`#38 <https://github.com/fetchrobotics/robot_controllers/issues/38>`_)
* [GCC][Warnings] SYSTEM includes and catch ref (`#36 <https://github.com/fetchrobotics/robot_controllers/issues/36>`_)
  This is an attempt to fix (silence) the buildbot failures from -Werror
* Updates maintainers
* Add init trajectory for future start time (`#40 <https://github.com/fetchrobotics/robot_controllers/issues/40>`_)
* Contributors: Alex Moriarty, Michael Ferguson, Naoya Yamaguchi, Russell Toris, Shingo Kitagawa

0.6.0 (2018-07-11)
------------------
* updates ownership
* Contributors: Russell Toris

0.5.3 (2017-06-11)
------------------
* remove limiter
* Improve cartesian twist controller (`#26 <https://github.com/fetchrobotics/robot_controllers/issues/26>`_)
* Contributors: Hanjun Song, Michael Ferguson

0.5.2 (2016-07-18)
------------------
* do not export python library for linking
* Contributors: Michael Ferguson

0.5.1 (2016-07-18)
------------------
* base_controller: only update odometry if inputs are finite (prevents NANs to TF)
* base_controller: add velocity limiting
* pid: fix error in derivative error calculation
* allow velocity limiting code be reused for forward simulation `#18 <https://github.com/fetchrobotics/robot_controllers/issues/18>`_
* Contributors: Cappy Pitts, Derek King, Michael Ferguson

0.5.0 (2016-02-21)
------------------
* require finite commands to base controller
* Contributors: Michael Ferguson

0.4.3 (2015-12-05)
------------------
* fix path/goal tolerance preparation
* Contributors: Michael Ferguson

0.4.2 (2015-10-22)
------------------
* fixed segmentation faults due to misconfiguration
* Contributors: Arvin Asokan

0.4.1 (2015-06-12)
------------------
* add centering pid to gripper controller
* Contributors: Michael Ferguson

0.4.0 (2015-05-23)
------------------

0.3.4 (2015-05-22)
------------------
* add ability to reset controllers
* add timeout to laser speed scaling
* maintain constant curvature when scaling base velocity
* remove DiffDriveBaseController::publish()
* Contributors: Michael Ferguson

0.3.3 (2015-05-03)
------------------
* use laser to slow base when obstacles are near
* add mutexes around command/odometry. publish odom in timer
* add -Wall to compile flags, fix compile warnings
* Contributors: Derek King, Michael Ferguson

0.3.2 (2015-04-09)
------------------

0.3.1 (2015-03-28)
------------------
* use shortest_angular_distance for diff drive dx calculations
* Contributors: Michael Ferguson

0.3.0 (2015-03-23)
------------------
* improve windup support
* make spliced trajectory consistent in qd/qdd size
* Contributors: Michael Ferguson

0.1.4 (2015-03-13)
------------------
* follow joint trajectory: add stop_on_path_violation parameter
* diff drive controller: split theta update into two parts
* diff drive controller: fix frames in odometry message
* diff drive controller: add autostart
* scaled mimic controller: add autostart
* add root/tip params to gravity compensation
* Contributors: Michael Ferguson

0.1.3 (2015-01-28)
------------------
* add scaled mimic controller (for bellows)
* Contributors: Michael Ferguson

0.1.2 (2015-01-06)
------------------

0.1.1 (2015-01-05)
------------------
* initial release
* Contributors: Michael Ferguson
