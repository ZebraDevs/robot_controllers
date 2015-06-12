^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package robot_controllers
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
