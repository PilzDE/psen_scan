^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package psen_scan
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.4 (2020-03-05)
------------------
* Fix/ros parameter handling (#15)
* Add Travis and ROS Buildfarm Badges to README. (#16)
* Minor typefix (#14)
* Fix unstable udp interface test. (#13)
* Feature/code analysis tools (#12)

1.0.3 (2020-01-17)
------------------
* Add x-axis-rotation feature.
  Introduced new parameter "x_axis_rotation" which describes clockwise rotation of the x-axis around the center in degrees.
  The default value is 275°/2 = 137.5°.
  Then the x-axis points in the direction of the middle of the scan.
* Additional change:
  Fill time related message fields in sensor_msgs/LaserScan message.

1.0.2 (2019-12-17)
------------------
* Change spinner to AsyncSpinner. (#6)
* Add missing install of launch files (#5)

1.0.1 (2019-11-25)
------------------
* Initial Commit.
