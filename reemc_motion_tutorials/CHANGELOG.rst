^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package reemc_motion_tutorials
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* reemc_motion_tutorials: Add rqt example resources
* Contributors: Adolfo Rodriguez Tsouroukdissian

0.0.4 (2015-06-12)
------------------

0.0.3 (2015-06-10)
------------------

0.0.2 (2015-06-09)
------------------

0.0.1 (2015-06-08)
------------------
* Add axcli runtime dep, rqt perspective
* Add 'simple_grasp' script
  Port script from ULNA+Hey5 project and extend it to work with both right and
  left hands.
* Drop reach_pose in favor of play_motion
  - Remove reach_motion binary, and associated configuration files.
  - Port motion sequence script from ULNA-Hey5 project for executing a sequence
  of play_motion motions.
  - Create an example motion sequence.
* Remove not valid folder from installation
* Removed not needed dependencies on rviz and rxtool
* Fix controllers tutorial for package name modification. Fix motion tutorials dependencies. Set right path in launch file for basic tutorials
* Move code and scripts for movements to reemc_motion_tutorials
* Contributors: Adolfo Rodriguez Tsouroukdissian, Luca Marchionni
