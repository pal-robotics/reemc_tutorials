^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package reemc_wbc
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.4 (2015-06-11)
------------------
* Added topic to visualize dancing markers
* renamed stabilizer to ft_imu of dancing file
* Added more coments
* Contributors: Hilario Tome

1.0.3 (2015-06-10)
------------------
* changed ft names to ankle
* Added documentation and licenses
* refactoring for TUM tutorials
* Fixed typo in include
* Fixed include in gtest
* Contributors: Hilario Tome

1.0.2 (2015-06-05)
------------------

1.0.1 (2015-06-04)
------------------
* removed old depends from walking tools and rbld dynamics
* Contributors: Hilario Tome

1.0.0 (2015-06-04)
------------------
* Fixing versions
* Added reemc_wbc gtest
* Changed capsule decomposition to capsule collision in launch file
* Clean up
* Succesfully tested in REEM-C acceptance of whole body control for TUM, added nan safetry in kinematic controller, added reem_wbc, and migrated old reem-c stacks
* Tested working stabilizer in real reemc
* Added FT and IMU to kinematic controller
* Removed damping parameters from param server
* Added reserve to levels
* Fix install rule
* Added tiago_wbc, bug when using stack with position, orientation, and bug with self collision
* Fixed missing function implementation in torque damping task and made solver realtime
* Added extra dynamic stacks
* Succesfully added friction cone formulation and friction cone visualization
* walking dynamic with unilateral constraints working
* Updated dynamic tasks
* Major refactoring of all pwc, dependency with soth removed, solver and stackOfTacks are decoupled, individual wbc control package for every robot, kinematic wbc tested, it remains to test dynamic
* Contributors: Bence Magyar, Hilario Tome
