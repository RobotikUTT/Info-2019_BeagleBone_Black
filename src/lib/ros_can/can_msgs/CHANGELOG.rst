^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package can_msgs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.6.8 (2017-07-18)
------------------

0.6.7 (2017-03-28)
------------------

0.6.6 (2017-03-18)
------------------

0.6.5 (2016-12-10)
------------------
* hamonized versions
* styled and sorted CMakeLists.txt
  * removed boilerplate comments
  * indention
  * reviewed exported dependencies
* styled and sorted package.xml
* Adds message_runtime to can_msgs dependencies.
  Added the missing dependency, also changes message_generation to a build_depend.
* Finalizes work on the socketcan_bridge and can_msgs.
  Readies the packages socketcan_bridge and can_msgs for the merge with ros_canopen.
  Bumps the version for both packages to 0.1.0. Final cleanup in CMakeLists, added
  comments to the shell script and launchfile used for testing.
* Introduces the can_msgs package for message types.
  Package to hold CAN message types, the Frame message type can contain the data
  as returned by SocketCAN.
* Contributors: Ivor Wanders, Mathias Lüdtke
