Fraunhofer IPA
Mathieu Bapst

Seher Project

Robot Human Collaboration

Abstract: This repo is the source code of the software for a human robot collaboration demonstrator.
HW: A UR5e robot and 4 Realsense D435 cameras
SW environment: Linux Ubuntu 18.04 and ROS Melodic
The demonstrator has following functions:

- An occupancy map of the workcell is built and represented in Rviz, based on the camera sensors.
- Minimal distance is computed between robot TCP and the occupancy map
- Based on the minimal distance, the robot is slowed down or stopped if the human comes too near to it.
- Thanks to a human skeleton tracker, the human hand position is tracked 
- An autonomous tool handover between the robot and the human and vice-versa can be performed, when triggered by the human.
- The robot behaviour is based on a state machine.
- The connection is established between the ROS system and the MSB. Data can be sent and received.
- The minimal distance calculation can be performed from remote computer through MSB.

The developed codes and launch files are in the package "seher" and "seher_msgs". "cob_people_perception" has been modified too.

Necessary packages which have to be cloned in your repository :
- cob_perception_common from https://github.com/ipa320/cob_perception_common.git
- fmauch_universal_robot , Universal_Robots_ROS_Driver ur5e_egp50_moveit_config (for ur5e robot) as described on https://github.com/ipa-kut/ur_manipulation. Follow the steps given on the page.
- openni2_camera from https://github.com/ros-drivers/openni2_camera.git
- realsense_ros from https://github.com/IntelRealSense/realsense-ros
- rgbd_launch from https://github.com/ros-drivers/rgbd_launch
- seher_support from https://github.com/ipa-kut/seher_support
- robot_self_filter from https://github.com/PR2/robot_self_filter.git

Install OpenNI2 wrapper for realsense as described in https://github.com/IntelRealSense/librealsense/tree/master/wrappers/openni2 (for human body tracking)

With current gripper, set "pincer_height" property to 0.042 in ur_manipulation/urdf/egp50.urdf.xacro.
Timeout in Universal_Robots_ROS_Driver/ur_robot_driver/resources/ros_control.urscript Ln107 set to 0.1 instead of 0.02, in order to avoid TCP connection dropping because of non real-time system.

To launch the workcell setup :
- clone the above given repos.
- build (catkin_make_isolated necessary) and source the repository
- roslaunch seher workcell_setup.launch

To launch the robot moving demonstration with speed reduction and stopping when approaching as well as the tool handover function :
- roslaunch seher move_robot

Hint!! robot has to be in Remote Control mode when launching the workcell setup. This must be activated from Teach Pendant. AND External Control URCaps must be put in the program on Teach Pendant.

For the cloud/MSB part:
- To send/receive data to/from the MSB: roslaunch seher MSB_data_sending.launch
Hint: vfk_msb_client is required in workspace/src

-To send the occupancy map and the robot positions to the MSB: roslaunch seher MSB_distance_calculation_sender

-From a remote computer, to get the occupancy map and the robot positions from the MSB and to then send the minimal distance to the MSB: roslaunch seher MSB_distance_calculation_receiver
And then, also run the distance computing node: rosrun seher distance_calculation
