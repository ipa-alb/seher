Fraunhofer IPA
Mathieu Bapst

Seher Project

Robot Human Collaboration

The new codes and launch files are in the package "seher"

SIM BRANCH : working with a gazebo simulated ur5e robot.

Packages needed to launch the simulated robot on Gazebo:
- git clone https://github.com/ipa-kut/universal_robot -b sim
- git clone https://github.com/ipa-kut/ur5e_egp50_moveit_config -b sim
- git clone https://github.com/ipa-kut/ur_manipulation -b sim

instead of these mentioned for the workcell setup (master branch)


To launch the workcell setup :
- build and source the repository
- roslaunch seher workcell_setup.launch    (in this launch file are all the needed commands to launch the simulated robot thanks to Gazebo)

For the home setup :

both cameras are placed in parallel and separetad by 47 cm apporximately

They visualize the scene at a depth of 1.5 m, the scene is cropped and voxelized.

To launch the home setup :

-build and source the repository
- roslaunch seher home_setup.launch to launch all the setup + moveit + distanceCalcualtion node

Then to launch the robot moving node with distance monitoring :
- rosrun seher move_sim_robot (in new terminal)
