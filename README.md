# seher - Person Tracking

## Installation steps

&rarr; all of the following packages should be cloned in the src-file of your workspace

### Install required repositories for Robot Dummy Task Setup

* Clone universal_robot repository: </br>
```git clone git clone https://github.com/ipa-kut/universal_robot -b sim```

* Clone ur5e_egp50_moveit_config repository: </br>
```git clone https://github.com/ipa-kut/ur5e_egp50_moveit_config```

* Clone & Install ur_manipulation repository: ```git clone https://github.com/ipa-kut/ur_manipulation ``` </br>
&rarr; don't forget the step, which installs all other requirements for that repository (see https://github.com/ipa-kut/ur_manipulation):</br>
```rosdep install --from-paths src --ignore-src -r -y```

### Install required repositories for Camera Setup

* Follow the guide in https://github.com/IntelRealSense/realsense-ros </br>
&rarr; Don't forget to add https://github.com/pal-robotics/ddynamic_reconfigure.git

### Clone seher:

Clone this repository: ```git clone https://github.com/ipa-alb/seher.git -b jk/sim```

### Launch setup
* Make sure conda is deactivated: ```conda deactivate```

* Add caffe library and openpose to LD_LIBRARY_PATH: 
* ```LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/student/seher_ws/src/openpose/caffe/lib/:/home/student/seher_ws/src/openpose/src/openpose/
 export LD_LIBRARY_PATH```
* source workspace(here 'seher_ws'): ```source seher_ws/devel_isolated/setup.bash```

* start setup: ```roslaunch seher setup_start.launch```




