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
&rarr; Don't forget to add https://github.com/pal-robotics/ddynamic_reconfigure.git </br>
&rarr; when installing librealsense from source in repo you possibly need to use ```catkin_make_isolated```


### Install openPose Pose Tracker

* Follow the guide in https://github.com/CMU-Perceptual-Computing-Lab/openpose/blob/master/doc/installation/0_index.md & clone Version 1.6.0 </br> 
&rarr; For Ubuntu18.04, CUDA 10.1 with cuDNN 7.5.1 the installation steps in https://github.com/ipa-jk/Installationen helped

* Follow the guide in https://github.com/ravijo/ros_openpose


### Clone seher:

Clone this repository: ```git clone https://github.com/ipa-alb/seher.git -b jk/sim```




