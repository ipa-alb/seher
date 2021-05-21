cmake_minimum_required(VERSION 2.8.3)
project(vfk_msb_client)


find_package(catkin REQUIRED COMPONENTS
)

catkin_python_setup()

catkin_package(
)

install(PROGRAMS
   scripts/msb_ros_bridge.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
