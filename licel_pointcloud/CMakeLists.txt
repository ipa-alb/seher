cmake_minimum_required(VERSION 3.0.2)
project(licel_pointcloud)


find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
)




###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES licel_pointcloud
  CATKIN_DEPENDS pcl_conversions pcl_ros roscpp sensor_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

add_executable(licel_pointcloud_publisher_1500mm src/licel_pointcloud_publisher_1500mm.cpp src/arena_flex_camera.cpp)
target_link_libraries(licel_pointcloud_publisher_1500mm ${catkin_LIBRARIES})
target_link_libraries(licel_pointcloud_publisher_1500mm libarenaflex.so )
#target_link_libraries(licel_pointcloud_publisher_1500mm licel_catkin_ws/devel/lib/usr/lib/aarch64-linux-gnu/libopencv*)
link_directories( /home/seherlicel/Desktop/code_review_1/SeherSources/licel_catkin_ws/devel/lib/opencv_dirty/usr/lib/aarch64-linux-gnu)


#add_executable(licel_pointcloud_publisher_6000mm src/licel_pointcloud_publisher_6000mm.cpp src/arena_flex_camera.cpp)
#target_link_libraries(licel_pointcloud_publisher_6000mm ${catkin_LIBRARIES})
#target_link_libraries(licel_pointcloud_publisher_6000mm libarenaflex.so )
#link_directories(licel_pointcloud_publisher_6000mm  /home/seherlicel/Desktop/code_review_1/SeherSources/licel_catkin_ws/devel/lib/opencv_dirty/usr/lib/aarch64-linux-gnu/lib*))


