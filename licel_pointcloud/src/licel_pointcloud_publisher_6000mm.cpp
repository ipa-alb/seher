/***************************************************************************************
 ***  Copyright (c) 2021, Licel GMBh                                                   *
 ***  Author:Elyes Harzallah                                                           *
 ***************************************************************************************/
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include "arena_flex_camera.h"
#include <unistd.h>



// Define the function to be called when ctrl-c (SIGINT) is sent
void
signalCallbackHandler(int aSignum)
{
  ROS_INFO("Exit with Signal %d", aSignum);
  ros::shutdown();
  }

// get pointcloud as an input,tranformes it onto ros_pointcloud2 msg and
// publishes it to ros
void
publishScan(ros::Publisher* aPub, pcl::PointCloud<pcl::PointXYZI> aCloud)
{
  sensor_msgs::PointCloud2 cloud_msg;
  ros::Time ros_now = ros::Time::now();

  pcl::toROSMsg(aCloud, cloud_msg);
  cloud_msg.header.frame_id = "licel";
  cloud_msg.header.stamp = ros_now;

  aPub->publish(cloud_msg);
}


int
main(int argc, char** argv)
{

  // initial camera parameter
  arena_camera::mControlParameters gDefaultParam;
  gDefaultParam.iopMode = ArenaFlex::AF_DEV_OPERATING_MODE_6000MM;
  // Creating camera object
  arena_camera gCamera;

  /*
  Internal note: need to open stream before initiating ros otherwise race
  condition Need to set operating mode before openeing stream otherwise error
  */
  gCamera.openStream(gDefaultParam);

  // Initialize ROS
  ros::init(argc, argv, "licel_pointcloud");
  ros::NodeHandle gNode_handler;
  ros::Publisher gPub =
    gNode_handler.advertise<sensor_msgs::PointCloud2>("licel_arenaflex_pc2", 10);

  // struct to hold the acquired parameter from rosparameter server
  arena_camera::mControlParameters gRosparam;
  // Read Ros parameter server and update the camera parameters
  gRosparam = gCamera.getRosParam(gNode_handler);
  gRosparam.iopMode = ArenaFlex::AF_DEV_OPERATING_MODE_6000MM;

  ROS_INFO("Started in Operation Mode 6000mm");

  // set param to Ros paramter
  gCamera.setStreamParam(gRosparam);

  int gSigsetError;
  // define Signal Handling set
  sigset_t oldsigset, sigset;
  gSigsetError = sigfillset(&oldsigset);
  gSigsetError = sigfillset(&sigset);

  // signal handler to properly close device when pressing ctr-c
  signal(SIGINT, signalCallbackHandler);
  std::string gEmptyScene ; 
  gNode_handler.getParam("empty_scene",gEmptyScene);
  
  if (gEmptyScene == "empty_scene" ){

    // blocking interrupt signal in this CRITICAL CODE SECTION  BEGIN
    gSigsetError = sigprocmask(SIG_BLOCK, &sigset, &oldsigset);

    // fill gCamera.cloud with point cloud data
    gCamera.getPointCloud();

    pcl::io::savePCDFileASCII ("test_pcd.pcd", gCamera.mCloud);
    // unblocking interrupt signal  CRITICAL CODE SECTION  OVER
    gSigsetError = sigprocmask(SIG_UNBLOCK, &sigset, &oldsigset);
    
    // Sleep 0,5ms
    usleep(500);
  }
  else{
    
    do {
      // since we are operating on buffer and have no direct control over it (all
      // buffer handlig are done using ARENAFlex API),
      // to avoid any buffer misshandling we block incoming interrupt signals till
      // the points are published and the buffer realased

      // blocking interrupt signal in this CRITICAL CODE SECTION  BEGIN
      gSigsetError = sigprocmask(SIG_BLOCK, &sigset, &oldsigset);

      // fill gCamera.cloud with point cloud data
      gCamera.getPointCloud();

      // publishes the point cloud data to ros
      publishScan(&gPub, gCamera.mCloud);

      // unblocking interrupt signal  CRITICAL CODE SECTION  OVER
      gSigsetError = sigprocmask(SIG_UNBLOCK, &sigset, &oldsigset);
      
      // Sleep 0,5ms
      usleep(500);

    }while (ros::master::check() && ros::ok());
  }
  // Cleaning camera object
  gCamera.stopAndCloseStream();
  return 0;
}