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
#include <stdlib.h> 
#include <chrono>
//#include <pcl/visualization/cloud_viewer.h>




// Define the function to be called when ctrl-c (SIGINT) is sent
void
signalCallbackHandler(int aSignum)
{
  ROS_INFO("Exit with Signal %d", aSignum);
  ros::shutdown();
}

// get pointcloud as an input,transformes it into ros_pointcloud2 msg and
// publishes it to ros
void
publishScan(ros::Publisher* aPub, pcl::PointCloud<pcl::PointXYZI> aCloud)
{
  sensor_msgs::PointCloud2 cloud_msg;
  ros::Time ros_now = ros::Time::now();

  pcl::toROSMsg(aCloud, cloud_msg);
  cloud_msg.header.frame_id = "map";
  cloud_msg.header.stamp = ros_now;

  aPub->publish(cloud_msg);
}

// get a raw pointcloud(with noise) and eliminate the points which have a confidence value less than int confidancethreshold
pcl::PointCloud<pcl::PointXYZI>  cleanPointCloud(pcl::PointCloud<pcl::PointXYZI> aCloud, int confidencethreshold){
  pcl::PointCloud<pcl::PointXYZI> pc;
  for (int i=0;i<aCloud.size();i++){
    if ( aCloud.points[i].intensity > confidencethreshold && aCloud.points[i].z > 0 ){
     pc.push_back(aCloud.points[i]);

    }
  }

  return pc ;
}

/*
@param inputCloud : acquired pointclud from sensor .
@param refCloud : refrenced pointcloud to be compared too (holds the empty scene).
@param movedPointsCount: number of point that moved.
@param movedByMM : threshold to count moved points. 
function returns true if (actual moved point)>movedPointsCount . the moved points will be accounted for if they move more thant movedByMM.
*/ 
bool somethingNewInFrame(pcl::PointCloud<pcl::PointXYZI> inputCloud, pcl::PointCloud<pcl::PointXYZI> refCloud,int movedPointsCount, int confidencethreshold,float32_t movedByMM){
  bool ret = false;
  int j= 0;
  int count =0;
  for (int i=0;i<inputCloud.size();i++){
    if (abs(inputCloud.points[i].x-refCloud.points[i].x) > movedByMM || 
        abs(inputCloud.points[i].y-refCloud.points[i].y) > movedByMM ||
        abs(inputCloud.points[i].z-refCloud.points[i].z) > movedByMM ){
      
        if (inputCloud.points[i].intensity > confidencethreshold && refCloud.points[i].intensity > confidencethreshold){
            j++;
            count++;
        }
    }
  }
  
  ROS_INFO("COUNT POINTS THAT VARIATES: %d",count);
  if (count >movedPointsCount ){
    ret = true;
  }
  return ret ;
}

//return index of closest point 
int closestPoint(pcl::PointCloud<pcl::PointXYZI> inputCloud){
  int index = 0;
  for (int i=0;i<inputCloud.size();i++){
    if (inputCloud.points[index].z > inputCloud.points[i].z && inputCloud.points[i].z >0){
      index = i; 
    //ROS_INFO(" THIS IS index = %f and this is neapoint = %f",inputCloud.points[index].z , inputCloud.points[i].z);

    }
  }
  return index ;
}

int
main(int argc, char* argv[])
{

  // initial camera parameter
  arena_camera::mControlParameters gDefaultParam;
  gDefaultParam.iopMode = ArenaFlex::AF_DEV_OPERATING_MODE_1500MM;

  // Creating camera object
  arena_camera gCamera;
  gCamera.openStream(gDefaultParam);

  // Initialize ROS node
  ros::init(argc, argv, "licel_pointcloud");
  ros::NodeHandle gNode_handler;
  ros::Publisher gPub =
    gNode_handler.advertise<sensor_msgs::PointCloud2>("licel_arenaflex_raw", 3);

  // struct to hold the acquired parameter from rosparameter server
  arena_camera::mControlParameters gRosparam;

  // Read Ros parameter server and update the camera parameters
  gRosparam = gCamera.getRosParam(gNode_handler);
  gRosparam.iopMode = ArenaFlex::AF_DEV_OPERATING_MODE_1500MM;

  ROS_INFO("Started in Operation Mode 1500mm");

  // set param to Ros paramter
  gCamera.setStreamParam(gRosparam);

  int gSigsetError;
  // define Signal Handling set
  sigset_t oldsigset, sigset;
  gSigsetError = sigfillset(&oldsigset);
  gSigsetError = sigfillset(&sigset);

  // signal handler to properly close device when pressing ctr-c
  signal(SIGINT, signalCallbackHandler);

  // Get Ros Paramter Empty_scene If empty scene than save the PC
  std::string gEmptyScene ; 
  gNode_handler.getParam("/licel_pointcloud/scene",gEmptyScene);
  pcl::PointCloud <pcl::PointXYZI> refCloud;


  if (gEmptyScene == "scene" ){
    ROS_INFO("Recording Empty scene");
    // blocking interrupt signal in this CRITICAL CODE SECTION  BEGIN
    gSigsetError = sigprocmask(SIG_BLOCK, &sigset, &oldsigset);

    // fill gCamera.cloud with point cloud data
    
    gCamera.getPointCloud();
    

    pcl::io::savePCDFileASCII ("/home/seherlicel/Desktop/code_review_1/SeherSources/licel_catkin_ws/src/licel_pointcloud/empty_scene/empty_scene.pcd", gCamera.mCloud);
    // unblocking interrupt signal  CRITICAL CODE SECTION  OVER
    gSigsetError = sigprocmask(SIG_UNBLOCK, &sigset, &oldsigset);
  }
  
  else{
    pcl::io::loadPCDFile ("/home/seherlicel/Desktop/code_review_1/SeherSources/licel_catkin_ws/src/licel_pointcloud/empty_scene/empty_scene.pcd", refCloud);
    ros::Publisher gPub_static_image =gNode_handler.advertise<sensor_msgs::PointCloud2>("licel_arenaflex_filtered", 3);
    pcl::PointCloud<pcl::PointXYZI> filtredPointCloud ;
    filtredPointCloud.resize(307200);
      do {        
      
        // since we are operating on buffer and have no direct control over it (all
        // buffer handlig are done using ARENAFlex API),
        // to avoid any buffer misshandling we block incoming interrupt signals till
        // the points are published and the buffer realased

        // blocking interrupt signal in this CRITICAL CODE SECTION  BEGIN
        gSigsetError = sigprocmask(SIG_BLOCK, &sigset, &oldsigset);

        // fill gCamera.cloud with point cloud data
        auto start = std::chrono::high_resolution_clock::now();
        gCamera.getPointCloud();
        
        // publishes the point cloud data to ros
        publishScan(&gPub, gCamera.mCloud);
        
        filtredPointCloud= cleanPointCloud(gCamera.mCloud,gRosparam.iConfidence_threshold_value);
        int closest_index = closestPoint(filtredPointCloud);
        double MIN_distance = sqrt(pow (filtredPointCloud.points[closest_index].x,2)+pow(filtredPointCloud.points[closest_index].y,2)+pow(filtredPointCloud.points[closest_index].z,2) );
        ROS_WARN("CLOSEST POINT IS X:%f Y:%f Z:%f with MIN_distance = %f ", filtredPointCloud.points[closest_index].x,filtredPointCloud.points[closest_index].y,filtredPointCloud.points[closest_index].z,MIN_distance);
 
        publishScan(&gPub_static_image,filtredPointCloud);
        
        if (somethingNewInFrame(gCamera.mCloud,refCloud,3500,gRosparam.iConfidence_threshold_value,0.001))
        { 
          ROS_WARN("ETWAS LIEGT IM BILD");
        }

        auto finish = std::chrono::high_resolution_clock::now();
        ROS_WARN(" %d" ,std::chrono::duration_cast<std::chrono::nanoseconds>(finish-start).count());
        
        // unblocking interrupt signal  CRITICAL CODE SECTION  OVER
        gSigsetError = sigprocmask(SIG_UNBLOCK, &sigset, &oldsigset);
        
        // Sleep 0,5ms
        usleep(5);

      }while (ros::master::check() && ros::ok());
  }

  // Cleaning camera object
  gCamera.stopAndCloseStream();
  return 0;
}