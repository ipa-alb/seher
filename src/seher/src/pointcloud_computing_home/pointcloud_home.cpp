#include <pcl/filters/crop_box.h>
#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>

#include <Eigen/Dense>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <iostream>
#include "tf/transform_listener.h"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>


//ros::Publisher pub1;
//ros::Publisher pub2;
ros::Publisher pub;

ros::Subscriber sub1;
ros::Subscriber sub2;


tf::TransformListener* listenerL;  
tf::TransformListener* listenerR;

sensor_msgs::PointCloud2 pcl_fusion;
sensor_msgs::PointCloud2 pcl_L;
sensor_msgs::PointCloud2 pcl_R;



//workcell 
const Eigen::Vector4f min_workcell =Eigen::Vector4f(0,0,0,1);
const Eigen::Vector4f max_workcell =Eigen::Vector4f(1.0,1.0,2.0,1);

//const Eigen::Vector4f min_workcell =Eigen::Vector4f(0.076f,0.793f,1.625f,1); // ok ?
//const Eigen::Vector4f max_workcell =Eigen::Vector4f(0.294f--0.77f,0.29f,1);


const Eigen::Vector3f t2=Eigen::Vector3f(-0.5f,-0.5f,0.0f);
const Eigen::Vector3f r2=Eigen::Vector3f(0.0f,0.0f,0.0f);



void cloud_cb1 (const sensor_msgs::PointCloud2ConstPtr& input)
{
  // Create a container for the data.
  sensor_msgs::PointCloud2 inter4;
  
 
  
 pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
 pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
 pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2 ());

 
 // Pointcloud cropping 
 pcl_conversions::toPCL(*input, *cloud);

  pcl::CropBox<pcl::PCLPointCloud2> cropFilter;
  
  //cropFilter.setTransform(trans2);
  cropFilter.setMin(min_workcell);
  cropFilter.setMax(max_workcell);
  cropFilter.setTranslation(t2);
  //cropFilter.setRotation(r2);
  cropFilter.setInputCloud(cloudPtr);
  cropFilter.filter(*cloud_filtered);
  
  
  // down sampling with voxelization
  pcl::PCLPointCloud2::Ptr voxel_cloud (new pcl::PCLPointCloud2 ());

  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloud_filtered);
  //sor.setDownsampleAllData(true);
  sor.setLeafSize (0.01f, 0.01f, 0.01f);
  sor.filter (*voxel_cloud);

// Outlier Removal filter
pcl::PointCloud<pcl::PointXYZ>::Ptr inter2 (new pcl::PointCloud<pcl::PointXYZ>);   //inter pointcloud used for conversions

  pcl::PointCloud<pcl::PointXYZ>::Ptr inter(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(*voxel_cloud,*inter);
//pcl_conversions::fromPCL(*voxel_cloud, inter);

pcl::StatisticalOutlierRemoval<pcl::PointXYZ> filter;
filter.setInputCloud(inter);
filter.setMeanK(50);
filter.setStddevMulThresh (1.0);
filter.filter(*inter2);


//conversions to sensor_msgs


pcl::PCLPointCloud2::Ptr inter3 (new pcl::PCLPointCloud2 ());
pcl::toPCLPointCloud2(*inter2, *inter3);
pcl_conversions::fromPCL(*inter3, inter4);

//transforming to world frame

pcl_ros::transformPointCloud("world", inter4, pcl_L, *listenerL);

//pcl::toPCLPointCloud2(*inter3, pcl_L);

//concatenate pointcloud
pcl::concatenatePointCloud(pcl_L,pcl_R,pcl_fusion);



    
  

  // Publish the data.
  //pub1.publish (pcl_L);
  //ROS_INFO_STREAM( "number of points before voxelization" << cloud_filtered->width*(cloud_filtered)->height);
  //ROS_INFO_STREAM( "number of points after voxelization" << pcl_L.width*pcl_L.height);
  pub.publish (pcl_fusion);
}


void cloud_cb2 (const sensor_msgs::PointCloud2ConstPtr& input)
{
  sensor_msgs::PointCloud2 inter4;
  
 pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
 pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
 pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2 ());

 
 // Pointcloud cropping 
 pcl_conversions::toPCL(*input, *cloud);

  pcl::CropBox<pcl::PCLPointCloud2> cropFilter;
  
  //cropFilter.setTransform(trans2);
  cropFilter.setMin(min_workcell);
  cropFilter.setMax(max_workcell);
  cropFilter.setTranslation(t2);
  //cropFilter.setRotation(r2);
  cropFilter.setInputCloud(cloudPtr);
  cropFilter.filter(*cloud_filtered);
  
  
  // down sampling with voxelization

  pcl::PCLPointCloud2::Ptr voxel_cloud (new pcl::PCLPointCloud2 ());

  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloud_filtered);
  //sor.setDownsampleAllData(true);
  sor.setLeafSize (0.01f, 0.01f, 0.01f);
  sor.filter (*voxel_cloud);


// Outlier Removal filter
pcl::PointCloud<pcl::PointXYZ>::Ptr inter2 (new pcl::PointCloud<pcl::PointXYZ>);   //inter pointcloud used for conversions

  pcl::PointCloud<pcl::PointXYZ>::Ptr inter(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(*voxel_cloud,*inter);
//pcl_conversions::fromPCL(*voxel_cloud, inter);

pcl::StatisticalOutlierRemoval<pcl::PointXYZ> filter;
filter.setInputCloud(inter);
filter.setMeanK(50);
filter.setStddevMulThresh (1.0);
filter.filter(*inter2);

//conversions to sensor_msgs

pcl::PCLPointCloud2::Ptr inter3 (new pcl::PCLPointCloud2 ());
pcl::toPCLPointCloud2(*inter2, *inter3);
pcl_conversions::fromPCL(*inter3, inter4);
//transforming to world frame
listenerR->waitForTransform("world", inter4.header.frame_id, ros::Time(0), ros::Duration(5.0));
pcl_ros::transformPointCloud("world", inter4, pcl_R, *listenerR);

//pcl::toPCLPointCloud2(*inter3, pcl_R);

  // Publish the data.
  //pub2.publish (pcl_R);
}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "pointcloud_compute");
  ros::NodeHandle nh;

  
tf::TransformListener lstnrL(ros::Duration(5));
listenerL=&lstnrL;  
tf::TransformListener lstnrR(ros::Duration(5));
listenerR=&lstnrR;
  // Create a ROS subscriber for the input point cloud

  sub1 = nh.subscribe ("/camL/depth/color/points", 1, cloud_cb1); //from camL
  sub2 = nh.subscribe ("/camR/depth/color/points", 1, cloud_cb2);

  // Create a ROS publisher for the output point cloud
  //pub1 = nh.advertise<sensor_msgs::PointCloud2> ("/camL/depth/color/points_computed", 1);
  //pub2 = nh.advertise<sensor_msgs::PointCloud2> ("/camR/depth/color/points_computed", 1);
  pub = nh.advertise<sensor_msgs::PointCloud2> ("/cameras/depth_pointcloud_fusion", 1);
  //listener.lookupTransform("/world", "/camL_link", ros::Time(0), transform);
  // Spin
  ros::spin ();
}

