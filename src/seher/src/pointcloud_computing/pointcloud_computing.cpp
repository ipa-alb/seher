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
//ros::Publisher pub4;


ros::Publisher pub;

ros::Subscriber sub1;
ros::Subscriber sub2;
ros::Subscriber sub3;
ros::Subscriber sub4;


tf::TransformListener* listener1;  
tf::TransformListener* listener2;
tf::TransformListener* listener3;
tf::TransformListener* listener4;

sensor_msgs::PointCloud2 pcl_fusion;
sensor_msgs::PointCloud2 pcl_fusion_final;
sensor_msgs::PointCloud2 pcl_pre_fusion;
sensor_msgs::PointCloud2 pcl_pre_fusion2;
sensor_msgs::PointCloud2 pcl_1;
sensor_msgs::PointCloud2 pcl_2;
sensor_msgs::PointCloud2 pcl_3;
sensor_msgs::PointCloud2 pcl_4;




//workcell 
const Eigen::Vector4f min_workcell =Eigen::Vector4f(0,0,0,1);
const Eigen::Vector4f max_workcell =Eigen::Vector4f(0.96,1.0,1.47,1);

const Eigen::Vector4f max_workcell_final =Eigen::Vector4f(0.86,1.47,1.0,1);

//const Eigen::Vector4f min_workcell =Eigen::Vector4f(0.076f,0.793f,1.625f,1); // ok ?
//const Eigen::Vector4f max_workcell =Eigen::Vector4f(0.294f--0.77f,0.29f,1);


const Eigen::Vector3f t2=Eigen::Vector3f(-0.475f,-0.85f,1.0f);  //OK
const Eigen::Vector3f r2=Eigen::Vector3f(-0.496f,0.767f,0.2055f);

const Eigen::Vector3f t1=Eigen::Vector3f(-0.687f,0.0f,0.625f); //OK
const Eigen::Vector3f r1=Eigen::Vector3f(0.49f,0.767f,-0.2f);

const Eigen::Vector3f t3=Eigen::Vector3f(-0.025f,-0.05f,0.025f); 
const Eigen::Vector3f r3=Eigen::Vector3f(0.95f,0.2f,1.249f);

const Eigen::Vector3f t4=Eigen::Vector3f(-0.462f,-0.75f,1.05f);  //OK
const Eigen::Vector3f r4=Eigen::Vector3f(-0.496f,0.767f,0.2055f);


const Eigen::Vector3f t_final=Eigen::Vector3f(-0.4f,-0.28f,0.05f);


void cloud_cb2 (const sensor_msgs::PointCloud2ConstPtr& input)
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
  cropFilter.setRotation(r2);
  cropFilter.setInputCloud(cloudPtr);
  cropFilter.filter(*cloud_filtered);
  
  //sensor_msgs::PointCloud2 test2;
  //pcl_conversions::fromPCL(*cloud_filtered, test2);

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

/*pcl::StatisticalOutlierRemoval<pcl::PointXYZ> filter;
filter.setInputCloud(inter);
filter.setMeanK(50);
filter.setStddevMulThresh (1.0);
filter.filter(*inter2);*/


//conversions to sensor_msgs


pcl::PCLPointCloud2::Ptr inter3 (new pcl::PCLPointCloud2 ());
pcl::toPCLPointCloud2(*inter, *inter3);
pcl_conversions::fromPCL(*inter3, inter4);

//transforming to world frame

pcl_ros::transformPointCloud("world", inter4, pcl_2, *listener2);



//pcl::toPCLPointCloud2(*inter3, pcl_L);

//concatenate pointcloud
//pcl::concatenatePointCloud(pcl_2,pcl_3,pcl_pre_fusion);
//pcl::concatenatePointCloud(pcl_pre_fusion,pcl_4,pcl_pre_fusion2);
pcl::concatenatePointCloud(pcl_2,pcl_4,pcl_pre_fusion2);
pcl::concatenatePointCloud(pcl_pre_fusion2,pcl_1,pcl_fusion);

pcl::PCLPointCloud2* cloud_final = new pcl::PCLPointCloud2;
 pcl::PCLPointCloud2ConstPtr cloud_finalPtr(cloud_final);
 pcl::PCLPointCloud2::Ptr cloud_final_filtered (new pcl::PCLPointCloud2 ());

 
 // Pointcloud cropping 
 pcl_conversions::toPCL(pcl_fusion, *cloud_final);

  pcl::CropBox<pcl::PCLPointCloud2> cropFilter_world;
  
  //cropFilter.setTransform(trans2);
  cropFilter_world.setMin(min_workcell);
  cropFilter_world.setMax(max_workcell_final);
  cropFilter_world.setTranslation(t_final);
  cropFilter_world.setInputCloud(cloud_finalPtr);
  cropFilter_world.filter(*cloud_final_filtered);

  pcl_conversions::fromPCL(*cloud_final_filtered, pcl_fusion_final);
  // Publish the data.
  //pub2.publish (test2);
  //ROS_INFO_STREAM( "number of points before voxelization" << cloud_filtered->width*(cloud_filtered)->height);
  //ROS_INFO_STREAM( "number of points after voxelization" << pcl_L.width*pcl_L.height);
  pub.publish (pcl_fusion_final);
}


void cloud_cb3 (const sensor_msgs::PointCloud2ConstPtr& input)
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
  cropFilter.setTranslation(t3);
  cropFilter.setRotation(r3);
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

/*pcl::StatisticalOutlierRemoval<pcl::PointXYZ> filter;
filter.setInputCloud(inter);
filter.setMeanK(50);
filter.setStddevMulThresh (1.0);
filter.filter(*inter2);*/

//conversions to sensor_msgs

pcl::PCLPointCloud2::Ptr inter3 (new pcl::PCLPointCloud2 ());
pcl::toPCLPointCloud2(*inter, *inter3);
pcl_conversions::fromPCL(*inter3, inter4);
//transforming to world frame
listener3->waitForTransform("world", inter4.header.frame_id, ros::Time(0), ros::Duration(5.0));
pcl_ros::transformPointCloud("world", inter4, pcl_3, *listener3);

//pcl::toPCLPointCloud2(*inter3, pcl_R);

  // Publish the data.
  //pub2.publish (pcl_R);
}


void cloud_cb4 (const sensor_msgs::PointCloud2ConstPtr& input)
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
  cropFilter.setTranslation(t4);
  cropFilter.setRotation(r4);
  cropFilter.setInputCloud(cloudPtr);
  cropFilter.filter(*cloud_filtered);
  
  //sensor_msgs::PointCloud2 test4;
  //pcl_conversions::fromPCL(*cloud_filtered, test4);
  
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

/*pcl::StatisticalOutlierRemoval<pcl::PointXYZ> filter;
filter.setInputCloud(inter);
filter.setMeanK(50);
filter.setStddevMulThresh (1.0);
filter.filter(*inter2);*/

//conversions to sensor_msgs

pcl::PCLPointCloud2::Ptr inter3 (new pcl::PCLPointCloud2 ());
pcl::toPCLPointCloud2(*inter, *inter3);
pcl_conversions::fromPCL(*inter3, inter4);
//transforming to world frame
listener4->waitForTransform("world", inter4.header.frame_id, ros::Time(0), ros::Duration(5.0));
pcl_ros::transformPointCloud("world", inter4, pcl_4, *listener4);

//pcl::toPCLPointCloud2(*inter3, pcl_R);

  // Publish the data.
  //pub4.publish (test4);
}

void cloud_cb1 (const sensor_msgs::PointCloud2ConstPtr& input)
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
  cropFilter.setTranslation(t1);
  cropFilter.setRotation(r1);
  cropFilter.setInputCloud(cloudPtr);
  cropFilter.filter(*cloud_filtered);
  

  //sensor_msgs::PointCloud2 test1;
  //pcl_conversions::fromPCL(*cloud_filtered, test1);

  
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

/*pcl::StatisticalOutlierRemoval<pcl::PointXYZ> filter;
filter.setInputCloud(inter);
filter.setMeanK(50);
filter.setStddevMulThresh (1.0);
filter.filter(*inter2);*/

//conversions to sensor_msgs

pcl::PCLPointCloud2::Ptr inter3 (new pcl::PCLPointCloud2 ());
pcl::toPCLPointCloud2(*inter, *inter3);
pcl_conversions::fromPCL(*inter3, inter4);
//transforming to world frame
listener1->waitForTransform("world", inter4.header.frame_id, ros::Time(0), ros::Duration(5.0));
pcl_ros::transformPointCloud("world", inter4, pcl_1, *listener1);

//pcl::toPCLPointCloud2(*inter3, pcl_R);

  // Publish the data.
  //pub1.publish (test1);
}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "pointcloud_compute");
  ros::NodeHandle nh;

  
tf::TransformListener lstnr1(ros::Duration(5));
listener1=&lstnr1;  
tf::TransformListener lstnr2(ros::Duration(5));
listener2=&lstnr2;
tf::TransformListener lstnr3(ros::Duration(5));
listener3=&lstnr3;
tf::TransformListener lstnr4(ros::Duration(5));
listener4=&lstnr4;
  // Create a ROS subscriber for the input point cloud

  sub1 = nh.subscribe ("/cam1/depth/color/points", 1, cloud_cb1); 
  sub2 = nh.subscribe ("/cam2/depth/color/points", 1, cloud_cb2);
  sub3= nh.subscribe ("/cam3/depth/color/points", 1, cloud_cb3);
  sub4=nh.subscribe ("/cam4/depth/color/points", 1, cloud_cb4);

  // Create a ROS publisher for the output point cloud
  /*pub1 = nh.advertise<sensor_msgs::PointCloud2> ("/cam1/depth/color/points_computed", 1);
  pub2 = nh.advertise<sensor_msgs::PointCloud2> ("/cam2/depth/color/points_computed", 1);
  pub4 = nh.advertise<sensor_msgs::PointCloud2> ("/cam4/depth/color/points_computed", 1);*/

  pub = nh.advertise<sensor_msgs::PointCloud2> ("/cameras/depth_pointcloud_fusion", 1);
  //listener.lookupTransform("/world", "/camL_link", ros::Time(0), transform);
  // Spin
  ros::spin ();
}

