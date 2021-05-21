#include <pcl/filters/crop_box.h>
#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>

#include <Eigen/Dense>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <iostream>

ros::Publisher pub1;
ros::Publisher pub2;
ros::Publisher pub3;
ros::Publisher pub4;

ros::Subscriber sub1;
ros::Subscriber sub2;
ros::Subscriber sub3;
ros::Subscriber sub4;


//workcell 
const Eigen::Vector4f min_workcell =Eigen::Vector4f(0,0,0,1);
const Eigen::Vector4f max_workcell =Eigen::Vector4f(1.097,0.96,1.47,1);

//for cam1 a etablir
const Eigen::Vector3f t1=Eigen::Vector3f(-0.5f,-0.8f,1.01f);
const Eigen::Vector3f r1=Eigen::Vector3f(-0.96f,0.23f,1.25f);

// for cam 2, a verifier
const Eigen::Vector3f t2=Eigen::Vector3f(-0.5f,-0.8f,1.01f);
const Eigen::Vector3f r2=Eigen::Vector3f(-0.96f,0.23f,1.25f);

//for cam3 a etablir
const Eigen::Vector3f t3=Eigen::Vector3f(-0.5f,-0.8f,1.01f);
const Eigen::Vector3f r3=Eigen::Vector3f(-0.96f,0.23f,1.25f);
//for cam4 a etablir
const Eigen::Vector3f t4=Eigen::Vector3f(-0.5f,-0.8f,1.01f);
const Eigen::Vector3f r4=Eigen::Vector3f(-0.96f,0.23f,1.25f);

void cloud_cb2 (const sensor_msgs::PointCloud2ConstPtr& input)
{
  // Create a container for the data.
  sensor_msgs::PointCloud2 output;
  
 pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
 pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
 pcl::PCLPointCloud2 cloud_filtered;

 // Do data processing here...
 pcl_conversions::toPCL(*input, *cloud);

  

  
  pcl::CropBox<pcl::PCLPointCloud2> cropFilter;
  
  //cropFilter.setTransform(trans2);
  cropFilter.setMin(min_workcell);
  cropFilter.setMax(max_workcell);
  cropFilter.setTranslation(t2);
  cropFilter.setRotation(r2);
  cropFilter.setInputCloud(cloudPtr);
  cropFilter.filter(cloud_filtered);
  

  pcl_conversions::fromPCL(cloud_filtered, output);
  // Publish the data.
  pub2.publish (output);
}

void cloud_cb1 (const sensor_msgs::PointCloud2ConstPtr& input)
{
  // Create a container for the data.
  sensor_msgs::PointCloud2 output;
  
 pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
 pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
 pcl::PCLPointCloud2 cloud_filtered;

 // Do data processing here...
 pcl_conversions::toPCL(*input, *cloud);

  

  
  pcl::CropBox<pcl::PCLPointCloud2> cropFilter;
  
  //cropFilter.setTransform(trans2);
  cropFilter.setMin(min_workcell);
  cropFilter.setMax(max_workcell);
  cropFilter.setTranslation(t1);
  cropFilter.setRotation(r1);
  cropFilter.setInputCloud(cloudPtr);
  cropFilter.filter(cloud_filtered);
  

  pcl_conversions::fromPCL(cloud_filtered, output);
  // Publish the data.
  pub1.publish (output);
}

void cloud_cb3 (const sensor_msgs::PointCloud2ConstPtr& input)
{
  // Create a container for the data.
  sensor_msgs::PointCloud2 output;
  
 pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
 pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
 pcl::PCLPointCloud2 cloud_filtered;

 // Do data processing here...
 pcl_conversions::toPCL(*input, *cloud);

  

  
  pcl::CropBox<pcl::PCLPointCloud2> cropFilter;
  
  //cropFilter.setTransform(trans2);
  cropFilter.setMin(min_workcell);
  cropFilter.setMax(max_workcell);
  cropFilter.setTranslation(t3);
  cropFilter.setRotation(r3);
  cropFilter.setInputCloud(cloudPtr);
  cropFilter.filter(cloud_filtered);
  

  pcl_conversions::fromPCL(cloud_filtered, output);
  // Publish the data.
  pub3.publish (output);
}

void cloud_cb4 (const sensor_msgs::PointCloud2ConstPtr& input)
{
  // Create a container for the data.
  sensor_msgs::PointCloud2 output;
  
 pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
 pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
 pcl::PCLPointCloud2 cloud_filtered;

 // Do data processing here...
 pcl_conversions::toPCL(*input, *cloud);

  

  
  pcl::CropBox<pcl::PCLPointCloud2> cropFilter;
  
  //cropFilter.setTransform(trans2);
  cropFilter.setMin(min_workcell);
  cropFilter.setMax(max_workcell);
  cropFilter.setTranslation(t4);
  cropFilter.setRotation(r4);
  cropFilter.setInputCloud(cloudPtr);
  cropFilter.filter(cloud_filtered);
  

  pcl_conversions::fromPCL(cloud_filtered, output);
  // Publish the data.
  pub4.publish (output);
}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;

  //std::cout << "min" << min_workcell2;
//std::cout << "max" << max_workcell2;

  // Create a ROS subscriber for the input point cloud
  sub2 = nh.subscribe ("/cam2/depth/color/points", 1, cloud_cb2);
  // Create a ROS publisher for the output point cloud
  pub2 = nh.advertise<sensor_msgs::PointCloud2> ("/cam2/depth/color/points_filtered", 1);

  sub1 = nh.subscribe ("/cam1/depth/color/points", 1, cloud_cb1);
  // Create a ROS publisher for the output point cloud
  pub1 = nh.advertise<sensor_msgs::PointCloud2> ("/cam1/depth/color/points_filtered", 1);

  sub3 = nh.subscribe ("/cam3/depth/color/points", 1, cloud_cb3);
  // Create a ROS publisher for the output point cloud
  pub3 = nh.advertise<sensor_msgs::PointCloud2> ("/cam3/depth/color/points_filtered", 1);

  sub4 = nh.subscribe ("/cam4/depth/color/points", 1, cloud_cb4);
  // Create a ROS publisher for the output point cloud
  pub4 = nh.advertise<sensor_msgs::PointCloud2> ("/cam4/depth/color/points_filtered", 1);

  // Spin
  ros::spin ();
}

