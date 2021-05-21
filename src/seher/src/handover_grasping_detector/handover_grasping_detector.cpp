/* This node checks and confirms that the tool is grasped by tool/hand, based on the handover flag
It takes as input the raw adequate pointcloud
*/

#include <pcl/filters/crop_box.h>
#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>

#include <Eigen/Dense>



#include "tf/LinearMath/Scalar.h"
#include "tf/LinearMath/Quaternion.h"
#include "tf/LinearMath/Matrix3x3.h"
#include "tf/transform_datatypes.h"
#include <Eigen/Geometry>
#include "Eigen/Core"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <iostream>
#include "tf/transform_listener.h"
#include "tf_conversions/tf_eigen.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/segmentation/extract_clusters.h>
#include <std_msgs/Bool.h>
#include <pcl/common/centroid.h>

tf::TransformListener* TCP_listener;  


//thresholds for the tests --> adjusted by test
const int cluster_points_threshold=250;
const float max_distance_threshold=0.05;
const ros::Duration grasp_timer_threshold=ros::Duration(2.0);

ros::Publisher pub;
ros::Publisher pub_grasp;

bool approach_flag=false;  //flag to know when robot approaches the tool 
bool handover_dir_flag=true; // if true handover from robot to human, else handover from human to robot

ros::Time grasp_timer;


float distanceComputing (tf::StampedTransform TCP1, tf::StampedTransform TCP2){
    //computes the distance between the tfs.
    float distance,x,y,z,x2,y2,z2;
    x=TCP1.getOrigin().x();
    y=TCP1.getOrigin().y();
    z=TCP1.getOrigin().z();
    x2=TCP2.getOrigin().x();
    y2=TCP2.getOrigin().y();
    z2=TCP2.getOrigin().z();
    distance= sqrt(pow(x2-x,2)+pow(y2-y,2)+pow(z2-z,2));
    return distance;
}

float distanceComputing_points (pcl::PointXYZ point, tf::StampedTransform TCP){
    float distance,x,y,z;
    x=TCP.getOrigin().x();
    y=TCP.getOrigin().y();
    z=TCP.getOrigin().z();
    distance= sqrt(pow(point.x-x,2)+pow(point.y-y,2)+pow(point.z-z,2));
    return distance;
}

void handoverDirCallback (const std_msgs::Bool::ConstPtr& flag){
  if (flag->data){
    handover_dir_flag=true;
    
  }
  else {
    handover_dir_flag=false;
  }
  
}

void handoverCallback (const std_msgs::Bool::ConstPtr& flag){
  if (flag->data){
    approach_flag=true;
    
  }
  else {
    approach_flag=false;
  }
  
}

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
    //if robot approaches the tool, tests are done
    if (approach_flag) {
    
    sensor_msgs::PointCloud2 output;
    ROS_INFO_STREAM("looking for grapsing");
    tf::StampedTransform transform_TCP;
    tf::StampedTransform transform_hand;
    
    try{
        TCP_listener->lookupTransform("/world", "/egp_50_tip",  
                                ros::Time(0), transform_TCP);
      }
      catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
      }
      try{
        TCP_listener->lookupTransform("/world", "/cam3_link/left_hand",  
                                ros::Time(0), transform_hand);
      }
      catch (tf::TransformException ex){
        //ROS_WARN_STREAM("no hand detected");
        
      }
     //get the TCP pose to crop the cloud correctly
     Eigen::Vector3f t=Eigen::Vector3f(transform_TCP.getOrigin().x(),transform_TCP.getOrigin().y(),transform_TCP.getOrigin().z());
     tfScalar roll,pitch,yaw;
     const tf::Quaternion q=transform_TCP.getRotation();
     tf::Matrix3x3(q).getRPY(roll,pitch,yaw);
     Eigen::Vector3f r_f=Eigen::Vector3f(roll,pitch,yaw);
     
      pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
      pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
      pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2 ());

      // Pointcloud cropping : crop the area of the TCP
      pcl_conversions::toPCL(*input, *cloud);

      pcl::CropBox<pcl::PCLPointCloud2> cropFilter;


      if (!handover_dir_flag){
      ROS_INFO_STREAM("human to robot handover");

      //box around TCP
      Eigen::Vector4f min_box =Eigen::Vector4f(-0.03,-0.03,-0.01,1); 
      Eigen::Vector4f max_box =Eigen::Vector4f(0.03,0.03,0.05,1);
      //flags to determine if it is grasped or not
      bool cluster_flag=false;
      
      bool max_distance_flag=false;
      bool on_tool_flag=false;

      
      //crop      
      cropFilter.setMin(min_box);
      cropFilter.setMax(max_box);
      cropFilter.setTranslation(t);
      cropFilter.setRotation(r_f);
      cropFilter.setInputCloud(cloudPtr);
      cropFilter.filter(*cloud_filtered);
      pcl_conversions::fromPCL(*cloud_filtered, output);
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_2 (new pcl::PointCloud<pcl::PointXYZ>);
      pcl::fromPCLPointCloud2(*cloud_filtered,*cloud_filtered_2);

      pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
      tree->setInputCloud (cloud_filtered_2);
    float max=0;
    int nb_points=0;
    for (int pit = 0; pit < cloud_filtered_2->size(); ++pit){
      float dist =distanceComputing_points(cloud_filtered_2->at(pit), transform_TCP);
      if (dist > max ){
        max=dist;
      }
    }

    if (max < max_distance_threshold){
      max_distance_flag=true;
    }

    //clustering of the pointcloud to know if the tool is in contact with the gripper
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (0.004); 
    ec.setMinClusterSize (30);
    ec.setMaxClusterSize (2000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud_filtered_2);
    ec.extract (cluster_indices);

   
    int j = 0;
    
    sensor_msgs::PointCloud2 output; 
    
    if (cluster_indices.size()==1 && cluster_indices[0].indices.size()>=cluster_points_threshold ){
      
      cluster_flag=true;
    }
    
    
    std_msgs::Bool flag; 
    flag.data=false;
    if (cluster_flag && max_distance_flag){
      on_tool_flag=true;
      
    }
    if (!on_tool_flag){
      grasp_timer=ros::Time::now();
    }
    // tests must be successfull for 2 secs
    if (ros::Time::now()-grasp_timer>grasp_timer_threshold) {
      flag.data=true;
    }
    pub_grasp.publish(flag);
    //ROS_INFO_STREAM("General Flag: " << on_tool_flag);
      //pub.publish(output);

      
    } else {
      // determine if hand is on tool in order to open gripper
      ROS_INFO_STREAM("robot to human handover");
      bool distance_hand_TCP=true;  //not used
      bool distance_centroid=false;
      bool nb_points=false;
      bool hand_on_tool_flag=false;
      Eigen::Vector4f min_box =Eigen::Vector4f(-0.03,-0.2,-0.05,1); 
      Eigen::Vector4f max_box =Eigen::Vector4f(0.03,0.2,0.05,1);  
      
      //thresholds
      int nb_points_threshold = 750;
      float dist_hand_TCP_threshold=0.25;
      float dist_centroid_TCP_threshold=0.06;

      //cropping
      cropFilter.setMin(min_box);
      cropFilter.setMax(max_box);
      cropFilter.setTranslation(t);
      cropFilter.setRotation(r_f);
      cropFilter.setInputCloud(cloudPtr);
      cropFilter.filter(*cloud_filtered);
      pcl_conversions::fromPCL(*cloud_filtered, output);
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_2 (new pcl::PointCloud<pcl::PointXYZ>);
      pcl::fromPCLPointCloud2(*cloud_filtered,*cloud_filtered_2);
      const pcl::PointCloud<pcl::PointXYZ> cloud_filtered_pcl_const=*cloud_filtered_2;
      
      if (distanceComputing(transform_hand, transform_TCP) < dist_hand_TCP_threshold){
        distance_hand_TCP=true;
      }
      
      //control the number of points
      if (cloud_filtered_2->size() > nb_points_threshold){
        nb_points=true;
      }
      ROS_INFO_STREAM("Number of points : " << cloud_filtered_2->size());

      Eigen::Matrix<double,4,1> centroid;
      pcl::compute3DCentroid(cloud_filtered_pcl_const, centroid);
      pcl::PointXYZ point_centroid;
      point_centroid.x=centroid(0,0);
      point_centroid.y=centroid(1,0);
      point_centroid.z=centroid(2,0);

      ROS_INFO_STREAM("distance centroid to TCP : " << distanceComputing_points(point_centroid, transform_TCP));
      //control the distance between TCP and the centroid point
      if (distanceComputing_points(point_centroid, transform_TCP)>dist_centroid_TCP_threshold){
        distance_centroid=true;
      }

      hand_on_tool_flag=distance_hand_TCP && distance_centroid && nb_points;
      std_msgs::Bool flag; 
      flag.data=false;
      //tests must be successful for 2 secs
      if (!hand_on_tool_flag){
        grasp_timer=ros::Time::now();
      }
      if (ros::Time::now()-grasp_timer>grasp_timer_threshold) {
        flag.data=true;
        
      }
      pub_grasp.publish(flag);
      
            
    }
    pub.publish(output);
  }  
    
}




int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "handover_grasping_detector");
  ros::NodeHandle nh;

  tf::TransformListener lstnr(ros::Duration(5));
  TCP_listener=&lstnr;  

  // Create a ROS subscriber for the input point cloud

  ros::Subscriber sub = nh.subscribe ("/cameras/raw_depth_pointcloud_fusion", 1, cloud_cb); 
  
  ros::Subscriber handover_sub=nh.subscribe("/handover/approach_flag",1, handoverCallback);
  ros::Subscriber handover_direction_sub=nh.subscribe("/handover/direction",1, handoverDirCallback);
  
  

  pub = nh.advertise<sensor_msgs::PointCloud2> ("/handover/grasp_pointcloud", 1);
  pub_grasp = nh.advertise<std_msgs::Bool> ("/handover/grasp_flag", 1);
  
  ros::spin ();
}
