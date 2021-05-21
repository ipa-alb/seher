/* This node tests if  handover is triggered and if a tool is hold in the hand.
It publishes the trigger and the direction of the handover as well as the tool grasping pose.
*/

#include <pcl/filters/crop_box.h>
#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>

#include <Eigen/Dense>


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
#include <visualization_msgs/Marker.h>
#include <pcl/common/common.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <vector>
#include <pcl/common/centroid.h>
#include <tf/transform_broadcaster.h>
#include <angles/angles.h>
#include <pcl/common/angles.h>
#include <std_msgs/Bool.h>
#include "seher/move_robot/move_robot.h"
#include <seher_msgs/handover.h>

tf::TransformListener* hand_listener;  

// out box
const Eigen::Vector4f min_box =Eigen::Vector4f(-0.2,-0.2,-0.2,1);
const Eigen::Vector4f max_box =Eigen::Vector4f(0.2,0.2,0.2,1);

const float distance_threshold = 0.15;



ros::Publisher pub;
ros::Publisher handover_pub;

geometry_msgs::Point hand_position_current; //current hand position 
geometry_msgs::Point hand_position_old; //hand position at time t-1. to compare if the hand is static or not

ros::Time hand_timer; //timer to know if the hand is static in the workcell, to trigger the handover
ros::Duration hand_timer_threshold=ros::Duration(3.0); //time after whose tool handover phase is triggered
float hand_tolerance=0.05; // cm


float distanceComputing (Eigen::Vector4f point, Eigen::Matrix<double,4,1> point2){
    //compute the distance between 2 points
    float distance;
    distance= sqrt(pow(point[0]-point2(0,0),2)+pow(point[1]-point2(1,0),2)+pow(point[2]-point2(2,0),2));
    return distance;
}


bool is_in_the_cell(tf::StampedTransform transform){
  //tests if the hand is in the cell
  if (transform.getOrigin().x()<WORKCELL_XMAX && transform.getOrigin().x()>WORKCELL_XMIN && transform.getOrigin().y()<WORKCELL_YMAX &&
          transform.getOrigin().y()>WORKCELL_YMIN && transform.getOrigin().z()<WORKCELL_ZMAX && transform.getOrigin().z()>WORKCELL_ZMIN) {
            return true;
          }
  return false;
}

void update_hand_position(tf::StampedTransform transform){
  //update the hand position
  hand_position_old=hand_position_current;
  geometry_msgs::Point point;
  point.x=transform.getOrigin().x();
  point.y=transform.getOrigin().y();
  point.z=transform.getOrigin().z();
  hand_position_current=point;
}

float distanceComputingPoints (geometry_msgs::Point point1, geometry_msgs::Point point2){
    float distance;
    
    distance= sqrt(pow(point1.x-point2.x,2)+pow(point1.y-point2.y,2)+pow(point1.z-point2.z,2));
    return distance;
}


void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
    //gets the raw pointcloud and crops it around the hand
    tf::StampedTransform transform_hand;
    tf::StampedTransform transform_elbow;
    sensor_msgs::PointCloud2 output;
    seher_msgs::handover handover_data;
    handover_data.direction=true;
    try{
      hand_listener->lookupTransform("/world", "/cam3_link/left_hand",  
                               ros::Time(0), transform_hand);

        Eigen::Vector3f t=Eigen::Vector3f(transform_hand.getOrigin().x(),transform_hand.getOrigin().y(),transform_hand.getOrigin().z());
        Eigen::Quaterniond q;
        tf::quaternionTFToEigen(transform_hand.getRotation(), q);
        Eigen::Vector3d r=q.toRotationMatrix().eulerAngles(2,1,0);
        Eigen::Vector3f r_f=Eigen::Vector3f(r[0],r[1],r[2]);
        pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
        pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
        pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2 ());

        // Pointcloud cropping : crop the area of the hand
        pcl_conversions::toPCL(*input, *cloud);

        pcl::CropBox<pcl::PCLPointCloud2> cropFilter;
        
        //cropFilter.setTransform(trans2);
        cropFilter.setMin(min_box);
        cropFilter.setMax(max_box);
        cropFilter.setTranslation(t);
        cropFilter.setRotation(r_f);
        cropFilter.setInputCloud(cloudPtr);
        cropFilter.filter(*cloud_filtered);

        
        
        hand_listener->lookupTransform("/world", "/cam3_link/left_elbow",  
                               ros::Time(0), transform_elbow);
        Eigen::Vector3f t_elb=Eigen::Vector3f(transform_elbow.getOrigin().x(),transform_elbow.getOrigin().y(),transform_elbow.getOrigin().z());
        Eigen::Quaterniond q_elb;
        tf::quaternionTFToEigen(transform_elbow.getRotation(), q_elb);
        Eigen::Vector3d r_elb=q.toRotationMatrix().eulerAngles(2,1,0);
        Eigen::Vector3f r_f_elb=Eigen::Vector3f(r_elb[0],r_elb[1],r_elb[2]);

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_2 (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_3 (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ> cloud_filtered_pcl;
        pcl::fromPCLPointCloud2(*cloud_filtered,*cloud_filtered_2);

        if (!cloud_filtered_2->empty()){
          //apply a statistical filter to the cloud around the hand
          pcl::StatisticalOutlierRemoval<pcl::PointXYZ> filter;
        filter.setInputCloud(cloud_filtered_2);
        filter.setMeanK(30);
        filter.setStddevMulThresh (1); 
        filter.filter(*cloud_filtered_3);
        } else {
          *cloud_filtered_3=*cloud_filtered_2;
        }
        
        
        const pcl::PointCloud<pcl::PointXYZ> cloud_filtered_pcl_const=*cloud_filtered_3;

        const Eigen::Vector4f elbow = Eigen::Vector4f(t_elb[0],t_elb[1],t_elb[2],1);
        Eigen::Vector4f tool_max_point;

        //get the maximum distance in the cropped cloud from elbow
        pcl::getMaxDistance(cloud_filtered_pcl_const, elbow, tool_max_point);
        //get the centroid of the human cropped pointcloud
        Eigen::Matrix<double,4,1> hand_centroid;
        pcl::compute3DCentroid(cloud_filtered_pcl_const, hand_centroid);
        
        

        if (!isnan(tool_max_point[0]) && distanceComputing(tool_max_point, hand_centroid)> distance_threshold) {
            
          //search for the nearest points from max point to compute the centroid
          // and so to get the tool grasping position
          
          pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
          kdtree.setInputCloud (cloud_filtered_3);
          pcl::PointXYZ searchPoint;
          searchPoint.x=tool_max_point[0];
          searchPoint.y=tool_max_point[1];
          searchPoint.z=tool_max_point[2];
          std::vector<int> pointIdxRadiusSearch;
          std::vector<float> pointRadiusSquaredDistance;
          float radius = 0.03;
          pcl::PointCloud<pcl::PointXYZ>::Ptr max_cloud (new pcl::PointCloud<pcl::PointXYZ>);
          if ( kdtree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )
            {
              
              max_cloud->width = pointIdxRadiusSearch.size ();
              max_cloud->height = 1;
              max_cloud->points.resize (max_cloud->width * max_cloud->height);
              for (std::size_t i = 0; i < pointIdxRadiusSearch.size (); ++i) {
                pcl::PointXYZ point;
                max_cloud->points[i].x=cloud_filtered_3->points[ pointIdxRadiusSearch[i] ].x;
                max_cloud->points[i].y=cloud_filtered_3->points[ pointIdxRadiusSearch[i] ].y;
                max_cloud->points[i].z=cloud_filtered_3->points[ pointIdxRadiusSearch[i] ].z;
                
              }
            }
          const pcl::PointCloud<pcl::PointXYZ> max_cloud_const=*max_cloud;
          Eigen::Matrix<double,4,1> centroid;
          //compute the centroid 
          pcl::compute3DCentroid(max_cloud_const, centroid);


          
          
          //get the angle to grasp the tool
          //compute the tool vector
          const Eigen::Vector3f tool_vector = Eigen::Vector3f(centroid(0,0)-transform_hand.getOrigin().x(), centroid(1,0)-transform_hand.getOrigin().y(), centroid(2,0)-transform_hand.getOrigin().z());
          //unit vectors
          const Eigen::Vector3f x_vector = Eigen::Vector3f(1,0,0);
          const Eigen::Vector3f y_vector = Eigen::Vector3f(0,1,0);
          const Eigen::Vector3f z_vector = Eigen::Vector3f(0,0,1);
          
          //for roll
          
          double R_angle = std::acos((tool_vector-tool_vector.dot(x_vector)*x_vector).normalized().dot(y_vector));

          //for pitch
          
          double P_angle = std::acos((tool_vector-tool_vector.dot(y_vector)*y_vector).normalized().dot(x_vector));

          
          

          static tf::TransformBroadcaster br;
          tf::Transform transform;
          
          transform.setOrigin(tf::Vector3(centroid(0,0), centroid(1,0), centroid(2,0)));
          float roll_command_deg = pcl::rad2deg(R_angle+angles::from_degrees(-180));
          float pitch_command_deg = pcl::rad2deg(-P_angle+angles::from_degrees(90));
          //publish the handover and the tool grasping pose
          transform.setRotation(tf::createQuaternionFromRPY(R_angle+angles::from_degrees(-180), -P_angle+angles::from_degrees(90), 0));
          br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/world", "/tool_grasping_point"));
          handover_data.direction=false;
          
        }
        
        tf::StampedTransform transform_tool;
        //without tool in hand
        handover_data.trigger=false;
        if (is_in_the_cell(transform_hand)){
          update_hand_position(transform_hand);
          
          if (distanceComputingPoints (hand_position_current,hand_position_old)>hand_tolerance) {
            hand_timer=ros::Time::now();
          }
          if (ros::Time::now()-hand_timer>hand_timer_threshold) {
              
          ROS_INFO_STREAM("handover triggered");
          
          handover_data.trigger=true;
        

          }
        }
        handover_pub.publish(handover_data);
        pcl_conversions::fromPCL(*cloud_filtered, output);
    }
    catch (tf::TransformException ex){
      ROS_INFO_STREAM("no hand in cell");
      output.header.frame_id="world";

    }
    
    
    pub.publish(output);
    
}




int main (int argc, char** argv)
{
  hand_position_current.z=-1.0;
  hand_position_old.z=-1.0;
  
  // Initialize ROS
  ros::init (argc, argv, "handover_tool_in_hand_detector");
  ros::NodeHandle nh;
  hand_timer=ros::Time::now();
  tf::TransformListener lstnr(ros::Duration(5));
  hand_listener=&lstnr;  

  // Create a ROS subscriber for the input point cloud

  ros::Subscriber sub = nh.subscribe ("/cameras/raw_depth_pointcloud_fusion", 1, cloud_cb); 
  handover_pub=nh.advertise<seher_msgs::handover>("/handover/handover_data",1);
  

  pub = nh.advertise<sensor_msgs::PointCloud2> ("/handover/hand_pointcloud", 1);
  
  
  ros::spin ();
}
