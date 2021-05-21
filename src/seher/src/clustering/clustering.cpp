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



ros::Publisher pub;

ros::Subscriber sub;

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input){

pcl::PCLPointCloud2::Ptr pcl_fusion_pc (new pcl::PCLPointCloud2 ());
pcl_conversions::toPCL(*input, *pcl_fusion_pc);


pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_to_cluster(new pcl::PointCloud<pcl::PointXYZ>);
pcl::fromPCLPointCloud2(*pcl_fusion_pc,*pcl_to_cluster);
//cluster extraction

pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (pcl_to_cluster);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (0.04); // 2cm
  ec.setMinClusterSize (100);
  ec.setMaxClusterSize (25000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (pcl_to_cluster);
  ec.extract (cluster_indices);

  std::vector<sensor_msgs::PointCloud2::Ptr> pc2_clusters;
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr > clusters;

  int j = 0;
   
  sensor_msgs::PointCloud2 output; 

  if (!cluster_indices.empty()) {
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
      cloud_cluster->points.push_back (pcl_to_cluster->points[*pit]); //*
    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    //std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;

    clusters.push_back(cloud_cluster);
    sensor_msgs::PointCloud2::Ptr tempROSMsg(new sensor_msgs::PointCloud2);
    pcl::toROSMsg(*cloud_cluster, *tempROSMsg);
    tempROSMsg->header.frame_id="world";
    pc2_clusters.push_back(tempROSMsg);
    output=*(pc2_clusters.at(0));
  }
  
  }
  else{
      output.header.frame_id="world";
      output.data.clear();
      output.is_dense=true;
  }

    pub.publish(output);

  }


    int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "clustering");
  ros::NodeHandle nh;

  
  // Create a ROS subscriber for the input point cloud
  sub = nh.subscribe ("/cameras/depth_pointcloud_fusion", 1, cloud_cb);
  
  // Create a ROS publisher for the output point cloud
  //pub1 = nh.advertise<sensor_msgs::PointCloud2> ("/camL/depth/color/points_computed", 1);
  //pub2 = nh.advertise<sensor_msgs::PointCloud2> ("/camR/depth/color/points_computed", 1);
  pub = nh.advertise<sensor_msgs::PointCloud2> ("/cameras/depth_pointcloud_fusion_clustered", 1);
  //listener.lookupTransform("/world", "/camL_link", ros::Time(0), transform);
  // Spin
  ros::spin ();
}