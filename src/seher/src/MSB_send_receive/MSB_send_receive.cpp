/*This node sends and receives data from/to the MSB and writes the delay in a .dat file.
*/


#include <ros/ros.h>
#include "std_msgs/Float32.h"
#include "rosgraph_msgs/Clock.h"
#include <fstream>
#include <sensor_msgs/PointCloud2.h>

ros::Duration delay;
ros::Time t_send;
ros::Time t_receive;

using namespace std;
ofstream outfile;
int i=0;

ros::Publisher distance_send;

void distanceCallback (const sensor_msgs::PointCloud2ConstPtr& input){
    //sending the cloud
    sensor_msgs::PointCloud2 cloud;
    cloud=*input;
    cloud.header.stamp=ros::Time::now();
    distance_send.publish(cloud);

}

void distanceReceiveCallback (const sensor_msgs::PointCloud2ConstPtr& t){
    //receiving the cloud and writting the delay in the file
    t_receive=ros::Time::now();
    delay=t_receive-t->header.stamp;
    outfile << i << " : " << delay << endl;
    i++;


}


int main(int argc, char** argv){
  ros::init(argc, argv, "MSB_send_receive");

  ros::NodeHandle node;


  ros::Subscriber distance_sub = node.subscribe("/cameras/depth_pointcloud_fusion",1, distanceCallback);
  distance_send=node.advertise<sensor_msgs::PointCloud2>("/distance_calculation/minimal_distance/send",1);
  ros::Subscriber distance_get = node.subscribe("/distance_calculation/minimal_distance/receive",1, distanceReceiveCallback);

   
  
  outfile.open("MSB_send_receive.dat");
  
  ros::spin();
    

  outfile.close();
  
  return 0;
};