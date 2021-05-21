/* This node sends current time to ROS topics.
It is used to send it to the MSB and then measure the send/receive delay.
*/


#include <ros/ros.h>
#include "std_msgs/Float32.h"
#include "rosgraph_msgs/Clock.h"
#include <fstream>
#include <sensor_msgs/PointCloud2.h>

ros::Duration delay;
ros::Time t_send;
ros::Time t_receive;



ros::Publisher time_pub;






int main(int argc, char** argv){
  ros::init(argc, argv, "MSB_send_time");

  ros::NodeHandle node;


 
  time_pub=node.advertise<rosgraph_msgs::Clock>("/MSB_time/send",1);
  
   
  ros::Rate r(30);  //sending with 30 Hz
  
  
  while (node.ok()){
      rosgraph_msgs::Clock time_send;
      time_send.clock=ros::Time::now();
      time_pub.publish(time_send);
      ros::spinOnce();
      r.sleep();
  }
    

  
  
  return 0;
};