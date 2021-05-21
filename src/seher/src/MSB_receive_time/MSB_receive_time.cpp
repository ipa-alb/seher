/*This node subscribes to a time topic pubished by the MSB.
It is used to measure the send/receive delay from the MSB.
Getting time is in each loop written in a .dat file.
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





void timeCallback (const rosgraph_msgs::ClockConstPtr& t){
    t_receive=ros::Time::now();
    delay=t_receive-t->clock;
    outfile << i << " : " << delay << endl;  //write each time in a file
    i++;


}


int main(int argc, char** argv){
  ros::init(argc, argv, "MSB_receive_time");

  ros::NodeHandle node;


 
  
  ros::Subscriber time_get = node.subscribe("/MSB_time/receive",1, timeCallback);

   
  
  outfile.open("MSB_send_receive.dat");
  
  ros::spin();
    

  outfile.close();
  
  return 0;
};