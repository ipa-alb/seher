/*This node controls the robot start/stop command as well as the speed of the robot, depending on the minimal distance
It subscribes to the minimal distance and directly controls the robot through the appropriate ROS services calls.
*/

#include "tf/transform_datatypes.h"
#include <std_msgs/Header.h>
#include <std_msgs/Int64.h>
#include "std_msgs/Float32.h"
#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <ur_msgs/SetSpeedSliderFraction.h>
#include <std_msgs/Bool.h>
#include "ur_msgs/SetIO.h"
#include <fstream>

using namespace std;
ofstream outfile;
ofstream outfile2;
 int i =0;
 int j=0;
//threshold distance between robot and obstacle to stop the robot
float dist_threshold_low=0.15; //20 cm
float dist_threshold_up=0.3; //40 cm
bool near_obstacle=false;
bool status=false; //true if robot moving, false if robot stopped
float max_robot_speed = 0.5; //corresponds to % of max robot speed like on the Teach Pendant
float min_distance; //minimal distance between TCP and obstacle
float speed_distance=0.5; // max distance for adjusting the robot speed
float adjusted_speed=max_robot_speed;

//coefficients computing which are used to linearly adapt the speed of the robot
float distance_a = (max_robot_speed-0.1)/(speed_distance-dist_threshold_low);
float distance_b = max_robot_speed-distance_a*speed_distance;

//to know if a handover is running.
bool handover_flag=false;


void distanceCallback (const std_msgs::Float32::ConstPtr& dst){
    outfile << i << " : " << ros::Time::now() << endl;
    i++;
    min_distance=dst->data;

    if (min_distance<=dist_threshold_low){
        near_obstacle=true;  //setting flag to true
    }
    if (min_distance>=dist_threshold_up) {
        near_obstacle=false;  //setting flag to false when distance becomes big enough again
    }
}

void handoverCallback (const std_msgs::Bool::ConstPtr& flag){
  if (flag->data){
    handover_flag=true;
    
  }
  else {
    handover_flag=false;
  }
  
}



void startRobot(){
  //starts the robot
  ROS_WARN_STREAM("Robot Starting");
  std_srvs::Trigger trig;
  ros::service::call("/ur_hardware_interface/dashboard/play",trig); //to start the robot
  status=true;
}

void sleepSafeFor(double duration)
{
  ros::Time start = ros::Time::now();
  while(ros::Time::now() - start <= ros::Duration(duration))
  {
    ros::spinOnce();
  }

}

void stopRobot(){
    //stops the robot
    ROS_WARN_STREAM("Robot Stopping");
    std_srvs::Trigger trig;
    ros::service::call("/ur_hardware_interface/dashboard/pause",trig); //to stop the robot
    status=false;
    sleepSafeFor(1.0);
}

void updateSpeed(){
    //computes the new adapted speed of the robot
    ur_msgs::SetSpeedSliderFraction speed;
    adjusted_speed=(adjusted_speed+distance_a*min_distance+distance_b)/2.0;
    speed.request.speed_slider_fraction = std::min(max_robot_speed, adjusted_speed);
    ros::service::call("/ur_hardware_interface/set_speed_slider",speed);

}

void setSpeed(float wanted_speed){
    //sets the speed of the robot
    ur_msgs::SetSpeedSliderFraction speed;
    
    speed.request.speed_slider_fraction = wanted_speed;
    ros::service::call("/ur_hardware_interface/set_speed_slider",speed);

}







int main(int argc, char **argv)
{
  ros::init(argc, argv, "collision_avoidance_robot");
  ros::NodeHandle nh;
  
  
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::Rate r(30);

 ros::Subscriber distance_sub = nh.subscribe("/distance_calculation/minimal_distance",1, distanceCallback);
 ros::Subscriber handover_sub=nh.subscribe("/handover/approach_flag",1, handoverCallback);
 
 
 outfile.open("loop_time_min_distance_cb_with_cloud.dat");
 
    
  while(ros::ok())
  {
    

    //if a handover is running, robot will be near the hand. So, to perform it, speed is strongly reduced to 10% but collision avoidance is disabled.
    if (handover_flag){
      setSpeed(0.1);
      if (!status){
        startRobot();
      }
    } else {
      updateSpeed();
    }
    

    if (near_obstacle && status && !handover_flag){
        stopRobot();
        
    }
    if (!near_obstacle && !status && !handover_flag){
        startRobot();
    }

    
    

    
    ros::spinOnce();
    r.sleep();
    
  }
outfile.close();
//outfile2.close();
return 0;

}