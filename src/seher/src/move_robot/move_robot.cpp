#include "seher/move_robot/move_robot.h"
#include "tf/transform_datatypes.h"
#include <angles/angles.h>
#include <std_msgs/Header.h>
#include <std_msgs/Int64.h>
#include "std_msgs/Float32.h"
#include "actionlib_msgs/GoalID.h"
#include "std_msgs/Float64.h"
#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <ur_msgs/SetSpeedSliderFraction.h>



//threshold distance between robot and obstacle to stop the robot
float dist_threshold_low=0.2; //20 cm
float dist_threshold_up=0.4; //40 cm
bool near_obstacle=false;
float max_robot_speed = 0.5; //corresponds to % of max robot speed like on the Teach Pendant
float distance; //minimal distance between TCP and obstacle
float speed_distance=0.5; // max distance for adjusting the robot speed

float distance_a = (max_robot_speed-0.1)/(speed_distance-dist_threshold_low);
float distance_b = max_robot_speed-distance_a*speed_distance;


//constructor
MoveRobot::MoveRobot()
{
onTarget=false;
obstacle=false;
status=false; // true if moving, false if stopped
handover_flag=false;
hand_position_current.z=-1.0;
hand_position_old.z=-1.0;
hand_timer=ros::Time::now();
hand_timer_threshold=ros::Duration(3.0);
hand_tolerance=0.05; //5 cm
}

//Destructor
MoveRobot::~MoveRobot() {}

void MoveRobot::initialiseMoveit(ros::NodeHandle nh)
{
  namespace rvt = rviz_visual_tools;
  move_group = new moveit::planning_interface::MoveGroupInterface(GROUP_MANIP);
  joint_model_group = move_group->getCurrentState()->getJointModelGroup(GROUP_MANIP);
  
  visual_tools = new moveit_visual_tools::MoveItVisualTools("base_link");
  visual_tools->deleteAllMarkers();
  visual_tools->loadRemoteControl();
  text_pose.translation().z() = 1.75;
  visual_tools->publishText(text_pose, "move Robot for seher", rvt::WHITE, rvt::XLARGE);
  visual_tools->trigger();
  //planning_scene_diff_publisher = nh.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);

}

bool MoveRobot::comparePoses(geometry_msgs::Pose pose1, geometry_msgs::Pose pose2, double delta_posistion, double delta_orientation)
{

  if (  abs(pose1.position.x-pose2.position.x ) <= delta_posistion
        && abs(pose1.position.y-pose2.position.y ) <= delta_posistion
        && abs(pose1.position.z-pose2.position.z ) <= delta_posistion
        && abs(pose1.orientation.x - pose2.orientation.x) <= delta_orientation
        && abs(pose1.orientation.y - pose2.orientation.y) <= delta_orientation
        && abs(pose1.orientation.z - pose2.orientation.z) <= delta_orientation
        && abs(pose1.orientation.w - pose2.orientation.w) <= delta_orientation
     )
  {
    return true;
  }
  else
  {
    return false;
  }
}

void MoveRobot::sleepSafeFor(double duration)
{
  ros::Time start = ros::Time::now();
  while(ros::Time::now() - start <= ros::Duration(duration))
  {
    ros::spinOnce();
  }

}


void MoveRobot::executeCartesianTrajtoPose(geometry_msgs::Pose target, std::string label)
{
  int trial=0;
  while(trial<5)
  {
    if(moveGroupExecutePlan(getCartesianPathPlanToPose(target, label)))
    {
      return;
    }
    ROS_ERROR_STREAM("Execution to " << label << " trial " << trial++ << " failed. Reattempting");
    //failure_counter_++;
  }
  ROS_ERROR_STREAM("Maxx execution attempts reached, aborting program");
  ros::shutdown();
  exit(-1);
}

bool MoveRobot::moveGroupExecutePlan(moveit::planning_interface::MoveGroupInterface::Plan my_plan)
{

  
  move_group->setStartStateToCurrentState();
  
  return move_group->execute(my_plan)==moveit::planning_interface::MoveItErrorCode::SUCCESS;;
}

bool MoveRobot::moveToTarget(geometry_msgs::Pose target)
{

  move_group->setStartStateToCurrentState();
  move_group->setPoseTarget(target);
  
  if ( move_group->asyncMove()==moveit::planning_interface::MoveItErrorCode::SUCCESS){
      
      ROS_INFO_STREAM("MOVING");
      status=true;
      return true;
  }
  ROS_INFO_STREAM("NOT MOVING");
 

}


moveit::planning_interface::MoveGroupInterface::Plan MoveRobot::getCartesianPathPlanToPose(geometry_msgs::Pose target_pose, std::string display_label, double eef_step, double jump_threshold)
{
  namespace rvt = rviz_visual_tools;
  move_group->setStartStateToCurrentState();
  std::vector<geometry_msgs::Pose> waypoints;
  waypoints.push_back(move_group->getCurrentPose().pose);
  waypoints.push_back(target_pose);

  moveit_msgs::RobotTrajectory trajectory;
  double fraction=0.0;

  int trial = 0;
  while(fraction<0.5 && trial++<5)
  {
    fraction = move_group->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
  }
  if(trial == 5 && fraction<0.5)
  {
    ROS_ERROR_STREAM("Could not compute cartesian path for given waypoints, aborting!!");
    ros::shutdown();
    exit(-1);
  }

  ROS_INFO_STREAM("Visualizing Cartesian Path plan to "  <<  display_label <<" (" << fraction*100  << "% acheived)");

  adjustTrajectoryToFixTimeSequencing(trajectory);

  // Visualize the plan in RViz
  visual_tools->deleteAllMarkers();
  visual_tools->publishText(text_pose, "Cartesian path", rvt::WHITE, rvt::XLARGE);
  for (std::size_t i = 0; i < waypoints.size(); ++i)
    visual_tools->publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);
  visual_tools->trigger();

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  my_plan.trajectory_ = trajectory;
  return my_plan;

}


void MoveRobot::adjustTrajectoryToFixTimeSequencing(moveit_msgs::RobotTrajectory &trajectory)
{

  std::vector<ros::Duration> times_from_start;
  times_from_start.resize(trajectory.joint_trajectory.points.size());

  for(int i=0; i < times_from_start.size() ; i++)
  {
    times_from_start[i]= trajectory.joint_trajectory.points[i].time_from_start;
//    ROS_INFO_STREAM("P: "  << i<< " time_from_start: " << times_from_start[i]);
  }

//  ROS_INFO_STREAM("Size of time_from_start :" << times_from_start.size() << " trajectory points: " << trajectory.joint_trajectory.points.size());


  // Adjust starting from point 2 i.e. index 1
  bool adjusted_flag=false;
  for(int i=1; i< times_from_start.size()-1;i++)
  {
    if(times_from_start[i]==ros::Duration(0))
    {
      ros::Duration prev = times_from_start[i];
      times_from_start[i] = ros::Duration((times_from_start[i-1].toSec()+times_from_start[i+1].toSec())/2.0);
      ROS_WARN_STREAM("Recomputing point " << i << " from " << prev <<  " to: " << times_from_start[i-1] << " + " << times_from_start[i+1] << " = " <<times_from_start[i]);
      adjusted_flag=true;
    }
  }

  if( times_from_start.size()>1 &&  times_from_start[times_from_start.size()-1] == ros::Duration(0))
  {
    ROS_WARN_STREAM("Final point in trajectory has 0 timestamp, incrementing logically");
    times_from_start[times_from_start.size()-1] = times_from_start[times_from_start.size()-2] + ros::Duration(0.1);
    adjusted_flag=true;
  }

  if(adjusted_flag)
  {
    for(int i=0; i< times_from_start.size(); i++)
    {
      trajectory.joint_trajectory.points[i].time_from_start = times_from_start[i];
//      ROS_INFO_STREAM("Recomputed time point " << i << " : " << trajectory.joint_trajectory.points[i].time_from_start );
    }
  }

}


void MoveRobot::updateStatus(){
    actionlib::SimpleActionClient<moveit_msgs::MoveGroupAction>& move_action_client = move_group->getMoveGroupClient();
    auto error_code = move_action_client.getResult()->error_code.val;
    //ROS_INFO_STREAM("UPDATING STATUS");
    if (error_code == moveit::planning_interface::MoveItErrorCode::SUCCESS){
        onTarget=true;
        ROS_INFO_STREAM("MOVING SUCCESSFULL");
    }
    else {
        onTarget=false;
    }

    if (near_obstacle){
        obstacle=true;
        //ROS_INFO_STREAM("OBSTACLE NEAR ROBOT");
    } else {
        obstacle=false;
    }
    ur_msgs::SetSpeedSliderFraction speed;
    float adjusted_speed=distance_a*distance+distance_b;
    speed.request.speed_slider_fraction = std::min(max_robot_speed, adjusted_speed);
    ros::service::call("/ur_hardware_interface/set_speed_slider",speed);

}


void MoveRobot::stopRobot(){
    ROS_INFO_STREAM("Robot Stopping");
    //move_group->stop();
    //actionlib_msgs::GoalID stop;
    //cancel_pub.publish(stop);
    //sleepSafeFor(5);
    std_srvs::Trigger trig;
    ros::service::call("/ur_hardware_interface/dashboard/pause",trig); //to stop the robot
    status=false;
}


void MoveRobot::startRobot(){
  ROS_INFO_STREAM("Robot Starting");
  std_srvs::Trigger trig;
  ros::service::call("/ur_hardware_interface/dashboard/play",trig); //to start the robot
  status=true;
}

bool MoveRobot::getOnTarget(){
    return onTarget;
}

bool MoveRobot::getObstacle(){
    return obstacle;
}

void MoveRobot::setObstacle(bool is_obstacle){
    if (is_obstacle){
        obstacle=true;
    }
    else {
        obstacle=false;
    }
}

bool MoveRobot::getStatus(){
    return status;
}

void distanceCallback (const std_msgs::Float32::ConstPtr& dst){
    distance=dst->data;

    if (distance<=dist_threshold_low){
        near_obstacle=true;
    }
    if (distance>=dist_threshold_up) {
        near_obstacle=false;
    }


}

float MoveRobot::distanceComputing (geometry_msgs::Point point1, geometry_msgs::Point point2){
    float distance;
    
    distance= sqrt(pow(point1.x-point2.x,2)+pow(point1.y-point2.y,2)+pow(point1.z-point2.z,2));
    return distance;
}

void MoveRobot::update_hand_position(tf::StampedTransform transform){
  hand_position_old=hand_position_current;
  geometry_msgs::Point point;
  point.x=transform.getOrigin().x();
  point.y=transform.getOrigin().y();
  point.z=transform.getOrigin().z();
  hand_position_current=point;
}

bool MoveRobot::is_in_the_cell(tf::StampedTransform transform){
  if (transform.getOrigin().x()<WORKCELL_XMAX && transform.getOrigin().x()>WORKCELL_XMIN && transform.getOrigin().y()<WORKCELL_YMAX &&
          transform.getOrigin().y()>WORKCELL_YMIN && transform.getOrigin().z()<WORKCELL_ZMAX && transform.getOrigin().z()>WORKCELL_ZMIN) {
            return true;
          }
  return false;
}

void MoveRobot::update_handover_status(tf::StampedTransform hand_tf){
  if (is_in_the_cell(hand_tf)){
    update_hand_position(hand_tf);
    if (distanceComputing (hand_position_current,hand_position_old)>hand_tolerance) {
      hand_timer=ros::Time::now();
    }
    if (ros::Time::now()-hand_timer>hand_timer_threshold) {
        if(!handover_flag) {
        ROS_INFO_STREAM("handover triggered");
        }
        handover_flag=true;
        
    }
  }
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_robot");
  ros::NodeHandle nh;
  tf::StampedTransform transform_hand;
  tf::TransformListener hand_listener;
  ros::AsyncSpinner spinner(1);
  spinner.start();

/*
  ros::Publisher planning_scene_diff_publisher = nh.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
  ros::Publisher pub_seq = nh.advertise<std_msgs::Header>("/ur_manipulation/sequence",1);
  ros::Publisher pub_fail = nh.advertise<std_msgs::Header>("/ur_manipulation/failure_counter",1);
  */

  MoveRobot robot_obj;
  robot_obj.initialiseMoveit(nh);
  /*seher_obj.printBasicInfo();
  ROS_INFO("---------------------------");
  //seher_obj.addCollissionObjects();
  ROS_INFO("Moving to home pose");
  seher_obj.moveToNamedTarget("home");

  ROS_INFO("Starting PnP");
  ROS_INFO("---------------------------");
  */

 ros::Subscriber distance_sub = nh.subscribe("/distance_calculation/minimal_distance",1, distanceCallback);
 //ros::Publisher speed_pub=nh.advertise<std_msgs::Float64>("/speed_scaling_factor",1);

 

    geometry_msgs::Pose target_pose1;
  target_pose1.position.x = 0.3;
  target_pose1.position.y = 0.4;
  target_pose1.position.z = 0.05;
  geometry_msgs::Quaternion quat_msg;
  tf::quaternionTFToMsg(tf::createQuaternionFromRPY(angles::from_degrees(180),angles::from_degrees(0),angles::from_degrees(0)),quat_msg);
  target_pose1.orientation = quat_msg;

  geometry_msgs::Pose target_pose2 = target_pose1;
  target_pose1.position.x = 0.05;
  int seq = 0;
  bool switcher=true;
  /*std_msgs::Float64 low;
  std_msgs::Float64 fast;
  low.data=0.06;
  fast.data=0.25;*/
  robot_obj.startRobot();
  robot_obj.moveToTarget((switcher)?target_pose1:target_pose2);
  while(ros::ok())
  {
    //ROS_INFO_STREAM("----------------------SEQ " << seq++ << "-------------------------------------");
    
    if (!robot_obj.getOnTarget()){
        if (robot_obj.getObstacle()){
            if (robot_obj.getStatus()){
                robot_obj.stopRobot();
            }
            
        }
        else {
            if (!robot_obj.getStatus()){
                //speed_pub.publish((switcher)?low:fast);
                robot_obj.startRobot();
                //robot_obj.moveToTarget((switcher)?target_pose1:target_pose2);
            }
            else {
                robot_obj.sleepSafeFor(0.01);
            }
        }

    } else {
        switcher = !switcher;
        robot_obj.moveToTarget((switcher)?target_pose1:target_pose2);
    }
    try{
      hand_listener.lookupTransform("/world", "/cam3_link/left_hand",  
                               ros::Time(0), transform_hand);
      robot_obj.update_handover_status(transform_hand);
    }
    catch (tf::TransformException ex){
      //ROS_ERROR("%s",ex.what());
      //ROS_INFO_STREAM("no human skeleton detected");
      //ros::Duration(1.0).sleep();
    }
    robot_obj.updateStatus();
    

    
    ros::spinOnce();
    
    
  }
return 0;
}