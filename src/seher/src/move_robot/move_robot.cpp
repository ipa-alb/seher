/* This node is the state machine of the robot. It performs the nominal task and the handover when it is triggered.
Trajectory are planned thanks to MoveIT planner.
*/

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
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>




ros::Publisher handover_pub;
ros::Publisher handover_dir_pub;
ros::Publisher state_pub;

bool grasp_flag=false;   //grasping flag
bool handover_flag=false; //handover flag
bool handover_direction=false;  //handover direction

tf::TransformListener* hand_listener;  

void graspCallback (const std_msgs::Bool::ConstPtr& flag){
  //to know if the grasping is confirmed
  if (flag->data){
    grasp_flag=true;
    
  }
  else {
    grasp_flag=false;
  }
  
} 

void handoverDataCallback (const seher_msgs::handover::ConstPtr& data){
  //to know if handover has been triggered
  if (data->trigger){
    handover_flag=true;

    if (data->direction){
      
      handover_direction=true;
      } else {
        handover_direction=false;
        }
 
}
}




//constructor
MoveRobot::MoveRobot()
{

_status = nominal_task;

}

//Destructor
MoveRobot::~MoveRobot() {}

void MoveRobot::initialiseMoveit(ros::NodeHandle nh)
{
  //initialize
  namespace rvt = rviz_visual_tools;
  move_group = new moveit::planning_interface::MoveGroupInterface(GROUP_MANIP);
  joint_model_group = move_group->getCurrentState()->getJointModelGroup(GROUP_MANIP);
  move_group->setPlannerId("RRTConnectkConfigDefault");

  visual_tools = new moveit_visual_tools::MoveItVisualTools("base_link");
  visual_tools->deleteAllMarkers();
  visual_tools->loadRemoteControl();
  text_pose.translation().z() = 1.75;
  visual_tools->publishText(text_pose, "move Robot for seher", rvt::WHITE, rvt::XLARGE);
  visual_tools->trigger();
  //planning_scene_diff_publisher = nh.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
  n=nh;
  //tool place position in the cell
  tool_place.position.x=-0.2;
  tool_place.position.y=0.6;
  tool_place.position.z=-0.015;
  geometry_msgs::Quaternion quat_msg;
  tf::quaternionTFToMsg(tf::createQuaternionFromRPY(angles::from_degrees(180),angles::from_degrees(0),angles::from_degrees(0)),quat_msg);
  tool_place.orientation = quat_msg;
  planning_scene_diff_publisher = nh.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
  handover_dir_pub=nh.advertise<std_msgs::Bool>("/handover/direction", 1);
  timer=ros::Time::now();
  timeout=ros::Duration(30);

}

bool MoveRobot::comparePoses(geometry_msgs::Pose pose1, geometry_msgs::Pose pose2, double delta_posistion, double delta_orientation)
{
  //to compare if 2 poses are same

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


bool MoveRobot::executeCartesianTrajtoPose(geometry_msgs::Pose target)
{
  int trial=0;
  while(trial<20)
  {
    if(moveGroupExecutePlan(getCartesianPathPlanToPose(target)))
    {
      return true;
    }
    //ROS_ERROR_STREAM("Execution to " << label << " trial " << trial++ << " failed. Reattempting");
    //failure_counter_++;
    trial++;
  }
  ROS_ERROR_STREAM("Maxx execution attempts reached, error");
  
  sleepSafeFor(1.0);
  
  return false;
}

bool MoveRobot::moveGroupExecutePlan(moveit::planning_interface::MoveGroupInterface::Plan my_plan)
{

  
  move_group->setStartStateToCurrentState();
  
  return move_group->execute(my_plan)==moveit::planning_interface::MoveItErrorCode::SUCCESS;
}

bool MoveRobot::moveToTarget(geometry_msgs::Pose target)
{
  namespace rvt = rviz_visual_tools;
  bool plan_success = false;
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  

  move_group->setStartStateToCurrentState();
  move_group->setPoseTarget(target);

  plan_success = (move_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  
  visual_tools->publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools->publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools->trigger();
  
  if (plan_success){
    if ( move_group->move()==moveit::planning_interface::MoveItErrorCode::SUCCESS){
        
        ROS_INFO_STREAM("MOVING");
        
        return true;
    }
    ROS_INFO_STREAM("NOT MOVING");
    
  } else {
    ROS_WARN_STREAM("Failed planning");
  }
  return false;
}


moveit::planning_interface::MoveGroupInterface::Plan MoveRobot::getCartesianPathPlanToPose(geometry_msgs::Pose target_pose, double eef_step, double jump_threshold)
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

  //ROS_INFO_STREAM("Visualizing Cartesian Path plan to "  <<  display_label <<" (" << fraction*100  << "% acheived)");

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
  // update the status of the state machine, handles the state transitions
  if (_status==handover_hand_pick || _status==handover_hand_place) {
    geometry_msgs::Pose current_pose = move_group->getCurrentPose().pose;
    
    
    if( comparePoses(current_pose, hand_target, 0.05)) {
          
          ROS_WARN_STREAM("On hand position");
          sleepSafeFor(0.5);
          if (_status==handover_hand_pick){
            _status=handover_tool_pick;
            timer=ros::Time::now();  //timer is re initialized for timeout control
          } else {
            
            _status=handover_tool_place;
            timer=ros::Time::now();
          }
          

          
          
          
        } 
  
  }
  if (_status==handover_tool_pick){
    geometry_msgs::Pose current_pose = move_group->getCurrentPose().pose;
    
    
    if( comparePoses(current_pose, tool_target, 0.05)) {
          //_status=nominal_task;
          ROS_WARN_STREAM("On tool position for picking");
          sleepSafeFor(2.0);
          if (grasp_flag){
            if (gripperClose()){
              _status=place_tool;
              timer=ros::Time::now();
              ROS_WARN_STREAM("close_gripper");
              addRemoveToolObject(true);
            } else {
              ROS_WARN_STREAM("Gripper close failed");
            }

          } else {
            ROS_WARN_STREAM("Tool not in grapser");
          }
            
          
          
        } else {
          ROS_WARN_STREAM("Current pose is not tool target");
        }
  }

  if (_status==handover_tool_place){
    geometry_msgs::Pose current_pose = move_group->getCurrentPose().pose;
    
    
    if( comparePoses(current_pose, tool_target, 0.05)) {
          //_status=nominal_task;
          ROS_WARN_STREAM("On tool position for giving");
          sleepSafeFor(2.0);
          if (grasp_flag){
            if (gripperOpen()){
              _status=nominal_task;
              addRemoveToolObject(false); //remove tool from planner
              timer=ros::Time::now();
            }
          }
    }
  }

  

  if (_status==pick_tool){
    _status=handover_hand_place;
    timer=ros::Time::now();
  }
  //control if there is a timeout
  if (ros::Time::now()-timer > timeout && _status != nominal_task){
    timeOut();
  }

}

void MoveRobot::timeOut(){
  //to handle the timeout action
  ROS_WARN_STREAM("Blocked in state machine, timeout, going to nominal task");
  switch(_status){
    case handover_hand_place :
      _status=place_tool;
      break;
    
    case handover_tool_place :
      _status=place_tool;
      break;
    
    case handover_hand_pick :
      _status=nominal_task;
      break;
    
    case handover_tool_pick :
      _status=nominal_task;
    break;
  }
}

bool MoveRobot::gripperOpen()
{
  //open the gripper
  ur_msgs::SetIO io_msg;
  io_msg.request.fun = static_cast<int8_t>(IO_SERVICE_FUN_LEVEL_);
  io_msg.request.pin = static_cast<int8_t>(1);  //Pin 1 is open
  io_msg.request.state = 1;
  ros::ServiceClient client = n.serviceClient<ur_msgs::SetIO>("/ur_hardware_interface/set_io");

  if(client.call(io_msg))
  {
    ROS_INFO_STREAM("Open gripper initialise : " << ((io_msg.response.success==0)?"Failed":"Succeeded") );
    sleepSafeFor(0.5);
    io_msg.request.state = 0;
    if(client.call(io_msg))
    {
      ROS_INFO_STREAM("Open gripper conclude : " << ((io_msg.response.success==0)?"Failed":"Succeeded") );
      return true;
    }
    else
    {
      ROS_INFO_STREAM("Open gripper conclude : Failed");
      return false;
    }
  }
  else
  {
    ROS_INFO_STREAM("Open gripper initialise : Failed");
    return false;
  }
}

bool MoveRobot::gripperClose()
{
  //to close the gripper
  ur_msgs::SetIO io_msg;
  io_msg.request.fun = static_cast<int8_t>(IO_SERVICE_FUN_LEVEL_);
  io_msg.request.pin = static_cast<int8_t>(0);    //Pin 0 is close
  io_msg.request.state = 1;
  ros::ServiceClient client = n.serviceClient<ur_msgs::SetIO>("/ur_hardware_interface/set_io");

  if(client.call(io_msg))
  {
    ROS_INFO_STREAM("Close gripper initialise :  " << ((io_msg.response.success==0)?"Failed":"Succeeded") );
    sleepSafeFor(0.5);
    io_msg.request.state = 0;
    if(client.call(io_msg))
    {
      ROS_INFO_STREAM("Close gripper conclude :  " << ((io_msg.response.success==0)?"Failed":"Succeeded") );
      return true;
    }
    else
    {
      ROS_INFO_STREAM("Close gripper conclude : Failed");
      return false;
    }
  }
  else
  {
    ROS_INFO_STREAM("Close gripper initialise : Failed");
    return false;
  }
}




float MoveRobot::distanceComputing (geometry_msgs::Point point1, geometry_msgs::Point point2){
    //computes the distance between 2 points
    float distance;
    
    distance= sqrt(pow(point1.x-point2.x,2)+pow(point1.y-point2.y,2)+pow(point1.z-point2.z,2));
    return distance;
}

void MoveRobot::update_hand_position(tf::StampedTransform transform){
  //update the hand position
  geometry_msgs::Point point;
  point.x=transform.getOrigin().x();
  point.y=transform.getOrigin().y();
  point.z=transform.getOrigin().z();
  hand_position_current=point;
}

bool MoveRobot::is_in_the_cell(tf::StampedTransform transform){
  //controls if the position of the hand is in the cell or not
  if (transform.getOrigin().x()<WORKCELL_XMAX && transform.getOrigin().x()>WORKCELL_XMIN && transform.getOrigin().y()<WORKCELL_YMAX &&
          transform.getOrigin().y()>WORKCELL_YMIN && transform.getOrigin().z()<WORKCELL_ZMAX && transform.getOrigin().z()>WORKCELL_ZMIN) {
            return true;
          }
  return false;
}

void MoveRobot::update_handover_status(){
  // update the state of the handover and the hand/tool positions
  if (handover_flag){
  handover_flag=false;
  
  tf::StampedTransform transform_hand;
  tf::StampedTransform transform_tool;
  
  try{
    hand_listener->lookupTransform("/world", "/cam3_link/left_hand",  
                              ros::Time(0), transform_hand);
            

    
  }
  catch (tf::TransformException ex){
    ROS_WARN_STREAM("no hand in cell whereas handover triggered");
    return;
  }
  try {
    hand_listener->lookupTransform("/world", "/tool_grasping_point",  
                              ros::Time(0), transform_tool);
    
    
  }
  catch (tf::TransformException ex) {
    
    
  }
  if (!is_in_the_cell(transform_hand)){
    return;
  }
  update_hand_position(transform_hand);
  computePoseToHand();
 
  if(_status==nominal_task) {
  ROS_WARN_STREAM("handover triggered");
  std_msgs::Bool flag;

  if (handover_direction) {
    _status=pick_tool;
    timer=ros::Time::now();

    ROS_WARN_STREAM("no tool in hand, so robot to human handover");
    flag.data=true;
    
    computeGivePose(transform_hand);
  } else {
    _status=handover_hand_pick;
    timer=ros::Time::now();
    flag.data=false;
    ROS_WARN_STREAM("tool in hand, so human to robot handover");
    computePoseToTool(transform_tool);
  }
  handover_dir_pub.publish(flag);
  
  }
      
  
  }
}


enum status MoveRobot::getStatus(){
  return _status;
}



void MoveRobot::computePoseToHand(){
  // computes the approach position of the hand before handover
  geometry_msgs::Pose pose;
  pose.position=hand_position_current;
  pose.position.y=pose.position.y-0.15;
  geometry_msgs::Quaternion quat_msg;
  tf::quaternionTFToMsg(tf::createQuaternionFromRPY(angles::from_degrees(-90),angles::from_degrees(0),angles::from_degrees(0)),quat_msg);
  pose.orientation=quat_msg;
  hand_target=pose;
}

void MoveRobot::computePoseToTool(tf::StampedTransform tool_tf){
  // gets the tool position and orientation for picking in hand
  geometry_msgs::Pose pose;
  pose.position.x=tool_tf.getOrigin().x();
  pose.position.y=tool_tf.getOrigin().y();
  pose.position.z=tool_tf.getOrigin().z();
  geometry_msgs::Quaternion quat_msg;
  tf::quaternionTFToMsg(tool_tf.getRotation(),quat_msg);
  
  pose.orientation=quat_msg;
  tool_target=pose;
}

void MoveRobot::computeGivePose(tf::StampedTransform hand_tf){
  // computes the position to give the tool to the hand
  geometry_msgs::Pose pose;
  pose.position.x=hand_tf.getOrigin().x();
  pose.position.y=hand_tf.getOrigin().y();
  pose.position.z=hand_tf.getOrigin().z()+0.1;
  geometry_msgs::Quaternion quat_msg;
  tf::quaternionTFToMsg(tf::createQuaternionFromRPY(angles::from_degrees(-90),angles::from_degrees(-20),angles::from_degrees(0)),quat_msg);
  
  pose.orientation=quat_msg;
  give_pose=pose;
}

geometry_msgs::Pose MoveRobot::getHandTarget(){
  return hand_target;
}

geometry_msgs::Pose MoveRobot::getToolTarget(){
  return tool_target;
}

void MoveRobot::placeTool(){
  // place the tool at fixed place in the cell
  
  sleepSafeFor(0.5);
  float delta_z=0.1;
  tool_place.position.z+=delta_z;
  if(!executeCartesianTrajtoPose(tool_place)){
    return;
  }
  tool_place.position.z-=delta_z;
  if(!executeCartesianTrajtoPose(tool_place)){
    return;
  }
  sleepSafeFor(0.5);
  if (gripperOpen()){
    addRemoveToolObject(false);
    tool_place.position.z+=delta_z;
    executeCartesianTrajtoPose(tool_place);
    tool_place.position.z-=delta_z;
    _status=nominal_task;
    timer=ros::Time::now();
  }
}

void MoveRobot::pickTool(){
  //pick tool from fixed place in the cell
  float delta_z=0.1;
  tool_place.position.z+=delta_z;
  if(!executeCartesianTrajtoPose(tool_place)){
    return;
  }
  tool_place.position.z-=delta_z;
  if(!executeCartesianTrajtoPose(tool_place)){
    return;
  }
  sleepSafeFor(0.5);
  if (gripperClose()){
    addRemoveToolObject(true);
    tool_place.position.z+=delta_z;
    if(!executeCartesianTrajtoPose(tool_place)){
      return;
    }
    tool_place.position.z-=delta_z;
    _status=handover_hand_place;
    timer=ros::Time::now();
  }
}

void MoveRobot::addRemoveToolObject(bool add){

  // add/remove the tool from the MoveIT planner
  // i.e. attach/detach it from the robot
  moveit_msgs::AttachedCollisionObject attached_object;
  attached_object.link_name = move_group->getEndEffectorLink();
/* The header must contain a valid TF frame*/
attached_object.object.header.frame_id = move_group->getEndEffectorLink();
/* The id of the object */
attached_object.object.id = "tool";

/* A default pose */
geometry_msgs::Pose pose;
pose.orientation.w = 1.0;

/* Define a box to be attached */
shape_msgs::SolidPrimitive primitive;
primitive.type = primitive.BOX;
primitive.dimensions.resize(3);
primitive.dimensions[0] = 0.015;
primitive.dimensions[1] = 0.2;
primitive.dimensions[2] = 0.015;


pose.position.x = 0.0;
pose.position.y = 0.08;
pose.position.z = 0.0; 

attached_object.object.primitives.push_back(primitive);
attached_object.object.primitive_poses.push_back(pose);

attached_object.object.operation = add ? attached_object.object.ADD : attached_object.object.REMOVE;
attached_object.touch_links = std::vector<std::string>{ "egp_50_tip", "egp50_pincer_link", "egp50_body_link", "egp50_base_link" };

moveit_msgs::PlanningScene planning_scene;
if (!add){
  //to detach it
  planning_scene.robot_state.attached_collision_objects.clear();
  planning_scene.world.collision_objects.clear();
  planning_scene.world.collision_objects.push_back(attached_object.object);
}

planning_scene.robot_state.is_diff = true;
planning_scene.is_diff = true;
planning_scene.robot_state.attached_collision_objects.push_back(attached_object);
planning_scene_diff_publisher.publish(planning_scene);

}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_robot");
  ros::NodeHandle nh;
  tf::TransformListener lstnr(ros::Duration(5));
  hand_listener=&lstnr;  
  ros::AsyncSpinner spinner(1);
  spinner.start();

  handover_pub =nh.advertise<std_msgs::Bool>("/handover/approach_flag",1);
  ros::Subscriber grasp_sub=nh.subscribe("/handover/grasp_flag",1, graspCallback);
  ros::Subscriber handover_sub=nh.subscribe("/handover/handover_data",1,handoverDataCallback);
  
  handover_dir_pub=nh.advertise<std_msgs::Bool>("/handover/direction",1);
  state_pub=nh.advertise<std_msgs::String>("/robot_state_machine/state",1);
  MoveRobot robot_obj;
  robot_obj.initialiseMoveit(nh);
  
  //point A for nominal task
  geometry_msgs::Pose target_pose1;
  target_pose1.position.x = 0.3;
  target_pose1.position.y = 0.4; //0.4
  target_pose1.position.z = 0.05;
  geometry_msgs::Quaternion quat_msg;
  tf::quaternionTFToMsg(tf::createQuaternionFromRPY(angles::from_degrees(180),angles::from_degrees(0),angles::from_degrees(0)),quat_msg);
  target_pose1.orientation = quat_msg;
  //point B for nominal task
  geometry_msgs::Pose target_pose2 = target_pose1;
  target_pose1.position.x = 0.05;
  int seq = 0;
  bool switcher=true;
 
  robot_obj.executeCartesianTrajtoPose((switcher)?target_pose1:target_pose2);
  
  while(ros::ok())
  {
  //here is the state machine
  std_msgs::Bool flag;  
  std_msgs::String state;
  switch(robot_obj.getStatus()){
    case nominal_task :
      
      state.data="Nominal task";
      state_pub.publish(state);
      flag.data=false;
      handover_pub.publish(flag);
      switcher = !switcher;
      robot_obj.executeCartesianTrajtoPose((switcher)?target_pose1:target_pose2);
      robot_obj.sleepSafeFor(2.0);
      
      break;

    case handover_hand_pick :
      state.data="On hand to pick";
      state_pub.publish(state);
      flag.data=false;
      handover_pub.publish(flag);
      ROS_WARN_STREAM("Go To Hand to pick tool");    
      robot_obj.executeCartesianTrajtoPose(robot_obj.getHandTarget());
      robot_obj.sleepSafeFor(0.5);
      
      
      break;

    case handover_tool_pick :
      state.data="Pick tool from hand";
      state_pub.publish(state);
      flag.data=true;
      handover_pub.publish(flag);
      robot_obj.executeCartesianTrajtoPose(robot_obj.getToolTarget());
      ROS_WARN_STREAM("Go To Tool");
      robot_obj.sleepSafeFor(1.0);
      break;

    case place_tool :
      state.data="Place tool in cell";
      state_pub.publish(state);
      flag.data=false;
      handover_pub.publish(flag);
      ROS_WARN_STREAM("Place Tool");
      robot_obj.placeTool();
      robot_obj.sleepSafeFor(1.0);

      break;

    case handover_hand_place :
      state.data="On hand to place";
      state_pub.publish(state);
      flag.data=false;
      handover_pub.publish(flag);
      ROS_WARN_STREAM("Go To Hand to place tool");    
      robot_obj.executeCartesianTrajtoPose(robot_obj.getHandTarget());
      robot_obj.sleepSafeFor(0.5);
      break;


    case pick_tool :
      state.data="Pick tool from cell";
      state_pub.publish(state);
      flag.data=false;
      handover_pub.publish(flag);
      robot_obj.pickTool();
      ROS_WARN_STREAM("Pick Tool");
      robot_obj.sleepSafeFor(1.0);
      break;

    case handover_tool_place :
      state.data="Place tool in hand";
      state_pub.publish(state);
      flag.data=true;
      handover_pub.publish(flag);
      ROS_WARN_STREAM("Give Tool in hand");
      robot_obj.executeCartesianTrajtoPose(robot_obj.getToolTarget());
      robot_obj.sleepSafeFor(1.0);
      break;

  }
      
  robot_obj.updateStatus();
  robot_obj.update_handover_status();   
  
  
  
  

  
  ros::spinOnce();
  
    
  }
return 0;
}