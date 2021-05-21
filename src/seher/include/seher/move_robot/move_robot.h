// template of the robot class


#ifndef MOVE_SIM_ROBOT_H
#define MOVE_SIM_ROBOT_H

//Includes
#include "ros/ros.h"

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene/planning_scene.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <geometry_msgs/PoseStamped.h>

#include "ur_msgs/SetIO.h"
#include <tf/transform_listener.h>
#include "std_msgs/Float32.h"
#include <seher_msgs/handover.h>

//states of the robot state machine
enum status {nominal_task, handover_hand_pick, handover_tool_pick, handover_hand_place, handover_tool_place, place_tool, pick_tool};


//workcell limits
const float WORKCELL_XMAX=0.46;
const float WORKCELL_XMIN=-0.5;
const float WORKCELL_YMIN=-0.28;
const float WORKCELL_YMAX=1.19;
const float WORKCELL_ZMIN=0.0;
const float WORKCELL_ZMAX=1.0;


class MoveRobot {



public:
MoveRobot();
~MoveRobot();

const std::string GROUP_MANIP = "manipulator";

moveit::planning_interface::MoveGroupInterface *move_group;

ros::Publisher planning_scene_diff_publisher;

//methods
void initialiseMoveit(ros::NodeHandle nh);
bool comparePoses(geometry_msgs::Pose pose1, geometry_msgs::Pose pose2, double delta_posistion=0.05, double delta_orientation=0.01);
void sleepSafeFor(double duration);
bool executeCartesianTrajtoPose(geometry_msgs::Pose target);
bool moveToTarget(geometry_msgs::Pose target);
moveit::planning_interface::MoveGroupInterface::Plan getCartesianPathPlanToPose(geometry_msgs::Pose target_pose, double eef_step=0.01, double jump_threshold = 0.0); //0.01
bool moveGroupExecutePlan(moveit::planning_interface::MoveGroupInterface::Plan my_plan);
void adjustTrajectoryToFixTimeSequencing(moveit_msgs::RobotTrajectory &trajectory);
void updateStatus();
float distanceComputing (geometry_msgs::Point point1, geometry_msgs::Point point2);
void update_hand_position(tf::StampedTransform transform);
bool is_in_the_cell(tf::StampedTransform transform);
void update_handover_status();
enum status getStatus();
void computePoseToHand();
geometry_msgs::Pose getHandTarget();
void computePoseToTool(tf::StampedTransform tool_tf);
geometry_msgs::Pose getToolTarget();
bool gripperClose();
bool gripperOpen();
void placeTool();
void pickTool();
void addRemoveToolObject(bool add);
void computeGivePose(tf::StampedTransform hand_tf);
void timeOut();



private:

moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
const robot_state::JointModelGroup* joint_model_group ;
moveit_visual_tools::MoveItVisualTools *visual_tools;
Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
const int IO_SERVICE_FUN_LEVEL_ = 1;   // Not exactly sure what this is, but 1 seems to work. If it fails, try 2.
geometry_msgs::Point hand_position_current; //current hand position 
geometry_msgs::Pose hand_target;  //hand approach target for the handover
geometry_msgs::Pose tool_target;  //tool target for the handover
geometry_msgs::Pose tool_place;  
geometry_msgs::Pose give_pose;  //position for giving the tool to the hand
status _status;  //status of the state machine
ros::NodeHandle n;
ros::Publisher handover_dir_pub;
ros::Time timer;
ros::Duration timeout;
 
};

#endif