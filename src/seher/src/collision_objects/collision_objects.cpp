/*
This node publishes the static collision objects, i.e. walls and floor of the workcell
*/

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

const double BASE_OFFSET_FROM_BACK_WALL_ = 0.28;   //28cm
  const double BASE_OFFSET_FROM_LEFT_WALL_ = 0.46;   //46cm
  const double BASE_OFFSET_FROM_RIGHT_WALL_ = 0.5;   //50cm
  const double TOTAL_INNER_CELL_Y_DIMENSION_ = 1.47; //1470cm
  const double TOTAL_INNER_CELL_X_DIMENSION_ = 0.96; //960cm
  const double TOTAL_INNER_CELL_Z_DIMENSION = 1.15;  //115cm
  
moveit::planning_interface::MoveGroupInterface *move_group;



int main(int argc, char **argv)
{
  ros::init(argc, argv, "collision_objects");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::Publisher planning_scene_diff_publisher = nh.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
  //moveit::planning_interface::MoveGroupInterface group("right_arm");
  move_group = new moveit::planning_interface::MoveGroupInterface("manipulator");
  planning_scene_diff_publisher = nh.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
  ROS_INFO("Adding static objects");
  
  
  moveit_msgs::AttachedCollisionObject object;
  object.link_name = move_group->getPlanningFrame();
  object.object.header.frame_id = move_group->getPlanningFrame();
  object.object.id = "Floor";

  

  // Define a box to add to the world.
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = TOTAL_INNER_CELL_X_DIMENSION_;
  primitive.dimensions[1] = TOTAL_INNER_CELL_Y_DIMENSION_;
  primitive.dimensions[2] = 0;

  // Define a pose for the box (specified relative to frame_id)
  geometry_msgs::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x = -(BASE_OFFSET_FROM_LEFT_WALL_-BASE_OFFSET_FROM_RIGHT_WALL_)/2;  // Not perfectly symmetrical.
  box_pose.position.y = TOTAL_INNER_CELL_Y_DIMENSION_/2-BASE_OFFSET_FROM_BACK_WALL_; // Base is ofset by (0.1470/2-.275)
  box_pose.position.z = -0.03; //Push it slightly down to avoid collission with base plate. //-0.03

  // Since we are attaching the object to the robot base
  // we want the collision checker to ignore collisions between the object and the robot base
  object.touch_links = std::vector<std::string>{ "base_link", "tool"};
  moveit_msgs::PlanningScene planning_scene;

  object.object.operation = object.object.ADD;
  object.object.primitives.push_back(primitive);
  object.object.primitive_poses.push_back(box_pose);
  planning_scene.world.collision_objects.push_back(object.object);


  // The id of the object is used to identify it.
  object.object.id = "Cieling";

  // Define a box to add to the world.
  primitive.dimensions[0] = TOTAL_INNER_CELL_X_DIMENSION_;
  primitive.dimensions[1] = TOTAL_INNER_CELL_Y_DIMENSION_;
  primitive.dimensions[2] = 0;

  // Define a pose for the box (specified relative to frame_id
  box_pose.orientation.w = 1.0;
  box_pose.position.x = -(BASE_OFFSET_FROM_LEFT_WALL_-BASE_OFFSET_FROM_RIGHT_WALL_)/2;  // Not perfectly symmetrical.
  box_pose.position.y = TOTAL_INNER_CELL_Y_DIMENSION_/2-BASE_OFFSET_FROM_BACK_WALL_; // Base is ofset by (0.1470/2-.275)
  box_pose.position.z = TOTAL_INNER_CELL_Z_DIMENSION;

  object.object.operation = object.object.ADD;
  object.object.primitives.push_back(primitive);
  object.object.primitive_poses.push_back(box_pose);
  planning_scene.world.collision_objects.push_back(object.object);

  // The id of the object is used to identify it.
  object.object.id  = "Left Wall";

  // Define a box to add to the world.
  primitive.dimensions[0] = 0;
  primitive.dimensions[1] = TOTAL_INNER_CELL_Y_DIMENSION_;
  primitive.dimensions[2] = TOTAL_INNER_CELL_Z_DIMENSION;

  // Define a pose for the box (specified relative to frame_id
  box_pose.orientation.w = 1.0;
  box_pose.position.x = BASE_OFFSET_FROM_RIGHT_WALL_;
  box_pose.position.y = TOTAL_INNER_CELL_Y_DIMENSION_/2-BASE_OFFSET_FROM_BACK_WALL_;
  box_pose.position.z = TOTAL_INNER_CELL_Z_DIMENSION/2;

  object.object.operation = object.object.ADD;
  object.object.primitives.push_back(primitive);
  object.object.primitive_poses.push_back(box_pose);
  planning_scene.world.collision_objects.push_back(object.object);

  // The id of the object is used to identify it.
  object.object.id = "Right Wall";

  // Define a box to add to the world.
  primitive.dimensions[0] = 0;
  primitive.dimensions[1] = TOTAL_INNER_CELL_Y_DIMENSION_;
  primitive.dimensions[2] = TOTAL_INNER_CELL_Z_DIMENSION;

  // Define a pose for the box (specified relative to frame_id
  box_pose.orientation.w = 1.0;
  box_pose.position.x = -BASE_OFFSET_FROM_LEFT_WALL_;
  box_pose.position.y = TOTAL_INNER_CELL_Y_DIMENSION_/2-BASE_OFFSET_FROM_BACK_WALL_;
  box_pose.position.z = TOTAL_INNER_CELL_Z_DIMENSION/2;

  object.object.operation = object.object.ADD;
  object.object.primitives.push_back(primitive);
  object.object.primitive_poses.push_back(box_pose);
  planning_scene.world.collision_objects.push_back(object.object);

  // The id of the object is used to identify it.
  object.object.id = "Back Wall";

  // Define a box to add to the world.
  primitive.dimensions[0] = TOTAL_INNER_CELL_X_DIMENSION_;
  primitive.dimensions[1] = 0;
  primitive.dimensions[2] = TOTAL_INNER_CELL_Z_DIMENSION;

  // Define a pose for the box (specified relative to frame_id
  box_pose.orientation.w = 1.0;
  box_pose.position.x = -(BASE_OFFSET_FROM_LEFT_WALL_-BASE_OFFSET_FROM_RIGHT_WALL_)/2;
  box_pose.position.y = -BASE_OFFSET_FROM_BACK_WALL_;
  box_pose.position.z = TOTAL_INNER_CELL_Z_DIMENSION/2;

  object.object.operation = object.object.ADD;
  object.object.primitives.push_back(primitive);
  object.object.primitive_poses.push_back(box_pose);
  planning_scene.world.collision_objects.push_back(object.object);


  
  planning_scene.is_diff = true;
  planning_scene_diff_publisher.publish(planning_scene);
  ROS_INFO_NAMED("collision_objects", "Adding collission objects into the world"); 
  
  
  return 0;
}

