#include <ros/ros.h>
#include <tf/transform_listener.h>


#include <cmath>



#include "std_msgs/Float32.h"
#include <seher_msgs/distances.h>



tf::TransformListener* listener=NULL;

seher_msgs::distances distanceComputing (){
    tf::StampedTransform transform_TCP;
    tf::StampedTransform transform_head;
    tf::StampedTransform transform_left_shoulder;
    tf::StampedTransform transform_right_shoulder;
    tf::StampedTransform transform_left_elbow;
    tf::StampedTransform transform_right_elbow;
    tf::StampedTransform transform_left_hand;
    tf::StampedTransform transform_right_hand;
    tf::StampedTransform transform_torso;
    float min_distance=100;
    seher_msgs::distances result;

    try{
      listener->lookupTransform("/world", "/tool0",  
                               ros::Time(0), transform_TCP);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      //ros::Duration(1.0).sleep();
    }
    float x,y,z;
    x=transform_TCP.getOrigin().x();
    y=transform_TCP.getOrigin().y();
    z=transform_TCP.getOrigin().z();
    try{
      listener->lookupTransform("/world", "/camL_link/head",  
                               ros::Time(0), transform_head);
      result.head=sqrt(pow(transform_head.getOrigin().x()-x,2)+pow(transform_head.getOrigin().y()-y,2)+pow(transform_head.getOrigin().z()-z,2));
      if (result.head<min_distance) {
        min_distance=result.head;
      }
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      //ros::Duration(1.0).sleep();
    }

    try{
      listener->lookupTransform("/world", "/camL_link/left_shoulder",  
                               ros::Time(0), transform_left_shoulder);
      result.left_shoulder=sqrt(pow(transform_left_shoulder.getOrigin().x()-x,2)+pow(transform_left_shoulder.getOrigin().y()-y,2)+pow(transform_left_shoulder.getOrigin().z()-z,2));
      if (result.left_shoulder<min_distance) {
        min_distance=result.left_shoulder;
      }
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      //ros::Duration(1.0).sleep();
    }

    try{
      listener->lookupTransform("/world", "/camL_link/right_shoulder",  
                               ros::Time(0), transform_right_shoulder);
      result.right_shoulder=sqrt(pow(transform_right_shoulder.getOrigin().x()-x,2)+pow(transform_right_shoulder.getOrigin().y()-y,2)+pow(transform_right_shoulder.getOrigin().z()-z,2));
      if (result.right_shoulder<min_distance) {
        min_distance=result.right_shoulder;
      }
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      //ros::Duration(1.0).sleep();
    }

    try{
      listener->lookupTransform("/world", "/camL_link/left_elbow",  
                               ros::Time(0), transform_left_elbow);
      result.left_elbow=sqrt(pow(transform_left_elbow.getOrigin().x()-x,2)+pow(transform_left_elbow.getOrigin().y()-y,2)+pow(transform_left_elbow.getOrigin().z()-z,2));
      if (result.left_elbow<min_distance) {
        min_distance=result.left_elbow;
      }
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      //ros::Duration(1.0).sleep();
    }

    try{
      listener->lookupTransform("/world", "/camL_link/right_elbow",  
                               ros::Time(0), transform_right_elbow);
      result.right_elbow=sqrt(pow(transform_right_elbow.getOrigin().x()-x,2)+pow(transform_right_elbow.getOrigin().y()-y,2)+pow(transform_right_elbow.getOrigin().z()-z,2));
      if (result.right_elbow<min_distance) {
        min_distance=result.right_elbow;
      }
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      //ros::Duration(1.0).sleep();
    }

    try{
      listener->lookupTransform("/world", "/camL_link/left_hand",  
                               ros::Time(0), transform_left_hand);
      result.left_hand=sqrt(pow(transform_left_hand.getOrigin().x()-x,2)+pow(transform_left_hand.getOrigin().y()-y,2)+pow(transform_left_hand.getOrigin().z()-z,2));
      if (result.left_hand<min_distance) {
        min_distance=result.left_hand;
      }
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      //ros::Duration(1.0).sleep();
    }
    
    try{
      listener->lookupTransform("/world", "/camL_link/right_hand",  
                               ros::Time(0), transform_right_hand);
      result.right_hand=sqrt(pow(transform_right_hand.getOrigin().x()-x,2)+pow(transform_right_hand.getOrigin().y()-y,2)+pow(transform_right_hand.getOrigin().z()-z,2));
      if (result.right_hand<min_distance) {
        min_distance=result.right_hand;
      }
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      //ros::Duration(1.0).sleep();
    }

    try{
      listener->lookupTransform("/world", "/camL_link/torso",  
                               ros::Time(0), transform_torso);
      result.torso=sqrt(pow(transform_torso.getOrigin().x()-x,2)+pow(transform_torso.getOrigin().y()-y,2)+pow(transform_torso.getOrigin().z()-z,2));
      if (result.torso<min_distance) {
        min_distance=result.torso;
      }
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      //ros::Duration(1.0).sleep();
    }
    
    result.minimal=min_distance;
    return result;
}







int main(int argc, char** argv){
  ros::init(argc, argv, "distance_calculation_skeleton");

  ros::NodeHandle node;
  
  
  listener=new (tf::TransformListener);

  ros::Publisher distances_pub=node.advertise<seher_msgs::distances>("/distance_calculation/body_distances",1);
  
  seher_msgs::distances computed_distances;
   
  ros::Rate rate(30.0);
  while (node.ok()){
    
    
    seher_msgs::distances computed_distances=distanceComputing();

    
    

    ROS_INFO_STREAM("minimal distance between TCP and human body : " << computed_distances.minimal);
    
    distances_pub.publish(computed_distances);
    rate.sleep();
    
  }

  
  return 0;
};
