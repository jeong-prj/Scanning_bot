#include <ros/ros.h>
#include <ros/console.h>

#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Bool.h"
#include "tf/transform_listener.h"

#include <iostream>


using namespace std;

int s_map_available = 0;
//nav_msgs::Odometry odom_pose;
nav_msgs::OccupancyGrid s_fin_gridmap;

void s_mapCallBack(const nav_msgs::OccupancyGrid::ConstPtr& msg){
  s_fin_gridmap = *msg;
  s_map_available = 1;
  ROS_INFO("got map.. %d, info: %lu", s_map_available, s_fin_gridmap.data.size());
}

int main(int argc, char** argv){
  ros::init(argc, argv, "send_map");
  ros::NodeHandle s_nh;
  
  string mapFrameId = "map";
  string baseFrameId = "base_link";
  /*
  try{
    listener.waitForTransform(mapFrameId, baseFrameId, ros::Time(0), ros::Duration(3.0));
    listener.lookupTransform( mapFrameId, baseFrameId, ros::Time(0), transform);
  }
  
  catch (tf::TransformException &ex)  {
    ROS_ERROR("%s",ex.what());
    ros::Duration(1.0).sleep();
  }
  */
  ros::Publisher mapPub = s_nh.advertise<nav_msgs::OccupancyGrid>("m_gridmap_fin", 1000);
  ros::Subscriber mapSub = s_nh.subscribe<nav_msgs::OccupancyGrid>("/map", 1000, s_mapCallBack);
  //publish autoexplorer done
  ros::Publisher explorerPub = s_nh.advertise<std_msgs::Bool>("exploration_is_done", 1);
  
  geometry_msgs::PoseStamped outPose;
  
  
  int mode; 
  cout << "Type a number mode(3: sk, 4: smallhouse): ";
  cin >> mode; 
  
  if(mode==2){
    outPose.pose.position.x = -25.6;
    outPose.pose.position.y = -14.3;
  }
  else if(mode==3){
    outPose.pose.position.x = -19.5;
    outPose.pose.position.y = 26.5;
  }
  else if(mode==4){
    outPose.pose.position.x = -7;
    outPose.pose.position.y = -3;
  }
  else if(mode==5){
    outPose.pose.position.x = 31;
    outPose.pose.position.y = 13;
  }
  
  //outPose.pose.position.x = transform.getOrigin().x();
  //outPose.pose.position.y = transform.getOrigin().y();
  
	outPose.pose.position.z = 0.f;
	outPose.header.frame_id = mapFrameId;
  ROS_INFO("Find position: %lf, %lf", outPose.pose.position.x, outPose.pose.position.y);
  
  ros::Publisher startPosePub = s_nh.advertise<geometry_msgs::PoseStamped>("start_pose", 1000);
  
  int x; 
  cout << "Type a number 1: ";
  cin >> x; 
  while(x!=1 || s_map_available!=1){
    ros::spinOnce();
  }
  
  ROS_INFO("x: %d, map_available: %d", x, s_map_available);
  cout << "end"<<endl;
  std_msgs::Bool done_task;
  done_task.data = true;
  
  startPosePub.publish(outPose);
  mapPub.publish(s_fin_gridmap);
  explorerPub.publish(done_task);
  ROS_INFO("map x: %d, y: %d", s_fin_gridmap.info.width, s_fin_gridmap.info.height);
  ROS_INFO("send map..");
  
}
