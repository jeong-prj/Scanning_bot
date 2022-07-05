#include <ros/ros.h>
#include <ros/console.h>

#include "nav_msgs/OccupancyGrid.h"
#include "std_msgs/Bool.h"

#include <iostream>


using namespace std;

int s_map_available = 0;
nav_msgs::OccupancyGrid s_fin_gridmap;

void s_mapCallBack(const nav_msgs::OccupancyGrid::ConstPtr& msg){
  s_fin_gridmap = *msg;
  s_map_available = 1;
  ROS_INFO("got map.. %d, info: %d", s_map_available, s_fin_gridmap.data.size());
}

int main(int argc, char** argv){  
  ros::init(argc, argv, "send_map");
  ros::NodeHandle s_nh;

  ros::Publisher mapPub = s_nh.advertise<nav_msgs::OccupancyGrid>("m_gridmap_fin", 1000);
  ros::Subscriber mapSub = s_nh.subscribe<nav_msgs::OccupancyGrid>("/map", 1000, s_mapCallBack);
  //publish autoexplorer done
  ros::Publisher explorerPub = s_nh.advertise<std_msgs::Bool>("exploration_is_done", 1);
  
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
  
  mapPub.publish(s_fin_gridmap);
  explorerPub.publish(done_task);
  ROS_INFO("map x: %d, y: %d", s_fin_gridmap.info.width, s_fin_gridmap.info.height);
  ROS_INFO("send map..");
  
}
