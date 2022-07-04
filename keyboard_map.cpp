#include <ros/ros.h>
#include <iostream>
using namespace std;

int check_send = 0;

void mapCallBack(const nav_msgs::OccupancyGrid::ConstPtr& msg){
  nav_msgs::OccupancyGrid m_gridmap_fin = *msg;
  
  if(check_send==1){
    mapPub.publish(m_gridmap_fin);
  }
}
int main(int argc, char** argv){  
  ros::init(argc, argv, "keyboard");
  ros::NodeHandle n("~");

  ros::Publisher mapPub = m_nh.advertise<nav_msgs::OccupancyGrid>("m_gridmap_fin", 1000);
  ros::Subscriber mapSub = m_nh.subscribe("/map", 10, mapCallBack);
  
  int x; 
  while(1){
    cout << "Type a number 1: ";
    cin >> x; 
    if(cin.fail()){
      cin.clear();
      cin.ignore();
      continue;
    }
    
    if(x==1){
      check_send = x;
      cout << "end"<<endl;
      break;
    }
  }
}
