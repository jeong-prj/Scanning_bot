#ifndef INCLUDE_FIND_POSITIONS_CLASS_HPP_
#define INCLUDE_FIND_POSITIONS_CLASS_HPP_


#include <ros/ros.h>
#include <ros/console.h>

#include "nav_msgs/OccupancyGrid.h"
#include "map_msgs/OccupancyGridUpdate.h"
#include "nav_msgs/MapMetaData.h"
#include <visualization_msgs/Marker.h>

//#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/action_client.h>

#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <queue>
#include <array>

//#include <boost/bind.hpp>

#include "tsp.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

extern "C" {
#include <concorde.h>
}

namespace find_positions
{

using namespace std;
  
class FindPositions{
public:
  FindPositions();
  virtual ~FindPositions();
  
  array<double, 2> gridmapToworld(int x, int y);
  array<double, 2> getPoseTo2D(int pose);
  int get2DToPose(double x, double y);
  
  void setMarker(){
    m_points.header.frame_id = line_strip.header.frame_id= "map";
    m_points.header.stamp = line_strip.header.stamp = ros::Time(0);
    m_points.ns = line_strip.ns = "way_points";
    m_points.id=1000;
    m_points.type=visualization_msgs::Marker::POINTS;
    line_strip.type=visualization_msgs::Marker::LINE_STRIP;
    
    m_points.action = line_strip.action =visualization_msgs::Marker::ADD;
    m_points.pose.orientation.w = line_strip.pose.orientation.w =1.0;
    
    m_points.scale.x = 0.1;
    m_points.scale.y = 0.1;
    
    line_strip.scale.x = 0.03;

    m_points.color.b = 1.0f;
    m_points.color.a = 1.0;
    
    line_strip.color.b=1.0f;
    line_strip.color.a=1.0;
    
    m_points.lifetime = ros::Duration();
  }
  void writeDataFile (vector<array<double,2>> positions);
  ////void solving_tsp_concorde(vector<int> * tour, int flag);
  void solving_tsp_concorde(int flag);
  void padding(int pose, int siz);
  ////void findWaypointsDistance(vector<int> *tour);
  void findWaypointsDistance(int *tour);
  void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
  
  int m_mapAvailable = 0;
  
  vector<array<double,2>> waypointsWorld;
  
protected:
  //                     up ri  do  le
  array<int,4> x_move = { 0, 1, 0, -1};
  array<int,4> y_move = {-1, 0, 1,  0};
  
  ros::NodeHandle m_nh;
  
  ros::Subscriber m_gridmapsub;
  ros::Publisher m_wayPointsPub;
  
  TSP::TSPSolver m_TSPSolver;
  int map_width;
  int map_height;
  double map_resolution;
  array<double, 2> gridmap_origin_pose;
  vector<signed char> origin_map;
  vector<signed char> map_flags;
  visualization_msgs::Marker m_points, line_strip;
  vector<array<double,2>> middles;
  vector<array<double,2>> waypoints;
  

};
  
}


#endif
