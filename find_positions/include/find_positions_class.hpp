#ifndef INCLUDE_FIND_POSITIONS_CLASS_HPP_
#define INCLUDE_FIND_POSITIONS_CLASS_HPP_


#include <ros/ros.h>
#include <ros/console.h>

#include "nav_msgs/OccupancyGrid.h"
#include "map_msgs/OccupancyGridUpdate.h"
#include "nav_msgs/MapMetaData.h"
#include "nav_msgs/Odometry.h"
#include <visualization_msgs/Marker.h>

//#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/action_client.h>

#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <queue>
#include <array>
#include <time.h>

//#include <boost/bind.hpp>

#include "tsp.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include <omp.h>
#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Path.h>
#include <nav_core/base_global_planner.h>
#include <nav_msgs/GetPlan.h>
#include <nav_fn/potarr_point.h>
#include "global_planning_handler.hpp"

#include "tf/transform_listener.h"

extern "C" {
#include <concorde.h>
}
#define DIST_HIGH  (1.0e10)

namespace find_positions
{

using namespace std;
  
class FindPositions{
public:
  FindPositions();
  virtual ~FindPositions();
  
  array<double, 2> gridmapToworld(int x, int y);
  array<int, 2> getPoseTo2D(int pose);
  int get2DToPose(int x, int y);
  
  //void setMarker(visualization_msgs::Marker& m_points, visualization_msgs::Marker& line_strip){
  void setMarker(){
    m_points.header.frame_id = line_strip.header.frame_id = t2_line_strip.header.frame_id= "map";
    m_points.header.stamp = line_strip.header.stamp = t2_line_strip.header.stamp = ros::Time(0);
    m_points.ns = line_strip.ns = t2_line_strip.ns = "way_points";
    m_points.id=1000;
    m_points.type=visualization_msgs::Marker::POINTS;
    line_strip.type=visualization_msgs::Marker::LINE_STRIP;
    t2_line_strip.type=visualization_msgs::Marker::LINE_STRIP;  
    
    m_points.action = line_strip.action  =t2_line_strip.action  =visualization_msgs::Marker::ADD;
    m_points.pose.orientation.w = line_strip.pose.orientation.w = t2_line_strip.pose.orientation.w =1.0;
    
    m_points.scale.x = 0.1;
    m_points.scale.y = 0.1;
    
    line_strip.scale.x = 0.03;
    t2_line_strip.scale.x = 0.03;

    m_points.color.b = 1.0f;
    m_points.color.a = 1.0;
    
    
    t2_line_strip.color.b=1.0f;
    t2_line_strip.color.a=1.0;
    line_strip.color.b=1.0f;
    line_strip.color.a=1.0;
    
    m_points.lifetime = ros::Duration();
    
    ///////print path
    m_path_points.header.frame_id = "map";
    m_path_points.header.stamp = ros::Time(0);
    m_path_points.ns = "way_apath";
    m_path_points.id=2000;
    m_path_points.type=visualization_msgs::Marker::POINTS;
    
    m_path_points.action = visualization_msgs::Marker::ADD;
    m_path_points.pose.orientation.w = 1.0;
    
    m_path_points.scale.x = 0.05;
    m_path_points.scale.y = 0.05;

    m_path_points.color.g = 1.0f;
    m_path_points.color.a = 1.0;
    
    m_path_points.lifetime = ros::Duration(); 
    
    //First tsp solver result 
    tsp2_points.header.frame_id = "map";
    tsp2_points.header.stamp = ros::Time(0);
    tsp2_points.ns = "way_apath";
    tsp2_points.id=3000;
    tsp2_points.type=visualization_msgs::Marker::POINTS;
    
    tsp2_points.action = visualization_msgs::Marker::ADD;
    tsp2_points.pose.orientation.w = 1.0;
    
    tsp2_points.scale.x = 0.15;
    tsp2_points.scale.y = 0.15;

    tsp2_points.color.r = 1.0f;
    tsp2_points.color.a = 1.0;
    
    tsp2_points.lifetime = ros::Duration(); 
    
    //tsp solver result 2m
    tsp1_points.header.frame_id = t1_line_strip.header.frame_id ="map";
    tsp1_points.header.stamp = t1_line_strip.header.stamp = ros::Time(0);
    tsp1_points.ns = t1_line_strip.ns = "way_apath";
    tsp1_points.id = t1_line_strip.id =4000;
    tsp1_points.type=visualization_msgs::Marker::POINTS;
    t1_line_strip.type=visualization_msgs::Marker::LINE_STRIP;  
    
    tsp1_points.action = t1_line_strip.action = visualization_msgs::Marker::ADD;
    tsp1_points.pose.orientation.w = t1_line_strip.pose.orientation.w = 1.0;
    
    tsp1_points.scale.x = 0.045;
    tsp1_points.scale.y = 0.045;

    t1_line_strip.scale.x = 0.03;
    t1_line_strip.scale.y = 0.03;
    
    tsp1_points.color.b = t1_line_strip.color.b = 1.0f;
    tsp1_points.color.a = t1_line_strip.color.a = 1.0;
    
    tsp1_points.lifetime = t1_line_strip.lifetime = ros::Duration(); 
    
    //added point
    add_points.header.frame_id = "map";
    add_points.header.stamp = ros::Time(0);
    add_points.ns = "way_apath";
    add_points.id=5000;
    add_points.type=visualization_msgs::Marker::POINTS;
    
    add_points.action = visualization_msgs::Marker::ADD;
    add_points.pose.orientation.w = 1.0;
    
    add_points.scale.x = 0.15;
    add_points.scale.y = 0.15;

    add_points.color.r = 1.0f;
    add_points.color.g = 1.0f;
    add_points.color.a = 1.0;
    
    add_points.lifetime = ros::Duration(); 
    
    
  }
  
  geometry_msgs::PoseStamped StampedPosefromSE2( const float& x, const float& y, const float& yaw_radian ){
    geometry_msgs::PoseStamped outPose ;
    outPose.pose.position.x = x ;
    outPose.pose.position.y = y ;

    float c[3] = {0,};
    float s[3] = {0,};
    c[0] = cos(yaw_radian/2) ;
    c[1] = cos(0) ;
    c[2] = cos(0) ;
    s[0] = sin(yaw_radian/2) ;
    s[1] = sin(0) ;
    s[2] = sin(0) ;

    float qout[4] = {0,};
    qout[0] = c[0]*c[1]*c[2] + s[0]*s[1]*s[2];
    qout[1] = c[0]*c[1]*s[2] - s[0]*s[1]*c[2];
    qout[2] = c[0]*s[1]*c[2] + s[0]*c[1]*s[2];
    qout[3] = s[0]*c[1]*c[2] - c[0]*s[1]*s[2];

    outPose.pose.orientation.w = qout[0] ;
    outPose.pose.orientation.x = qout[1] ;
    outPose.pose.orientation.y = qout[2] ;
    outPose.pose.orientation.z = qout[3] ;

    outPose.header.frame_id = "map" ;
    outPose.header.stamp = ros::Time::now() ;

    return outPose;
  }
  
  void writeDataFile (vector<array<int,2>> positions, int mode, int exclude_n);
  int removeUnreach(vector<array<int,2>> positions);
  ////void solving_tsp_concorde(vector<int> * tour, int flag);
  void solving_tsp_concorde(int flag);
  void padding(int pose, int siz, int ch);
  ////void findWaypointsDistance(vector<int> *tour);
  void findWaypointsDistance(int *tour);
  void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
  void globalCostmapCallBack(const nav_msgs::OccupancyGrid::ConstPtr& msg ) ;
  void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
  
  //int eraseInvalidByDist();
  //int bfs(int node_size, vector<int>* graph, vector<int>& sequence);
  void makeMapForPath(int mode);
  double calculatePath(double ax, double ay, double bx, double by, int mode);
  double takeATime(ros::WallTime start, ros::WallTime end){
    return (end - start).toNSec()*1e-6;
  }
  
  int findFirstNode(vector<array<double,2>> nodes);
  void checkRealPath();
  int m_mapAvailable = 0;
  int m_sposeAvailable = 0;
  
  vector<array<double,2>> result_waypoints;
  
  ros::WallTime totalStartTime, totalEndTime;
  ros::WallTime tsp1StartTime, tsp1EndTime;
  ros::WallTime tsp2StartTime, tsp2EndTime;
  ros::WallTime scanningStartTime, scanningEndTime;
  
  double totalTime, tsp1Time, tsp2Time, scanningTime;
  
  
  
protected:
  //                     up ri  do  le
  array<int,4> x_move = { 0, 1, 0, -1};
  array<int,4> y_move = {-1, 0, 1,  0};
  
  ros::NodeHandle m_nh;
  
  ros::Subscriber m_gridmapsub, m_globalCostmapSub, m_startPoseSub;
  ros::Publisher m_wayPointsPub, m_pathPub, m_tsp2PointsPub, m_addPointsPub, m_tsp1PointsPub, m_tsp1linePub, m_tsp2linePub;
  
  TSP::TSPSolver m_TSPSolver;
  int map_width;
  int map_height;
  double map_resolution;
  array<double, 2> gridmap_origin_pose;
  array<double, 2> start_pose;
  
  vector<signed char> origin_map;
  vector<signed char> map_flags;
  vector<signed char> map_for_add;
  vector<signed char> map_for_path;
  
  vector<array<double,2>> waypointsWorld;
  vector<array<double,2>> open_loop;
  vector<array<double,2>> closed_loop;
  //ej_marker
  visualization_msgs::Marker add_points;
  visualization_msgs::Marker tsp1_points, tsp2_points;
  visualization_msgs::Marker m_points, line_strip, t1_line_strip, t2_line_strip;
  visualization_msgs::Marker m_path_points;
  vector<array<int,2>> middles;
  vector<array<int,2>> waypoints;
  vector<array<double, 2>> tsp2nodes;
  vector<vector<int>> path_lens;
  
  nav_msgs::OccupancyGrid m_globalcostmap ;
	int m_globalcostmap_rows ;
	int m_globalcostmap_cols ;
  costmap_2d::Costmap2D* mpo_costmap;
  uint8_t* mp_cost_translation_table;
};
  
}


#endif
