/*
ver 1.0.0




#include <ros.ros.h>
#include <ros/console.h>

#include "geometry_msgs/PointStamped.h"

#include <move_base/move_base.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/action_client.h>
*/

#include "simple_navigation_goals.hpp"

namespace autoexplorer
{

SimpleNavigationGoals::SimpleNavigationGoals(const ros::NodeHandle &nh_):
m_nh(nh_){ 
    m_gridmapsub = m_nh.subscribe("m_gridmap_fin", 1000, &SimpleNavigationGoals::mapCallback, this);
    m_mapAvailable = 0;
    
}

SimpleNavigationGoals::~SimpleNavigationGoals(){ }

void SimpleNavigationGoals::right(){
    ROS_INFO("send goal right\n");
}

void SimpleNavigationGoals::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg){
    m_gridmap = *msg;
    ROS_INFO("map map");
    m_data = m_gridmap.data;
    m_resolution = m_gridmap.info.resolution;
    m_rows = m_gridmap.info.height;
    m_cols = m_gridmap.info.width;
    m_mapAvailable = 1;
    
    ROS_INFO("Got map %d, %d", m_rows, m_cols);
    for(int i=0; i<m_rows*m_cols;i++){
        ROS_INFO("%c ", m_data[i]);
    }
}

void SimpleNavigationGoals::GetMap(){
    ROS_INFO("waiting subscribe");
    
    
    //    const std::unique_lock<mutex> lock(s_mutex_gridmap);
}

void SimpleNavigationGoals::SendGoal()
{
// call actionlib
// robot is ready to move
	ROS_INFO("Robot state in simple navigation goals \n ");
	
	actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> m_move_client_SNG("move_base", true);
	//if( m_eRobotState >= ROBOT_STATE::FORCE_TO_STOP   )
	//	return;

	//geometry_msgs::PoseWithCovarianceStamped goalpose = *msg ;

	//{
		//const std::unique_lock<mutex> lock(mutex_robot_state) ;
	//	m_eRobotState = ROBOT_STATE::ROBOT_IS_MOVING ;
	//}

	ROS_INFO("@simpleNavigationGoals received a plan\n");

	move_base_msgs::MoveBaseGoal goal;
	goal.target_pose.header.frame_id = "map"; //m_worldFrameId; //m_baseFrameId ;
	goal.target_pose.header.stamp = ros::Time::now();

//	geometry_msgs::PoseWithCovarianceStamped goalpose = // m_pathplan.poses.back() ;
	//put a goal point where i want to send.
	goal.target_pose.pose.position.x = 2.0 ;
	goal.target_pose.pose.position.y = 0.5 ;
	goal.target_pose.pose.orientation.w = 1.0;
	ROS_INFO("setting the goal to (%f, %f, %f)\n", goal.target_pose.pose.position.x, goal.target_pose.pose.position.y, goal.target_pose.pose.orientation.w);
// inspect the path
//////////////////////////////////////////////////////////////////////////////////////////////
    ROS_INFO("+++++++++++++++++++++++++ @msimpleNavigationGoals, sending a goal +++++++++++++++++++++++++++++++++++++\n");
        m_move_client_SNG.sendGoal(goal);
    ROS_INFO("+++++++++++++++++++++++++ @simpleNavigationGoals, a goal is sent +++++++++++++++++++++++++++++++++++++\n");
        m_move_client_SNG.waitForResult();
    ROS_INFO("+++++++++++++++++++++++++ @simpleNavigationGoals, waiting for result +++++++++++++++++++++++++++++++++++++\n");
    if(m_move_client_SNG.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("Move successfully \n");
    else
        ROS_INFO("Failed \n");
    }

}
