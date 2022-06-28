#include "find_positions_class.hpp"

using namespace find_positions;

int main(int argc, char **argv){
  ros::init(argc, argv, "find_positions");

  FindPositions m_fp;
  
  while(!m_fp.m_mapAvailable){
    ros::spinOnce();
  }
  m_fp.setMarker();
  //Tour stores the optimal tour
  vector<int> * tour = new vector<int>(m_fp.m_TSPSolver.ncount, 0);

  //Solve TSP
  m_fp.solving_tsp_concorde(tour, 0);

  //Print solution
  for(int i = 0; i < tour->size(); i++)
    cout<<tour->at(i)<<"_";
  cout << endl;
  
  //Find waypoints
  m_fp.findWaypointsDistance(tour);
  
  vector<int> * fin_tour = new vector<int>(m_fp.m_TSPSolver.ncount, 0);
  m_fp.solving_tsp_concorde(fin_tour, 1);
  
  m_fp.m_wayPointsPub.publish(m_fp.m_points);
  m_fp.m_wayPointsPub.publish(m_fp.line_strip);
  
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);
  for(int i = 0; i < m_fp.waypointsWorld.size(); i++){
    double x=m_fp.waypointsWorld[i][0], y=m_fp.waypointsWorld[i][1];
    move_base_msgs::MoveBaseGoal waypoint_goal;
    waypoint_goal.target_pose.header.frame_id = "map";
    waypoint_goal.target_pose.header.stamp = ros::Time::now();
    
    waypoint_goal.target_pose.pose.position.x = x;
    waypoint_goal.target_pose.pose.position.y = y;
    waypoint_goal.target_pose.pose.orientation.w = 1.0;
    
    ac.sendGoal(waypoint_goal);
    
    ac.waitForResult();
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
      ROS_INFO("The robot has arrived at the goal location");
      
      //system("adb shell input tap 800 2300");
      ros::Duration(5).sleep();
    }
    else{
      ROS_INFO("The robot failed to reach the goal location for some reason");
    }
  }
  
  //Print the waypoints on the map
  ROS_INFO("done find_positions_node");
  
  return 0;
}
