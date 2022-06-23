//This file solves a TSP using concorde given a symmetric 2D distance matrix

//Info about concorde functions: www.math.uwaterloo.ca/tsp/concorde/DOC/concorde_org.html
#include "getmap.hpp"
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

extern "C" {
#include <concorde.h>
}

using namespace std;
//                     up ri  do  le
array<int,4> x_move = { 0, 1, 0, -1};
array<int,4> y_move = {-1, 0, 1,  0};
// 
int m_mapAvailable = 0;
TSP::TSPSolver m_TSPSolver;
array<double, 2> gridmap_origin_pose;
vector<array<double,2>> middles;
visualization_msgs::Marker m_points, line_strip;
vector<array<double,2>> waypoints;
vector<array<double,2>> waypointsWorld;
int map_width;
int m_rows;
vector<signed char> origin_map;
vector<signed char> map_flags;


array<double, 2> gridmapToworld(int x, int y){
  //0.05 is resolution of the map
  double wx = (x * 0.05) + gridmap_origin_pose[0];
  double wy = (y * 0.05) + gridmap_origin_pose[1];
  
  return {wx, wy};
}

array<double, 2> getPoseTo2D(int wid, int pose){
  double x = pose % wid;
  double y = pose / wid;
  
  return {x, y};  
}

int get2DToPose(int wid, double x, double y){
  return (y*wid) + x;
}

double cacluateDistance(double x_axis, double y_axis){
  return sqrt((x_axis*x_axis) + (y_axis*y_axis));
}

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

void writeDataFile (vector<array<double,2>> positions) {
  ROS_INFO("function write data file");
  FILE *out = fopen (m_TSPSolver.name, "w");
  m_TSPSolver.ncount = positions.size();
  
  fprintf (out, "NAME: concorde%d\n",m_TSPSolver. ncount);
  fprintf (out, "TYPE: TSP\n");
  fprintf (out, "DIMENSION: %d\n", m_TSPSolver.ncount);
  fprintf (out, "EDGE_WEIGHT_TYPE: EUC_2D\n");
  fprintf (out, "NODE_COORD_SECTION\n");
  
  for (int i = 0; i < m_TSPSolver.ncount; i++) {
    fprintf(out, "%d %lf %lf\n", i, positions[i][0], positions[i][1]);
  }

  fprintf(out, "EOF\n");
  fclose(out);
  
  ROS_INFO("done the write task");
}

//Receive a symmetric 2D distance matrix (dist) and create a TSP optimal tour (tour)
void solving_tsp_concorde(vector<int> * tour, int flag){

  //TSP::TSPSolver m_TSPSolver;
  
  int rval = 0; //Concorde functions return 1 if something fails
  double szeit; //Measure cpu time
  
  string str = "map.tsp";
  m_TSPSolver.name = new char[str.size() + 1];
  copy(str.begin(), str.end(), m_TSPSolver.name);
  m_TSPSolver.name[str.size()]='\0';
  
  if(flag ==0)
    writeDataFile(middles);
  else
    writeDataFile(waypoints);
  
  CCrandstate rstate;
  int seed = rand();
  CCutil_sprand(seed, &rstate); //Initialize the portable random number generator
  m_TSPSolver.out_tour = CC_SAFE_MALLOC (m_TSPSolver.ncount, int);
  
  CCdatagroup dat;
  cout<<"=================initialize================="<<endl;
  //Initialize a CCdatagroup
  CCutil_init_datagroup (&dat);
  CCutil_gettsplib (m_TSPSolver.name, &m_TSPSolver.ncount, &dat);
  
  cout<<"=================solve data================="<<endl;
  //Solves the TSP over the graph specified in the datagroup
  rval = CCtsp_solve_dat (m_TSPSolver.ncount, &dat, m_TSPSolver.in_tour, m_TSPSolver.out_tour,
                              m_TSPSolver.in_val, &m_TSPSolver.optval, &m_TSPSolver.success,
                              &m_TSPSolver.foundtour, m_TSPSolver.name, m_TSPSolver.timebound,
                              &m_TSPSolver.hit_timebound, m_TSPSolver.silent, &rstate);
  geometry_msgs::Point p;
  
  cout <<m_TSPSolver.ncount<<endl;
  //Creating a sequential tour
  for(int i = 0; i < m_TSPSolver.ncount; i++){
    tour->push_back(i);
  }
  cout <<tour->size()<<endl;
  for (int i = 0; i < m_TSPSolver.ncount; i++) {
    tour->at(i) = m_TSPSolver.out_tour[i];
    if(flag ==1){
      int turn= m_TSPSolver.out_tour[i];
      double x=waypoints[turn][0], y=waypoints[turn][1];
      
      array<double, 2> world_position = gridmapToworld(x,y);
      waypointsWorld.push_back({world_position[0], world_position[1]});
      p.x=world_position[0]; p.y=world_position[1]; p.z=0.0;
      
      m_points.points.push_back(p);
      line_strip.points.push_back(p);
    }
  }
  cout<<endl<<"done"<<endl;

  //szeit = CCutil_zeit();
  CC_IFFREE (m_TSPSolver.out_tour, int);
//  CC_IFFREE (m_TSPSolver.name, char);
}

void padding(int pose, int wid, int siz){
  array<double, 2> grid = getPoseTo2D(wid, pose);
  for(int i=grid[0]-siz;i<=grid[0]+siz;i++){
    for(int j=grid[1]-siz;j<=grid[1]+siz;j++){
      if(i<0||i>=wid||j<0||j>=m_rows){
        continue;
      }
      int flag_pose=get2DToPose(wid,i,j);
      if(origin_map[flag_pose]==0){
        map_flags[flag_pose]=2;
      }
    }
  }
}

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg){
                //,vector<signed char>& m_data, vector<signed char>& map_flags){
  nav_msgs::OccupancyGrid m_gridmap = *msg;
  ROS_INFO("map callback");
  
  //grid map
  map_flags = m_gridmap.data;
  origin_map = m_gridmap.data;
  int m_resolution = m_gridmap.info.resolution;
  int m_cols= m_gridmap.info.width;
  map_width = m_cols;
  m_rows = m_gridmap.info.height;
  gridmap_origin_pose = {m_gridmap.info.origin.position.x, m_gridmap.info.origin.position.y};
  m_mapAvailable = 1;
    
  ROS_INFO("%d Got map %d, %d, size: %d", m_mapAvailable, m_cols, m_rows, origin_map.size());
  
  for(int i=0; i<origin_map.size();i++){
    if(origin_map[i]!=0){
      padding(i, m_cols, 9);
    }
  }
  
  int square_size = 3;
  for(int x = square_size/2; x<m_cols; x+=square_size){
    for(int y = square_size/2; y<m_rows; y+=square_size){
      int check_pose = get2DToPose(m_cols, x, y);
      if(map_flags[check_pose] == 0){
        middles.push_back({x, y});
      }
    }
  }
}

void findWaypointsDistance(vector<int> *tour){
  for(int i=0; i<tour->size(); i++){
    int turn = tour->at(i);
    array<double, 2> cur_2D = middles[turn];
    double x=cur_2D[0], y=cur_2D[1];
    int pose=get2DToPose(map_width, x, y);
    if(map_flags[pose]==0){
      cout<< "position: "<<x<< ", "<<y<<endl;
      waypoints.push_back({x, y});
      
      padding(pose, map_width, 38);
    }
  }
}

int main(int argc, char **argv){
  ros::init(argc, argv, "find_positions");
  ros::NodeHandle m_nh;
  ros::Publisher m_wayPointsPub = m_nh.advertise<visualization_msgs::Marker>("way_point", 10000);
  ros::Subscriber m_gridmapsub = m_nh.subscribe<nav_msgs::OccupancyGrid>("m_gridmap_fin", 1000, mapCallback); 
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);
  
  while(!m_mapAvailable){
    ros::spinOnce();
  }
  setMarker();
  //Tour stores the optimal tour
  vector<int> * tour = new vector<int>(m_TSPSolver.ncount, 0);

  //Solve TSP
  solving_tsp_concorde(tour, 0);

  //Print solution
  for(int i = 0; i < tour->size(); i++)
    cout<<tour->at(i)<<"_";
  cout << endl;
  
  //Find waypoints
  findWaypointsDistance(tour);
  
  vector<int> * fin_tour = new vector<int>(m_TSPSolver.ncount, 0);
  solving_tsp_concorde(fin_tour, 1);
  
  m_wayPointsPub.publish(m_points);
  m_wayPointsPub.publish(line_strip);
  
  for(int i = 0; i < waypointsWorld.size(); i++){
    double x=waypointsWorld[i][0], y=waypointsWorld[i][1];
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
      ros::Duration(10).sleep();
    }
    else
      ROS_INFO("The robot failed to reach the goal location for some reason");
  }
  
  //Print the waypoints on the map
  ROS_INFO("done find position");
  
  return 0;
}
