//This file solves a TSP using concorde given a symmetric 2D distance matrix

//Info about concorde functions: www.math.uwaterloo.ca/tsp/concorde/DOC/concorde_org.html
#include "tsp.h"
#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <queue>
#include <array>
#include <visualization_msgs/Marker.h>

#include "getmap.hpp"

extern "C" {
#include <concorde.h>
}

using namespace std;
// 
int m_mapAvailable = 0;
TSP::TSPSolver m_TSPSolver;
array<double, 2> gridmap_origin_pose;
vector<array<double,2>> middles;
visualization_msgs::Marker m_points, line_strip;
vector<array<double,2>> waypoints;

using namespace std;

array<double, 2> gridmapToworld(double x, double y){
  double wx = (x * 0.05) + gridmap_origin_pose[0];
  double wy = (y * 0.05) + gridmap_origin_pose[1];
  
  return {wx, wy};
}

array<double, 2> getMiddlePosition(int wid, int pose, double size){
  double x = (pose % wid)+(size/2.0);
  double y = (pose / wid)+(size/2.0);
  
  return {x, y};
}

array<double, 2> getPoseTo2D(int wid, int pose){
  double x = pose % wid;
  double y = pose / wid;
  
  return {x, y};  
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
  
  m_points.scale.x = 0.05;
  m_points.scale.y = 0.05;
  
  line_strip.scale.x = 0.03;

  m_points.color.r = 1.0f;
  m_points.color.a = 1.0;
  
  line_strip.color.b=1.0f;
  line_strip.color.a=1.0;
  
  m_points.lifetime = ros::Duration();
}

void writeDataFile (const vector<array<double,2>> &middle_point) {
  ROS_INFO("function write data file");
  m_TSPSolver.name = "/home/ej/catkin_ws/src/find_positions/src/map_3.tsp";
  FILE *out = fopen (m_TSPSolver.name, "w");
  m_TSPSolver.ncount = middle_point.size();
  
  ROS_INFO("%d", m_TSPSolver. ncount);
  ROS_INFO("open data_file and write information");
  fprintf (out, "NAME: concorde%d\n",m_TSPSolver. ncount);
  fprintf (out, "TYPE: TSP\n");
  fprintf (out, "DIMENSION: %d\n", m_TSPSolver.ncount);
  fprintf (out, "EDGE_WEIGHT_TYPE: EUC_2D\n");
  fprintf (out, "NODE_COORD_SECTION\n");
  
  ROS_INFO("write data about middle points");
  for (int i = 0; i < m_TSPSolver.ncount; i ++) {
    fprintf(out, "%d %lf %lf\n", i+1, middle_point[i][0], middle_point[i][1]);
  }
  
  ROS_INFO("End of writing data");
  fprintf(out, "EOF\n");
  fclose(out);
  
  ROS_INFO("done the write task");
}

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg){
  nav_msgs::OccupancyGrid m_gridmap = *msg;
  ROS_INFO("we get the map.");
  
  vector<signed char> m_data = m_gridmap.data;
  int m_resolution = m_gridmap.info.resolution;
  int m_cols= m_gridmap.info.width;
  int m_rows = m_gridmap.info.height;
  gridmap_origin_pose = {m_gridmap.info.origin.position.x, m_gridmap.info.origin.position.y};
  m_mapAvailable = 1;
    
  ROS_INFO("%d Got map %d, %d, size: %d", m_mapAvailable, m_cols, m_rows, m_data.size());
  
  int square_size = 3;
  for(int x = square_size/2.0; x<m_cols; x+=square_size){
    for(int y = square_size/2.0; y<m_rows; y+=square_size){
      int check_pose = (y*m_cols) + x;
      if(m_data[check_pose]==0){
        middles.push_back({x+0.5, y+0.5});
      }
    }
  }
    
  ROS_INFO("Write middle points to tsp file.");
  writeDataFile(middles);
}

//Receive a symmetric 2D distance matrix (dist) and create a TSP optimal tour (tour)
void solving_tsp_concorde(vector<int> * tour){

  //TSP::TSPSolver m_TSPSolver;
  //Creating a sequential tour
  for(int i = 0; i < tour->size(); i++){
    tour->at(i) = i;
  }
  int rval = 0; //Concorde functions return 1 if something fails
  double szeit; //Measure cpu time

  CCrandstate rstate;
  int seed = rand();
  
  CCutil_sprand(seed, &rstate); //Initialize the portable random number generator
  //m_TSPSolver.ncount = tour->size(); //Number of nodes (cities)

  m_TSPSolver.out_tour = CC_SAFE_MALLOC (m_TSPSolver.ncount, int);
  //m_TSPSolver.name = "/home/ej/catkin_ws/src/find_positions/src/map_4.tsp";
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
  //I changed /home/ej/catkin_ws/src/how_to_build_concorde/lib/concorde/TSP/tsp_call.c 
  geometry_msgs::Point p;
  setMarker();
  
  
  cout <<m_TSPSolver.ncount<<endl;
  for (int i = 0; i < m_TSPSolver.ncount; i++) {
    tour->at(i) = m_TSPSolver.out_tour[i];
    array<double, 2> world_position = gridmapToworld(middles[tour->at(i)][0], middles[tour->at(i)][1]);
    p.x=world_position[0]; p.y=world_position[1]; p.z=0.0;
    m_points.points.push_back(p);
    line_strip.points.push_back(p);
  }

  //szeit = CCutil_zeit();
  CC_IFFREE (m_TSPSolver.out_tour, int);
//  CC_IFFREE (m_TSPSolver.name, char);
}

//find positions having a distance between them
void findWaypoints(vector<int> *tour){
  double x=0, y = 0;
  //geometry_msgs::Point p;
  //setMarker();
  //ofstream dataFile("/home/ej/catkin_ws/src/find_positions/data.txt");
  
  for(int i=0; i<tour->size(); i++){
    int turn = tour->at(i);
    array<double, 2> cur_pose = middles[turn];
    //double distance = cacluateDistance(x-cur_pose[0], y-cur_pose[1]);
    

    
    /*
    if((x!=0||y!=0)&&distance<36){
      continue;
    }
    else{
      x = cur_pose[0]; y = cur_pose[1];
      dataFile<< x<< "\t"<<y<<endl;
      
      cout<< "position: "<<x<< ", "<<y<<endl;
      
      waypoints.push_back({x, y});
      //set markers
      //array<double, 2> world_position = gridmapToworld(x, y);
      //p.x=world_position[0]; p.y=world_position[1]; p.z=0.0;
      //m_points.points.push_back(p);
      //ROS_INFO("position: {%f, %f}", x, y);
    }
    */
  }

  //dataFile.close();
}

void makeDistance(){
  geometry_msgs::Point p;
  setMarker();
  
  int pos=0;
  while(1){
    double x = waypoints[pos][0];
    double y = waypoints[pos][1];
    
    for(int i=0; i<waypoints.size(); i++){
      double x_check = waypoints[i][0];
      double y_check = waypoints[i][1]; 
      double dist = cacluateDistance(x-x_check, y-y_check);;
      if(dist<36 && pos!=i){
        waypoints.erase(waypoints.begin()+i);
      }
    }
    pos++;
    if(pos>waypoints.size()) break;
  }
  
  for(int i=0; i<waypoints.size(); i++){
    array<double, 2> world_position = gridmapToworld(waypoints[i][0], waypoints[i][1]);
    p.x=world_position[0]; p.y=world_position[1]; p.z=0.0;
    m_points.points.push_back(p);
  }
  
  
}


int main(int argc, char **argv){
  ros::init(argc, argv, "find_positions");
  ros::NodeHandle m_nh;
  ros::Publisher m_wayPointsPub = m_nh.advertise<visualization_msgs::Marker>("way_point", 10000);
  
  ros::Subscriber m_gridmapsub = m_nh.subscribe("m_gridmap_fin", 1000, mapCallback);
  while(!m_mapAvailable){
    ros::spinOnce();
  }
  //Tour stores the optimal tour
  vector<int> * tour = new vector<int>(m_TSPSolver.ncount , 0);

  //Solve TSP
  solving_tsp_concorde(tour);

  //Print solution
  for(int i = 0; i < tour->size(); i++)
    cout<<tour->at(i)+1<<"_";
  cout << endl;
  
  //Find waypoints
  //findWaypoints(tour);
  
  //make distance
  //makeDistance();
  
  //Print the waypoints on the map
  m_wayPointsPub.publish(m_points);
  
  return 0;
}
