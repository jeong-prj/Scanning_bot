//This file solves a TSP using concorde given a symmetric 2D distance matrix

//Info about concorde functions: www.math.uwaterloo.ca/tsp/concorde/DOC/concorde_org.html
#include "find_positions_class.hpp"

namespace find_positions
{

FindPositions::FindPositions(){
  m_wayPointsPub = m_nh.advertise<visualization_msgs::Marker>("way_point", 10000);
  m_gridmapsub = m_nh.subscribe<nav_msgs::OccupancyGrid>("m_gridmap_fin", 1000, &FindPositions::mapCallback, this); 
}
FindPositions::~FindPositions(){
  
}

array<double, 2> FindPositions::gridmapToworld(int x, int y){
  //0.05 is resolution of the map
  double wx = (x * map_resolution) + gridmap_origin_pose[0];
  double wy = (y * map_resolution) + gridmap_origin_pose[1];
  
  return {wx, wy};
}

array<double, 2> FindPositions::getPoseTo2D(int pose){
  double x = pose % map_width;
  double y = pose / map_width;
  
  return {x, y};  
}

int FindPositions::get2DToPose(double x, double y){
  return (y*map_width) + x;
}

void FindPositions::writeDataFile (vector<array<double,2>> positions) {
  ROS_INFO("function write data file");
  FILE *out = fopen (m_TSPSolver.name, "w");
  m_TSPSolver.ncount = positions.size();
  
  fprintf (out, "NAME: concorde%d\n",m_TSPSolver.ncount);
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
////void FindPositions::solving_tsp_concorde(vector<int> * tour, int flag){
void FindPositions::solving_tsp_concorde(int flag){
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
  
  ////cout <<m_TSPSolver.ncount<<"tour.size: "<<tour->size()<<endl;
  cout <<m_TSPSolver.ncount<<endl;
  //Creating a sequential tour
  //for(int i = 0; i < m_TSPSolver.ncount; i++){
  //  tour->push_back(i);
  //}
  //cout <<tour->size()<<endl;
  if(flag ==1){
    for (int i = 0; i < m_TSPSolver.ncount; i++) {
      ////tour->at(i) = m_TSPSolver.out_tour[i];
      
      int turn= m_TSPSolver.out_tour[i];
      double x=waypoints[turn][0], y=waypoints[turn][1];
      array<double, 2> world_position = gridmapToworld(x,y);
      waypointsWorld.push_back({world_position[0], world_position[1]});
      
      p.x=world_position[0]; p.y=world_position[1]; p.z=0.0;
        
      m_points.points.push_back(p);
      line_strip.points.push_back(p);
    }
    
    m_wayPointsPub.publish(m_points);
    m_wayPointsPub.publish(line_strip);
  }
  else{
    findWaypointsDistance(m_TSPSolver.out_tour); 
  }
  
  cout<<endl<<"done: "<<flag<<endl;
  
  //szeit = CCutil_zeit();
  CC_IFFREE (m_TSPSolver.out_tour, int);
//  CC_IFFREE (m_TSPSolver.name, char);
}

void FindPositions::padding(int pose, int siz){
  array<double, 2> grid = getPoseTo2D(pose);
  for(int i=grid[0]-siz;i<=grid[0]+siz;i++){
    for(int j=grid[1]-siz;j<=grid[1]+siz;j++){
      if(i<0||i>=map_width||j<0||j>=map_height){
        continue;
      }
      int flag_pose=get2DToPose(i, j);
      if(origin_map[flag_pose]==0){
        map_flags[flag_pose]=2;
      }
    }
  }
}

void FindPositions::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg){
  nav_msgs::OccupancyGrid m_gridmap = *msg;
  ROS_INFO("map callback");
  
  map_flags = m_gridmap.data;
  origin_map = m_gridmap.data;
  map_resolution = m_gridmap.info.resolution;
  map_width = m_gridmap.info.width;
  map_height = m_gridmap.info.height;
  gridmap_origin_pose = {m_gridmap.info.origin.position.x, m_gridmap.info.origin.position.y};
  m_mapAvailable = 1;
    
  ROS_INFO("%d Got map %d, %d, size: %d", map_resolution, map_width, map_height, origin_map.size());
  
  for(int i=0; i<origin_map.size();i++){
    if(origin_map[i]!=0){
      padding(i, 9);
    }
  }
  
  int square_size = 3;
  for(int x = square_size/2; x<map_width; x+=square_size){
    for(int y = square_size/2; y<map_height; y+=square_size){
      int check_pose = get2DToPose(x, y);
      
      if(map_flags[check_pose] == 0){
        middles.push_back({x, y});
      }
    }
  }
}

////void FindPositions::findWaypointsDistance(vector<int> *tour){
void FindPositions::findWaypointsDistance(int* tour){
  ////for(int i=0; i<tour->size(); i++){
  ////  int turn = tour->at(i);
  for(int i=0; i<m_TSPSolver.ncount; i++){
    int turn = tour[i];

    array<double, 2> cur_2D = middles[turn];
    double x=cur_2D[0], y=cur_2D[1];
    int pose=get2DToPose(x, y);
    if(map_flags[pose]==0){
      //cout<< i<<"position"<<turn<<": "<<x<< ", "<<y<<endl;
      waypoints.push_back({x, y});
      
      padding(pose, 38);
    }
  }
}

}
