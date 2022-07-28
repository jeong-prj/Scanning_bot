#include "find_positions_class.hpp"

namespace find_positions
{

FindPositions::FindPositions(){
  m_wayPointsPub = m_nh.advertise<visualization_msgs::Marker>("way_point_marker", 10000);
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

array<int, 2> FindPositions::getPoseTo2D(int pose){
  int x = pose % map_width;
  int y = pose / map_width;
  
  return {x, y};  
}

int FindPositions::get2DToPose(int x, int y){
  return (y*map_width) + x;
}

double distance(double x, double y, double xx, double yy){
  return sqrt(((x-xx)*(x-xx))+((y-yy)*(y-yy)));
}

void FindPositions::writeDataFile (vector<array<int,2>> positions) {
  ROS_INFO("function write data file");
  FILE *out = fopen (m_TSPSolver.name, "w");
  m_TSPSolver.ncount = positions.size();
  
  fprintf (out, "NAME: concorde%d\n",m_TSPSolver.ncount);
  fprintf (out, "TYPE: TSP\n");
  fprintf (out, "DIMENSION: %d\n", m_TSPSolver.ncount);
  fprintf (out, "EDGE_WEIGHT_TYPE: EUC_2D\n");
  fprintf (out, "NODE_COORD_SECTION\n");
  
  for (int i = 0; i < m_TSPSolver.ncount; i++) {
    fprintf(out, "%d %d %d\n", i, positions[i][0], positions[i][1]);
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
  //geometry_msgs::Point p;
  
  cout <<m_TSPSolver.ncount<<endl;
  
  if(flag ==1){
    for (int i = 0; i < m_TSPSolver.ncount; i++) {
      int turn= m_TSPSolver.out_tour[i];
      int x=waypoints[turn][0], y=waypoints[turn][1];
      array<double, 2> world_position = gridmapToworld(x,y);
      waypointsWorld.push_back({world_position[0], world_position[1]});
      
      //p.x=world_position[0]; p.y=world_position[1]; p.z=0.0;
        
      //m_points.points.push_back(p);
      //line_strip.points.push_back(p);
    }
    
    //m_wayPointsPub.publish(m_points);
    //m_wayPointsPub.publish(line_strip);
    eraseInvalidByDist();
    //eraseInvalidByWall();
    
  }
  else{
    findWaypointsDistance(m_TSPSolver.out_tour); 
  }
  
  cout<<endl<<"done: "<<flag<<endl;
  
  CC_IFFREE (m_TSPSolver.out_tour, int);
}

void FindPositions::padding(int pose, int siz){
  array<int, 2> grid = getPoseTo2D(pose);
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
  
  this->map_flags = m_gridmap.data;
  this->origin_map = m_gridmap.data;
  this->map_resolution = m_gridmap.info.resolution;
  this->map_width = m_gridmap.info.width;
  this->map_height = m_gridmap.info.height;
  this->gridmap_origin_pose = {m_gridmap.info.origin.position.x, m_gridmap.info.origin.position.y};
  m_mapAvailable = 1;
    
  ROS_INFO("%lf Got map %d, %d, size: %lu", map_resolution, map_width, map_height, origin_map.size());
  
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
  for(int i=0; i<m_TSPSolver.ncount; i++){
    int turn = tour[i];

    array<int, 2> cur_2D = middles[turn];
    int x=cur_2D[0], y=cur_2D[1];
    int pose=get2DToPose(x, y);
    if(map_flags[pose]==0){
      waypoints.push_back({x, y});
      
      padding(pose, 32);
    }
  }
}



int FindPositions::bfs(int node_size, vector<int>* graph, vector<int>& sequence){
  queue<int> q;
  int size=0;
  bool visited[node_size];
  for(int i=0;i<node_size;i++){
    if(!visited[i]&& graph[i].size()>0){
      q.push(i);
      
      while(!q.empty()){
        int vertex=q.front();
        q.pop();
        
        visited[vertex]=true;
        ROS_INFO("size: %d, added node: %d", size, vertex);
        sequence.push_back(vertex);
        size++;
        
        for (int j = 0; j < graph[vertex].size(); j++) {
          int y = graph[vertex][j];
          if (!visited[y]) {
            q.push(y);
            visited[y] = true;
          }
        }
      }
    }
  }
  return size;
}


//ej_invalid_by distance
//waypointsWorld = the result of second TSP with waypoints..
int FindPositions::eraseInvalidByDist(){
  ROS_INFO("dist waypointsWorld's size: %lu", waypointsWorld.size());
  if(waypointsWorld.size()<0){
    ROS_INFO("couldn't erase Invalid path");
    return 0;
  }
  
  //node's waypoins world
  vector<vector<array<double,2>>> v;
  ////vector<array<int,2>> result_invalid;
  ////vector<array<int,2>> tmp_node;
  
  int flag = 0;
  v.push_back({waypointsWorld[0]});
  
  for(int i=0;i<waypointsWorld.size()-1; i++){
    double dist = distance(waypointsWorld[i][0], waypointsWorld[i][1], waypointsWorld[i+1][0], waypointsWorld[i+1][1]);
    if(dist>2.5){
      flag++;
      v.push_back({waypointsWorld[i+1]});
    }
    else{
      v[flag].push_back(waypointsWorld[i+1]);
    }
  }
  
  
  int node_size = v.size();
  bool visited[node_size];
  vector<int> graph[node_size];
  vector<int> sequence;
  
  for(int i=0;i<node_size; i++){
    int found = 0;
    int node = 0;
    double min_dist = DBL_MAX;
    for(int j=0;j<node_size; j++){
      if(i==j) continue;
      for(int k=0;k<v[j].size(); k++){
        double dist_f = distance(v[i][0][0], v[i][0][1], v[j][k][0], v[j][k][1]);
        if(dist_f<=2.5){
          //result_invalid.push_back({j, i});
          cout<<"{ "<<j<<", "<<i<<" } "<<endl;
          graph[j].emplace_back(i);
          found=1;
          break;
        }
        else{
          if(min_dist>dist_f){
            min_dist=dist_f;
            node=j;
          }
        }
      }
    }
    if(found==0){
      cout<<"{ "<<node<<", "<<i<<" } "<<endl;
      ////result_invalid.push_back({node, i});
      ////tmp_node.push_back({node, i});
      graph[node].emplace_back(i);
    }
  }
  /*
  for(int i=0;i<result_invalid.size(); i++){
    ROS_INFO("{ %d, %d }", result_invalid[i][0], result_invalid[i][1]);
  }
  */
  int size = bfs(node_size, graph, sequence);
  
  ROS_INFO("sequence size: %lu", sequence.size());
  
  geometry_msgs::Point p;
  
  for(int i=0;i<sequence.size(); i++){
    ROS_INFO("sequence: %d", sequence[i]);
    for(int k=0;k<v[sequence[i]].size(); k++){
      ROS_INFO("sequence of waypoints: %lf, %lf", v[sequence[i]][k][0], v[sequence[i]][k][1]);
      p.x=v[sequence[i]][k][0]; p.y=v[sequence[i]][k][1]; p.z=0.0;
      result_waypoints.push_back(v[sequence[i]][k]);
      m_points.points.push_back(p);
      line_strip.points.push_back(p);
    }
  }  

  m_wayPointsPub.publish(m_points);
  m_wayPointsPub.publish(line_strip);
  return 1;
}

/*
void FindPositions::checkTheWall(int x, int y, int xx, int yy){
  for(int i=x;i<xx;i++){
    
  }
  
}

int FindPositions::eraseInvalidByWall(){
  ROS_INFO("wall waypointsWorld's size: %d", waypointsWorld.size());
  if(waypointsWorld.size()<0){
    ROS_INFO("couldn't erase Invalid path");
    return 0;
  }
  vector<vector<array<double,2>>> v;
  
  int flag = 0;
  v.push_back({waypointsWorld[0]});
  
  for(int i=0;i<waypointsWorld.size()-1; i++){
    double dist = distance(waypointsWorld[i][0], waypointsWorld[i][1], waypointsWorld[i+1][0], waypointsWorld[i+1][1]);
    if(dist>2.5){
      flag++;
      v.push_back({waypointsWorld[i+1]});
    }
    else{
      v[flag].push_back(waypointsWorld[i+1]);
    }
  }

  //just check the cutting way points..
  for(int i=0;i<v.size(); i++){
    for(int j=0;j<v[i].size(); j++){
      cout<<"{ "<<v[i][j][0]<<", "<<v[i][j][1]<<" } ";
    }
    cout<<endl;
  }

  return 1;
}
*/

}
