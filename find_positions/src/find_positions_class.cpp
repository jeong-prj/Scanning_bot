#include "find_positions_class.hpp"
#include <fstream>
#include <limits>

/***************************************
 * -  update autoexplorer, navigation in virtual environments first
 * 
 * 1. A* planning for all nodes to weight
 *    over the 3m than set the weight to infinity
 * 
 * 2. 
 * 
 * 
 * *************************************/



namespace find_positions
{

FindPositions::FindPositions(){
  m_wayPointsPub = m_nh.advertise<visualization_msgs::Marker>("way_point_marker", 10000);
  m_tsp1PointsPub = m_nh.advertise<visualization_msgs::Marker>("tsp1_marker", 10000);
  m_tsp2PointsPub = m_nh.advertise<visualization_msgs::Marker>("tsp2_marker", 10000);
  m_tsp1linePub = m_nh.advertise<visualization_msgs::Marker>("tsp1_line", 10000);
  m_tsp2linePub = m_nh.advertise<visualization_msgs::Marker>("tsp2_line", 10000);
  m_addPointsPub = m_nh.advertise<visualization_msgs::Marker>("added_marker", 10000);
  m_pathPub = m_nh.advertise<visualization_msgs::Marker>("path_marker", 10000);
  m_gridmapsub = m_nh.subscribe<nav_msgs::OccupancyGrid>("m_gridmap_fin", 1000, &FindPositions::mapCallback, this); 
  m_globalCostmapSub = m_nh.subscribe("move_base/global_costmap/costmap", 100, &FindPositions::globalCostmapCallBack, this );
  m_startPoseSub = m_nh.subscribe("start_pose", 2, &FindPositions::poseCallback, this);
  
  //if (mp_cost_translation_table == NULL){
    mp_cost_translation_table = new uint8_t[101];

    // special values:
    mp_cost_translation_table[0] = 0;  // NO obstacle
    mp_cost_translation_table[99] = 253;  // INSCRIBED obstacle
    mp_cost_translation_table[100] = 254;  // LETHAL obstacle
//   mp_cost_translation_table[-1] = 255;  // UNKNOWN

    // regular cost values scale the range 1 to 252 (inclusive) to fit
    // into 1 to 98 (inclusive).
    for (int i = 1; i < 99; i++){
      mp_cost_translation_table[ i ] = uint8_t( ((i-1)*251 -1 )/97+1 );
    }
  //}
  
  //for (int i = 0; i < 100; i++){
  //  ROS_INFO("mp_cost_translation_table[val]: %d  \n", mp_cost_translation_table[i]);
  //}
}
FindPositions::~FindPositions(){
  delete [] mp_cost_translation_table;
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

void FindPositions::writeDataFile (vector<array<int,2>> positions, int mode, int exclude_n) {
  ROS_INFO("function write data file");
  FILE *out = fopen (m_TSPSolver.name, "w");
  ROS_INFO("ncount: %lu %d", positions.size(), exclude_n);
  m_TSPSolver.ncount = positions.size()-exclude_n;
  
  ROS_INFO("ncount: %d", m_TSPSolver.ncount);
  fprintf (out, "NAME: concorde%d\n",m_TSPSolver.ncount);
  fprintf (out, "TYPE: TSP\n");
  fprintf (out, "DIMENSION: %d\n", m_TSPSolver.ncount);
  if(mode == 0){
    fprintf (out, "EDGE_WEIGHT_TYPE: EUC_2D\n");
    fprintf (out, "NODE_COORD_SECTION\n");
    for (int i = 0; i < m_TSPSolver.ncount; i++) {
      fprintf(out, "%d %d %d\n", i, positions[i][0], positions[i][1]);
    }
  }
  else{
    fprintf (out, "EDGE_WEIGHT_TYPE: EXPLICIT\n");
    fprintf (out, "EDGE_WEIGHT_FORMAT: LOWER_DIAG_ROW\n");
    fprintf (out, "EDGE_WEIGHT_SECTION\n");
    for (int i = 0; i < m_TSPSolver.ncount; i++) {
      double xa = tsp2nodes[i][0], ya = tsp2nodes[i][1];
      for(int j = 0; j<=i; j++){
        if(j==i){
          fprintf(out, "%d\n", 0);
        }
        else{
          double xb = tsp2nodes[j][0], yb = tsp2nodes[j][1];
          //ROS_INFO("xb: %lf, yb: %lf ", xb, yb);
          //**set value properly
          double path_l=10000000;
          
          if(distance(xa, ya, xb, yb)<=4 || (j == i-1) ){
            double real_path = calculatePath(xa, ya, xb, yb, 0);  
            path_l = real_path<=4? real_path:real_path*30;
            ROS_INFO("writing %d to %d: %d", i, j, (int)(path_l));
          }
          fprintf(out, "%d ", (int)(path_l));
          //ROS_INFO("%lf length of path plan", calculatePath(xa, ya, xb, yb, 0));
          
        }
      }
    }
  }

  fprintf(out, "EOF\n");
  fclose(out);
  
  ROS_INFO("done the write task");
}

int FindPositions::removeUnreach(vector<array<int,2>> positions){
  ROS_INFO("Remove unreached nodes");
  int removed = 0;
  
  //It needs to check connecting all nodes?
  
  vector<array<double, 2>> tmpnodes;
  int positions_len = positions.size();
  for (int i = 0; i < positions_len; i++) {
    tmpnodes.push_back(gridmapToworld(positions[i][0], positions[i][1]));
  }
  int first_node = findFirstNode(tmpnodes);
  
  //Save all the A* planning results to vector somewhere
  //Remove the node if couldn't reach to it from other nodes
  //Then write data to the file with rearranged nodes
  for (int i = 0; i < positions_len; i++) {
    int cur_n = (first_node + i) % positions_len;
    tsp2nodes.push_back(tmpnodes[cur_n]);
  }
  
  //Not this way 
  //if cant find path from cur position than those are unreached positions.
  double xa = tsp2nodes[0][0], ya = tsp2nodes[0][1];
  ROS_INFO("xa: %lf,ya: %lf ", xa, ya);
  
  double longest_l = 0;
  int longest_i=0;
  for (int i = 1; i < positions_len; i++) {
    ROS_INFO("%d node for check path", i);
    
    double xb = tsp2nodes[i][0], yb = tsp2nodes[i][1];
    ROS_INFO("xb: %lf, yb: %lf ", xb, yb);

    double path_l = calculatePath(xa, ya, xb, yb, 0);
    if(path_l>0){
      if(longest_l<path_l){
        longest_l = path_l;
        longest_i = i;
      }
      continue;
    }
    else{
      tsp2nodes.erase(tsp2nodes.begin()+i);
      positions_len--;
      i--;
      removed++;
    }
  }
  
  array<double, 2> end_pose = tsp2nodes[longest_i];
  tsp2nodes.erase(tsp2nodes.begin()+longest_i);
  tsp2nodes.push_back(end_pose);
  
  return removed;
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
    writeDataFile(middles, 0, 0);
  else{
    makeMapForPath(0);
    int removed_num = removeUnreach(waypoints);
    writeDataFile(waypoints, 1, removed_num);
  }
  
  CCrandstate rstate;
  int seed = rand();
  CCutil_sprand(seed, &rstate); //Initialize the portable random number generator
  m_TSPSolver.out_tour = CC_SAFE_MALLOC (m_TSPSolver.ncount, int);
  
  CCdatagroup dat;
  ROS_INFO("solving ncount: %d", m_TSPSolver.ncount);
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
  //
  
  cout <<m_TSPSolver.ncount<<endl;
  
  if(flag ==1){
    for (int i = 0; i < m_TSPSolver.ncount; i++) {
      int turn= m_TSPSolver.out_tour[i];
      ROS_INFO("%d_",turn);
      //int x=waypoints[turn][0], y=waypoints[turn][1];
      //array<double, 2> world_position = gridmapToworld(x,y);
      waypointsWorld.push_back({tsp2nodes[turn][0], tsp2nodes[turn][1]});
      geometry_msgs::Point p;
      p.x=tsp2nodes[turn][0]; p.y=tsp2nodes[turn][1]; p.z=0.0;
        
      tsp2_points.points.push_back(p);
      t2_line_strip.points.push_back(p);
    }
    ROS_INFO("tsp2 points");
    m_tsp2PointsPub.publish(tsp2_points);
    m_tsp2linePub.publish(t2_line_strip);
    
    clock_t start_0=clock();
    while((clock()-start_0)/CLOCKS_PER_SEC <= 10) ;
  
    
    //m_wayPointsPub.publish(t_line_strip);
    ////eraseInvalidByDist();
    //eraseInvalidByWall();
    checkRealPath();
    
  }
  else{
    for (int i = 0; i < m_TSPSolver.ncount; i++) {
      int turn= m_TSPSolver.out_tour[i];
      int x=middles[turn][0], y=middles[turn][1];
      array<double, 2> world_position = gridmapToworld(x,y);
      geometry_msgs::Point p;
      p.x=world_position[0]; p.y=world_position[1]; p.z=0.0;
        
      tsp1_points.points.push_back(p);
      t1_line_strip.points.push_back(p);
    }
    ROS_INFO("tsp1 points");
    m_tsp1PointsPub.publish(tsp1_points);
    m_tsp1linePub.publish(t1_line_strip);
    
    clock_t start_1=clock();
    while((clock()-start_1)/CLOCKS_PER_SEC <= 15) ;
    
    findWaypointsDistance(m_TSPSolver.out_tour); 
  }
  
  cout<<endl<<"done: "<<flag<<endl;
  
  CC_IFFREE (m_TSPSolver.out_tour, int);
  
}

void FindPositions::padding(int pose, int siz, int ch){
  array<int, 2> grid = getPoseTo2D(pose);
  for(int i=grid[0]-siz;i<=grid[0]+siz;i++){
    for(int j=grid[1]-siz;j<=grid[1]+siz;j++){
      if(i<0||i>=map_width||j<0||j>=map_height){
        continue;
      }
      int flag_pose=get2DToPose(i, j);
      if(origin_map[flag_pose]==0){
        if(ch==1){
          map_flags[flag_pose]=2;
        }
        else if(ch==2){
          map_for_add[flag_pose]=254;
        }
        else{
          map_for_path[flag_pose]=254;
        }
        
      }
    }
  }
}

void FindPositions::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg){
  nav_msgs::OccupancyGrid m_gridmap = *msg;
  ROS_INFO("map callback");
  
  this->map_flags = m_gridmap.data;
  this->origin_map = m_gridmap.data;
  
  this->map_for_add = m_gridmap.data;
  this->map_for_path = m_gridmap.data;
  
  this->map_resolution = m_gridmap.info.resolution;
  this->map_width = m_gridmap.info.width;
  this->map_height = m_gridmap.info.height;
  this->gridmap_origin_pose = {m_gridmap.info.origin.position.x, m_gridmap.info.origin.position.y};
  m_mapAvailable = 1;
  
  //start_pose
  
  tf::TransformListener listener;
  tf::StampedTransform transform;
  string mapFrameId = "map";
  string baseFrameId = "base_link";
  
  try{
    listener.waitForTransform(mapFrameId, baseFrameId, ros::Time(0), ros::Duration(3.0));
    listener.lookupTransform( mapFrameId, baseFrameId, ros::Time(0), transform);
  }
  
  catch (tf::TransformException &ex) {
    ROS_ERROR("%s",ex.what());
    ros::Duration(1.0).sleep();
  }
  
  start_pose[0] = transform.getOrigin().x();
  start_pose[1] = transform.getOrigin().y();
  ROS_INFO("Find position: %lf, %lf", start_pose[0], start_pose[1] );  
  m_sposeAvailable=1;
  
  //startpose ej
  //m_sposeAvailable=1;
  
  
  ROS_INFO("%lf Got map %d, %d, size: %lu", map_resolution, map_width, map_height, origin_map.size());
  
  for(int i=0; i<origin_map.size();i++){
    if(origin_map[i]!=0){
      padding(i, 9, 1);
      padding(i, 5, 2);
      padding(i, 4, 3);
    }
  }
  
  //5 or 6 nice?
  int square_size = 7;
  for(int x = square_size/2; x<map_width; x+=square_size){
    for(int y = square_size/2; y<map_height; y+=square_size){
      int check_pose = get2DToPose(x, y);
      
      if(map_flags[check_pose] == 0){
        
        middles.push_back({x, y});
      }
    }
  }
}

void FindPositions::globalCostmapCallBack(const nav_msgs::OccupancyGrid::ConstPtr& msg){
	//const std::unique_lock<mutex> lock(mutex_costmap);
//ROS_INFO("cm callback is called \n");
	m_globalcostmap = *msg ;
	m_globalcostmap_rows = m_globalcostmap.info.height ;
	m_globalcostmap_cols = m_globalcostmap.info.width ;
  //ROS_INFO("get starting position {%d, %d}" , m_globalcostmap_rows, m_globalcostmap_cols);
}

void FindPositions::poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
  ROS_INFO("+++++++++++++++++++++++++++++++++++++++++++++++++");
  m_sposeAvailable=1;
  this->start_pose[0]= msg->pose.position.x;
  this->start_pose[1]= msg->pose.position.y;
  ROS_INFO("get starting position {%lf, %lf}" , start_pose[0], start_pose[1]);
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
      
      padding(pose, 36, 1);
    }
  }
}

//ej_invalid_by distance
//waypointsWorld = the result of second TSP with waypoints..
//need to fix it !!!!!!!!!!!

//calculate length of path
//return length

void FindPositions::makeMapForPath(int mode){
  ROS_INFO("make map for path");
  std::vector<signed char> cmdata;
  cmdata  =m_globalcostmap.data;
  
  mpo_costmap = new costmap_2d::Costmap2D();
  
  this->mpo_costmap->resizeMap( this->map_width, this->map_height, this->map_resolution, this->gridmap_origin_pose[0], this->gridmap_origin_pose[1] );
  ROS_INFO("mpo_costmap has been reset \n");
  
  unsigned char* pmap = this->mpo_costmap->getCharMap() ;
  
  ROS_INFO("w h datlen : %d %d %d \n", map_width, map_height, cmdata.size() );
  
  for(uint32_t ridx = 0; ridx < this->map_height; ridx++){
    for(uint32_t cidx=0; cidx < this->map_width; cidx++){
      uint32_t idx = ridx * this->map_width + cidx ;
      if(mode==0){
        signed char val = this->map_for_add[idx];
        pmap[idx] = val < 0 ? 255 : mp_cost_translation_table[val];
      }
      if(mode==1){
        signed char val = this->map_for_path[idx];
        pmap[idx] = val < 0 ? 255 : mp_cost_translation_table[val];
      }
    }
  }
  
  ROS_INFO("done map path");
}

double FindPositions::calculatePath(double ax, double ay, double bx, double by,int mode){
  ROS_INFO("Calculate len..");
  GlobalPlanningHandler o_gph( *mpo_costmap, "map", "base_link");
  o_gph.reinitialization();
  
  std::vector<geometry_msgs::PoseStamped> plan;
  
  geometry_msgs::PoseStamped start = StampedPosefromSE2( ax, ay, 0.f );
  start.header.frame_id = "map";
  
  geometry_msgs::PoseStamped goal = StampedPosefromSE2( bx, by, 0.f );
  goal.header.frame_id = "map" ;

  bool bplansuccess = o_gph.makePlan(start, goal, plan);
  ROS_INFO("Plan success: %d", bplansuccess);
  //caclulate path length..//
  if(!bplansuccess){
    ROS_INFO("Couldnt find a path..");
    return 0.0;
  }
  
  double path_len=0;
  int add_node=0;
  geometry_msgs::Point p;
  //if(mode==1){
  if(mode==2){//path marker
    p.x=plan[0].pose.position.x; p.y=plan[0].pose.position.y; p.z=0.0;
    m_path_points.points.push_back(p);
  }
  for(int k=0;k<plan.size()-1;k++){
    double pose_ax=plan[k].pose.position.x;
    double pose_ay=plan[k].pose.position.y;
    double pose_bx=plan[k+1].pose.position.x;
    double pose_by=plan[k+1].pose.position.y;
        
    path_len += distance(pose_ax, pose_ay, pose_bx, pose_by);
    
    if(mode==1){
      if(path_len>=2.0){
        //result_waypoints.push_back({pose_bx, pose_by});
        add_node++;
        path_len=0;
      }
      //p.x=pose_bx; p.y=pose_by; p.z=0.0;
      //m_path_points.points.push_back(p);
    }
    if(mode==2){
      p.x=pose_bx; p.y=pose_by; p.z=0.0;
      m_path_points.points.push_back(p);
      
    }
  }
  
  if(mode==1&&add_node>0){
    //int equi_dist=plan.size()/(add_node+1);
    double equi_dist=(path_len+(add_node*2.0))/(add_node*1.0);
    //for(int i=1;i<=add_node;i++){
    double check_len=0;
    for(int i=0; i<plan.size()-1;i++){
      double pose_ax=plan[i].pose.position.x;
      double pose_ay=plan[i].pose.position.y;
      double pose_bx=plan[i+1].pose.position.x;
      double pose_by=plan[i+1].pose.position.y;
      
      check_len += distance(pose_ax, pose_ay, pose_bx, pose_by);
      //double add_x=plan[equi_dist*i].pose.position.x;
      //double add_y=plan[equi_dist*i].pose.position.y;
      if(check_len>= equi_dist){
        check_len=0;
        int not_add=0;
          
        double add_x= pose_bx;
        double add_y= pose_by;
        for(int ch_node=0;ch_node<result_waypoints.size();ch_node++){
          double ch_x = result_waypoints[ch_node][0];
          double ch_y = result_waypoints[ch_node][1];
          if(distance(add_x, add_y, ch_x, ch_y)<1.1){
            double ch_path = calculatePath(add_x, add_y, ch_x, ch_y, 0);
            if(ch_path<1.5){
              not_add=1;
              break;
            }
          }
        }
        if(not_add==0){
          result_waypoints.push_back({add_x, add_y});
          p.x=add_x; p.y=add_y; p.z=0.0;
          add_points.points.push_back(p);
        }
      }
    }
  }
  
  return mode!=1? path_len : path_len+(add_node*2.0);
}

int FindPositions::findFirstNode(vector<array<double,2>> nodes){
  ROS_INFO("Function find first node..");
  ROS_INFO("nearby.. %lf %lf", start_pose[0], start_pose[1]);
  int node_idx = -1;
  double nearest_path = DBL_MAX;
  //for(int ch_node=0;ch_node<waypointsWorld.size();ch_node++){
  for(int ch_node=0;ch_node<nodes.size();ch_node++){
    //double dist_nn = distance(waypointsWorld[ch_node][0], waypointsWorld[ch_node][1], start_pose[0], start_pose[1]);
    double dist_nn = distance(nodes[ch_node][0], nodes[ch_node][1], start_pose[0], start_pose[1]);
    if(dist_nn<2.0){
      ROS_INFO("found_node: %d", ch_node);
      //double ch_path = calculatePath(waypointsWorld[ch_node][0], waypointsWorld[ch_node][1], start_pose[0], start_pose[1], 0);
      double ch_path = calculatePath(nodes[ch_node][0], nodes[ch_node][1], start_pose[0], start_pose[1], 0);
      if(ch_path<nearest_path && ch_path!=0){
      //if(dist_nn<nearest_path){
        nearest_path=dist_nn;
        node_idx = ch_node;
      }
    }
  }
  ROS_INFO("start_node: %d", node_idx);
  return node_idx;
}

void FindPositions::checkRealPath(){
  ROS_INFO("Function: check real path %lu", waypointsWorld.size());
  int start_node=0;
  int mode=0;
  
  double max_len=0;
  int idx_s = 0;
  int idx_e = 0;
  for(int i=0;i<waypointsWorld.size(); i++){
    int next_i= i < waypointsWorld.size()-1 ? i+1 : 0;
    double path_len_tmp = calculatePath(waypointsWorld[i][0], waypointsWorld[i][1],
                          waypointsWorld[next_i][0], waypointsWorld[next_i][1], 0);
    ROS_INFO("result: %d to %d = %lf", i, next_i, path_len_tmp);
    if(max_len<path_len_tmp){
      max_len = path_len_tmp;
      idx_s = i;
      idx_e = next_i;
    }
  }
  
  double path_len_s = calculatePath(start_pose[0], start_pose[1],
                            waypointsWorld[idx_s][0], waypointsWorld[idx_s][1], 0);
  double path_len_e = calculatePath(start_pose[0], start_pose[1],
                            waypointsWorld[idx_e][0], waypointsWorld[idx_e][1], 0);  
  
  if(path_len_s <= path_len_e ){
    //start with idx_s
    //need to reverse the sequence
    mode = 1;
    start_node= idx_s;
  }
  else{
    start_node= idx_e;
  }
  
  unsigned long way_size=waypointsWorld.size();
  for(int i=0;i<way_size-1; i++){
    int cur_i = mode==0 ? (start_node + i) % way_size:(start_node - i + way_size) % way_size;
    int next_i= mode==0 ? (cur_i + 1) % way_size:(cur_i-1+way_size) % way_size;
    ROS_INFO("path %d to %d..", cur_i, next_i);
    double path_len = calculatePath(waypointsWorld[cur_i][0], waypointsWorld[cur_i][1],
                                    waypointsWorld[next_i][0], waypointsWorld[next_i][1], 1);
    if(path_len==0.0){
      ROS_INFO("path is not found..");
      //do it something.
    }
    else{
      int not_add=0;
      //test_ej
      for(int ch_node=0;ch_node<result_waypoints.size();ch_node++){
        double ch_x = result_waypoints[ch_node][0];
        double ch_y = result_waypoints[ch_node][1];
        if(distance(waypointsWorld[next_i][0], waypointsWorld[next_i][1], ch_x, ch_y)<1.1){
          double ch_path = calculatePath(waypointsWorld[next_i][0], waypointsWorld[next_i][1], ch_x, ch_y, 0);
          if(ch_path<1.5){
            not_add=1;
            break;
          }
        }
      }
      if(not_add!=1){
        result_waypoints.push_back(waypointsWorld[next_i]);
      }
    }
  }
  
  makeMapForPath(1);
  ROS_INFO("result_waypoints: size:: %lu", result_waypoints.size());
  for(int i=0;i<result_waypoints.size(); i++){
    geometry_msgs::Point p;
      
    p.x=result_waypoints[i][0]; p.y=result_waypoints[i][1]; p.z=0.0;
    m_points.points.push_back(p);
    line_strip.points.push_back(p);
    
    if(i < result_waypoints.size()-1){
      double path_len_tmp = calculatePath(result_waypoints[i][0], result_waypoints[i][1],
                            result_waypoints[i+1][0], result_waypoints[i+1][1], 2);
      ROS_INFO("result: %d to %d = %lf", i, i+1, path_len_tmp);
    }
  }
  

  
  ////ej_visual
  /*
  * path marker
  ROS_INFO("path print");
  m_pathPub.publish(m_path_points);
  
  while((clock()-start_2)/CLOCKS_PER_SEC <= 10) ;
  */
  ROS_INFO("add points");
  m_addPointsPub.publish(add_points);
  clock_t start_1=clock();
  while((clock()-start_1)/CLOCKS_PER_SEC <= 5) ;
  
  ROS_INFO("result of sequence");
  m_wayPointsPub.publish(m_points);
  m_wayPointsPub.publish(line_strip);
  clock_t start_2=clock();
  while((clock()-start_2)/CLOCKS_PER_SEC <= 5) ;
  
  ROS_INFO("path print");
  m_pathPub.publish(m_path_points);
  
  //ROS_INFO("Sequence: %d, %d\n", i, next_idx);
}


}
