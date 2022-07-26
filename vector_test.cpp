#include <iostream>
#include <vector>
#include <cmath>
#include <array>
#include <queue>

using namespace std;



double distance(int x, int y, int xx, int yy){
  return sqrt(((x-xx)*(x-xx))+((y-yy)*(y-yy)));
}

void bfs(int start, bool* visited, vector<int>* graph, vector<int>& sequence) {
    queue<int> q;
    q.push(start);
    visited[start] = true;

    while (!q.empty()) {
        int x = q.front();
        q.pop();
        cout << x << ' ';
        sequence.push_back(x);
        for (int i = 0; i < graph[x].size(); i++) {
            int y = graph[x][i];
            if (!visited[y]) {
                q.push(y);
                visited[y] = true;
            }
        }
    }
}

int main(){
  vector<vector<array<int,2>>> v;
  vector<array<int,2>> result;
  vector<array<int,2>> waypoints;
  
  //vector<vector>> v;
  //vector<array<int,2>> points = { {0,0}, {2,0}, {2,2}, {2,4}, {4,0}, {4,2}, {4,4}};
  vector<array<int,2>> points = { {0,4}, {0,2}, {0,0}, {2,0}, {2,2}, 
                                  {6,4}, {8,4},
                                  {4,4}, {4,6},
                                  {2,4}};
  int flag = 0;
  v.push_back({points[0]});
  for(int i=0;i<points.size()-1; i++){
    double dist = distance(points[i][0], points[i][1], points[i+1][0], points[i+1][1]);
    if(dist>2.0){
      flag++;
      v.push_back({points[i+1]});
    }
    else{
      v[flag].push_back(points[i+1]);
    }
  }
  
  
  bool visited[v.size()];
  vector<int> graph[v.size()];
  vector<int> sequence;
  
  for(int i=0;i<v.size(); i++){
    for(int j=0;j<v.size(); j++){
      if(i==j) continue;
      for(int k=0;k<v[j].size(); k++){
        double dist_f = distance(v[i][0][0], v[i][0][1], v[j][k][0], v[j][k][1]);
        if(dist_f<=2.0){
          result.push_back({j, i});
          graph[j].emplace_back(i);
          break;
        }
      }
    }
  }

  //result {{j,i}, {j,i}...}
  for(int i=0;i<result.size(); i++){
    cout<<"{ "<<result[i][0]<<", "<<result[i][1]<<" } ";
    cout<<endl;
  }
  bfs(3, visited, graph, sequence);
  
  
  cout<<endl;
  cout<<sequence.size();
  cout<<endl;
  for(int i=0;i<sequence.size(); i++){
    cout<<sequence[i]<<endl;
    for(int k=0;k<v[sequence[i]].size(); k++){
      cout<<"{ "<<v[sequence[i]][k][0]<<", "<<v[sequence[i]][k][1]<<" } ";
    }
    cout<<endl;
  }
  

  return 0;
}



/*

geometry_msgs::Point p;

//p.x=world_position[0]; p.y=world_position[1]; p.z=0.0;
        
      //m_points.points.push_back(p);
      //line_strip.points.push_back(p);


    //m_wayPointsPub.publish(m_points);
    //m_wayPointsPub.publish(line_strip);











*/
