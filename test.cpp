#include <iostream>
#include <algorithm>
#include <array>
#include <cstdlib>
#include <queue>

using namespace std;

#define CANT 0
#define FREE 1
#define VISI 2
#define SIZE 2
//                        up r dow  l 
array<int, 4> x_direc = { 0, 1, 0, -1};
array<int, 4> y_direc = {-1, 0, 1,  0};


/*
problems
- too many data approach
- some cell is suddenly 1 -> 0 ??
- no efficiency
*/

void mapDisplay(int map[], int w, int h){
    for(int i=0; i<(w*h); i++){
        if((i%w) == 0) cout << endl;
        cout << map[i] << " " ;
    }
    cout << endl;
}

int getPosition(int wid, int x, int y){
    return x + (wid * y);
}

array<int, 2> checkCorner(int map[], int wid, int x, int y){
    array<int, 2> corner = {0, 0};
    for(int i=0; i<4; i++){
        int pose = getPosition(wid, x+x_direc[i], y+y_direc[i]);
        if(map[pose] == FREE){
            if(i%2 ==0) corner[1] = y_direc[i];
            else corner[0] = x_direc[i];
        }
    }
    if(corner[0]==0 && corner[1]==0){
        cout<<"There is no corner."<<endl;
        return {0, 0};
    }
    return corner;
}

void addCandidateToQueue(int map[], queue<array<int, 2>> &candidates, int wid, int x, int y){
    int pose = getPosition(wid, x, y);
    if(map[pose]==FREE){
        candidates.push({x, y});
    }
}

array<int,2> sortValue(int start, int end){
    array<int,2> order = {-1, -1};
    
    order[0] = start<end? start:end;
    order[1] = start<end? end:start;
    
    return order;
}

void findNextCells(int map[], queue<array<int, 2>> &candidates, int wid, int s_x, int s_y, int e_x, int e_y){
    int pose = 0;
    
    array<int,2> x = sortValue(s_x, e_x);
    array<int,2> y = sortValue(s_y, e_y);
    
    for(int i=x[0]; i<=x[1]; i++){
        addCandidateToQueue(map, candidates, wid, i, y[0]-1);
        addCandidateToQueue(map, candidates, wid, i, y[1]+1);
    }
    
    for(int j=y[0]; j<=y[1]; j++){
        addCandidateToQueue(map, candidates, wid, x[0]-1, j);
        addCandidateToQueue(map, candidates, wid, x[1]+1, j);
    }
}

array<int, 2> findRect(int map[], int wid, int x, int y){
    array<int, 2> corner = checkCorner(map, wid, x, y);
    
    int size_x = 0;
    int size_y = 0;
    
    array<int, 2> done = {0, 0};
    
    for(int i=0; i<SIZE; i++){
        if(done[1]!=1) size_y++;
        if(done[0]!=1) size_x++;
        
        if(done[0] ==1 && done[1] ==1) break;
        
        int check_x = x + (i*corner[0]);
        int check_y = y + (i*corner[1]);
        
        for(int t=0; t<size_y; t++){
            if(done[0] ==1) break;
            
            int pos_x = getPosition(wid, check_x, y+(t*corner[1]));
            if(corner[0] == 0){
                done[0] = 1;
                cout<<"dosent need x diffrent y"<<endl;
            }
            
            else if(t==i){
                cout<<"t==i , prevent double check"<<endl;
            }
            
            //if there are free space
            else if(map[pos_x] == FREE && done[0] == 0){
                map[pos_x] = VISI;
            }
            
            else if(map[pos_x] != FREE && done[0] == 0){
                done[0] = 1;
                if(t != 0){
                    for(int b=0; b<t; b++){
                        int ex_x_pose = getPosition(wid, check_x, y+((t-b-1)*corner[1]));
                        if(map[ex_x_pose]==VISI){
                            map[ex_x_pose] = FREE;
                        }
                    }
                }
                size_x -=1;
            }
        }
        
        for(int t=0; t<size_x; t++){
            if(done[1] ==1) break;
            
            int pos_y = getPosition(wid, x+(t*corner[0]), check_y);
            
            if(corner[1] == 0){
                if(t==i) map[pos_y] = VISI;
                done[1] = 1;
                cout<<"dosent need y diffrent x"<<endl;
            }
            else if(map[pos_y] == FREE && done[1] == 0){
                map[pos_y] = VISI;
            }
            else if(map[pos_y] != FREE && done[1] == 0 ){
                done[1] = 1;
                
                if(t != 0){
                    for(int b=0; b<t; b++){
                        int ex_y_pose = getPosition(wid, x+((t-b-1)*corner[0]), check_y);
                        if(map[ex_y_pose] == VISI){
                            map[ex_y_pose] = FREE;
                        }
                    }
                }
                size_y -= 1;
            }
        }
    }
    
    cout << "size: "<<size_x << " " <<size_y<<endl;
    return {x+(size_x*corner[0])+(corner[0]*(-1)), y+(size_y*corner[1])+(corner[1]*(-1))};
}

array<float,2> findMiddle(int map[], int x[], array<int, 2> &y){
    array<float, 2> middle = {-1, -1};
    
    middle[0] = x[0] == x[1] ? x[0]: (x[0]+x[1])/2.0;
    middle[1] = y[0] == y[1] ? y[0]: (y[0]+y[1])/2.0;
    
    return middle;
}

int main(){
    const int w = 6;
    const int h = 4;
    
    int map[] = {0,1,1,0,1,1,
                 0,1,1,0,1,1,
                 0,1,1,1,1,1,
                 0,1,1,1,1,1};
    
    queue <array<int, 2>> candidates;
    vector<array<float,2>> middles; 
    
    //start x, y
    int s_point[2] = {0, 0};
    
    int flag = 0;
    for(int i=0; i<h; i++){//y
        for(int j=0; j<w; j++){//x
            int pose = getPosition(w, j, i);
            if(map[pose]== FREE){
                s_point[0] = j; s_point[1] = i;//first position
                flag = 1;
                break;
            }
            else
                continue;
        }
        if(flag == 1) break;
    }
    
    while(1){
        cout<<"start_x: "<<s_point[0]<<", start_y: "<< s_point[1]<<endl;
        array<int, 2> e_point = findRect(map, w, s_point[0], s_point[1]);
        
        cout<<"end_x: "<<e_point[0]<<" end_y: "<<e_point[1]<<endl;
        cout << "result";
        mapDisplay(map, w, h);
        cout<<endl;
        
        middles.push_back(findMiddle(map, s_point, e_point));
        
        findNextCells(map, candidates, w, s_point[0], s_point[1], e_point[0], e_point[1]);
        if(candidates.empty()) break;
        
        while(1){
            array<int, 2> mem = candidates.front();
            s_point[0] = mem[0]; s_point[1] = mem[1];
            candidates.pop();
            int check_pos = getPosition(w, s_point[0], s_point[1]);
            //cout<<"check_pos: "<<check_pos<<endl;
            if(map[check_pos]==FREE) break;
            if(candidates.empty()) break;
        }
        
        if(map[getPosition(w, s_point[0], s_point[1])]==VISI) break;
    }
    
    for(int i=0;i<middles.size(); i++){
        cout << "x: "<<middles[i][0] <<", y: "<< middles[i][1]<< endl;
    }
    
    return 0;   
}
