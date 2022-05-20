#include <iostream>
#include <algorithm>

using namespace std;

#define CANT 0
#define FREE 1
#define VISI 2
#define SIZE 9
//                 up r dow  l 
int x_direc[4] = { 0, 1, 0, -1};
int y_direc[4] = {-1, 0, 1,  0};

void mapDisplay(int map[], int w, int h){
    for(int i=0; i<(w*h); i++){
        if((i%w) == 0) cout << endl;
        cout << map[i] << " " ;
    }
    cout << endl;
}

int getPosition(int wid, int a, int b){
    return a + (wid * b);
}

int findCells(int map[], int wid, int x, int y, int area, int direc){
    int pose = getPosition(wid, x, y);
    cout << pose << " " << x << " "<< y << endl;
    mapDisplay(map, wid, 4);
    if(area == SIZE){
        return 0;
    }
    if(map[pose]==FREE){
        map[pose] = VISI;
        area++;
        findCells(map, wid, x + x_direc[direc], y + y_direc[direc], area, (direc+1)%4);
        
    }
    
    return 0;
}


int main(){
    int map[] = {0,0,0,0,0,0,
                 0,1,1,1,1,0,
                 0,1,1,1,1,0,
                 0,1,1,1,0,0};
    const int w = 6;
    const int h = 4;
    
    //start x, y
    int s_x = 0;
    int s_y = 0;

    //end x, y
    int e_x = 0;
    int e_y = 0;
    
    int flag = 0;
    for(int i=0; i<h; i++){//y
        for(int j=0; j<w; j++){//x
            int pose = getPosition(w, j, i);
            if(map[pose]== FREE){
                s_x = j; s_y = i;
                flag = 1;
                break;
            }
            else
                continue;
        }
        if(flag == 1) break;
    }
    
    findCells(map, w, s_x, s_y, 0, 1);
    mapDisplay(map, w, h);
    return 0;   
}
