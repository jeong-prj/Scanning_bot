#include <iostream>

using namespace std;

#define CANT 0
#define FREE 1
#define VISI 2


int position(int wid, int a, int b){ 
    return a + (wid * b);
}


int main(){
    int map[] = {0,0,0,0,0,0,
                 0,1,1,1,1,0,
                 0,1,1,0,1,0,
                 0,0,0,0,0,0};
    const int x = 6;
    const int y = 4;
    
    int width = 0;
    
    //start x, y
    int s_x = 0;
    int s_y = 0;
    
    //end x, y
    int e_x = 0;
    int e_y = 0;
    
    for(int i=0; i<x; i++){
        for(int j=0; j<y; j++){
            int pose = position(x, i, j);
            if(map[pose]== FREE){
                map[pose] = VISI;
                s_x = i;
                s_y = j;
            }
            else
                continue;
        }
    }
    
    cout << width << endl;
    
    return 0;   
}
