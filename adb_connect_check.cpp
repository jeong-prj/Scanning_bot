#include <cstdio>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <array>

using namespace std;

string exec(const char* cmd) {
    array<char, 128> buffer;
    string result;
    unique_ptr<FILE, decltype(&pclose)> pipe(popen(cmd, "r"), pclose);
    if (!pipe) {
        throw runtime_error("popen() failed!");
    }
    while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
        result += buffer.data();
    }
    return result;
}

int main(){
    if(exec("adb devices")=="List of devices attached\n\n"){
        //glab5g - 192.168.0.43
        //matterport - 10.77.80.203 
        system("adb connect 192.168.0.43");
    }
    
    cout << exec("adb devices");
    system("adb shell input tap 800 2300");
    
    return 0;
}
