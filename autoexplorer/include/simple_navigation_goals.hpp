#ifndef INCLUDE_SIMPLE_NAVIGATION_GOALS_HPP_
#define INCLUDE_SIMPLE_NAVIGATION_GOALS_HPP_

#include "frontier_detector.hpp"
#include "frontier_detector_dms.hpp"

namespace autoexplorer
{

using namespace std;

class SimpleNavigationGoals
{
protected:
    ros::NodeHandle m_nh;
    ros::Subscriber m_gridmapsub;
    int m_mapAvailable;
    
    nav_msgs::OccupancyGrid m_gridmap;
    vector<signed char> m_data;
    int m_resolution;
    int m_rows;
    int m_cols;

public:
    SimpleNavigationGoals(const ros::NodeHandle &nh_);
    virtual ~SimpleNavigationGoals();
    
    void right();
    
    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    void GetMap();//const nav_msgs::OccupancyGrid::ConstPtr& msg);
    
    void SendGoal();
    
private:
    std::mutex s_mutex_gridmap;
};

}



#endif /* INCLUDE_FRONTIER_DETECTOR_DMS_HPP_ */
