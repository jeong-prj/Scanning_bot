#ifndef INCLUDE_GETMAP_HPP_
#define INCLUDE_GETMAP_HPP_

#include <ros/ros.h>
#include <ros/console.h>

#include "nav_msgs/OccupancyGrid.h"
#include "map_msgs/OccupancyGridUpdate.h"
#include "nav_msgs/MapMetaData.h"
#include <visualization_msgs/Marker.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/action_client.h>

#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <queue>
#include <array>

#include <boost/bind.hpp>

#include "tsp.h"
/*
protected:
    //ros::NodeHandle m_nh;
    //ros::Subscriber m_gridmapsub;
    int m_mapAvailable;
    
    nav_msgs::OccupancyGrid m_gridmap;
    vector<signed char> m_data;
    int m_resolution;
    int m_rows;
    int m_cols;
*/    


#endif /* INCLUDE_GETMAP_HPP_ */
