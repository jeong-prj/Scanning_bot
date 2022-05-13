#ifndef INCLUDE_SIMPLE_NAVIGATION_GOALS_HPP_
#define INCLUDE_SIMPLE_NAVIGATION_GOALS_HPP_

#include "frontier_detector.hpp"

namespace autoexplorer
{

using namespace std;

class SimpleNavigationGoals
{
public:
    SimpleNavigationGoals();
    virtual ~SimpleNavigationGoals();
    
    void right();
    void SendGoal();
};

}



#endif /* INCLUDE_FRONTIER_DETECTOR_DMS_HPP_ */
