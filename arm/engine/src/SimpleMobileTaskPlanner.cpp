
#include "SimpleMobileTaskPlanner.h"
#include "plog/Log.h"

SimpleMobileTaskPlanner::SimpleMobileTaskPlanner()
{

}

bool SimpleMobileTaskPlanner::init()
{
    LOGV << "Simple init!"; 
    return true; 
}

bool SimpleMobileTaskPlanner::planPick(const Eigen::Affine3f& aT_G_ee, Eigen::Affine3f& aT_G_R)
{
    LOGV << "Planning pick task with simple mobile planner"; 
    return true; 
}
