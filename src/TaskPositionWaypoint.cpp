
#include "TaskPositionWaypoint.h"

TaskPositionWaypoint::TaskPositionWaypoint(const KDL::Frame& aGoalPose, 
                                           const std::vector<double>& anArrivalTolerances, 
                                           const std::string& aCommandFrame) : mGoalPose(aGoalPose), 
                                                                               mArrivalTolerances(anArrivalTolerances), 
                                                                               mCommandFrame(aCommandFrame)
{

}

TaskPositionWaypoint::~TaskPositionWaypoint()
{

}


