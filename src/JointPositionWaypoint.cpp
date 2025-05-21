
#include "JointPositionWaypoint.h"
#include "plog/Log.h"

JointPositionWaypoint::JointPositionWaypoint() : mGoal(KDL::JntArray(6)), mArrivalTolerance(KDL::JntArray(6))
{

}

JointPositionWaypoint::JointPositionWaypoint(const KDL::JntArray& aGoal, const KDL::JntArray& anArrival) : 
    mGoal(aGoal), mArrivalTolerance(anArrival)
{

}


JointPositionWaypoint::~JointPositionWaypoint()
{

}

std::string JointPositionWaypoint::toString()
{
    LOGD << "Commanding joint position waypoint: \n" 
    << "Joint 0: " << mGoal(0) << "\n" 
    << "Joint 1: " << mGoal(1) << "\n"
    << "Joint 2: " << mGoal(2) << "\n"
    << "Joint 3: " << mGoal(3) << "\n"
    << "Joint 4: " << mGoal(4) << "\n"
    << "Joint 5: " << mGoal(5) << "\n"
    << "\n\n";  
}

