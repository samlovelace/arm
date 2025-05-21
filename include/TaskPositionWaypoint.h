#ifndef TASKPOSITIONWAYPOINT_H
#define TASKPOSITIONWAYPOINT_H
 
#include <string> 
#include <vector> 
#include "kdl/frames.hpp"
 
class TaskPositionWaypoint 
{ 
public:
    TaskPositionWaypoint(const KDL::Frame& aGoalPose, const std::vector<double>& anArrivalTolerance, const std::string& aCommandFrame);
    ~TaskPositionWaypoint();

    KDL::Frame getGoalPose() {return mGoalPose;}
    std::string getCommandFrame() {return mCommandFrame; }
    std::vector<double> getArrivalTolerances() {return mArrivalTolerances; }

private:
    // TODO: make this an enum 
    std::string mCommandFrame; 
    KDL::Frame mGoalPose;
    
    // TODO: could probs use a std::array here for fixed size 
    std::vector<double> mArrivalTolerances; 

};
#endif //TASKPOSITIONWAYPOINT_H