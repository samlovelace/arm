#ifndef JOINTPOSITIONWAYPOINT_H
#define JOINTPOSITIONWAYPOINT_H
 
#include <kdl/jntarray.hpp> 

class JointPositionWaypoint 
{ 
public:
    JointPositionWaypoint();
    ~JointPositionWaypoint();

    JointPositionWaypoint(const KDL::JntArray& aJointGoal, const KDL::JntArray& anArrival); 

    inline void setJointPositionGoal(const KDL::JntArray& aGoal) {mGoal = aGoal; }
    inline void setArrivalTolerance(const KDL::JntArray& aTolerance) {mArrivalTolerance = aTolerance; }

    inline KDL::JntArray jointPositionGoal() {return mGoal; }
    inline KDL::JntArray arrivalTolerance() {return mArrivalTolerance; }

    std::string toString(); 

private:
    KDL::JntArray mGoal; 
    KDL::JntArray mArrivalTolerance; 
   
};
#endif //JOINTPOSITIONWAYPOINT_H