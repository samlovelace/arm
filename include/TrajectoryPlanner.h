#ifndef TRAJECTORYPLANNER_H
#define TRAJECTORYPLANNER_H
 
#include <kdl/jntarray.hpp>
 
class TrajectoryPlanner 
{ 
public:
    TrajectoryPlanner();
    ~TrajectoryPlanner();

    KDL::JntArray getNextWaypoint(); 

private:
   
};
#endif //TRAJECTORYPLANNER_H    