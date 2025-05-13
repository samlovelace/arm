#ifndef GAZEBOMANIPCOMMS_H
#define GAZEBOMANIPCOMMS_H
 
#include "IManipComms.h"
#include "std_msgs/msg/float64_multi_array.hpp"
#include <mutex> 

class GazeboManipComms : public IManipComms
{ 
public:
    GazeboManipComms();
    ~GazeboManipComms() override; 

    bool init() override; 
    KDL::JntArray getJointPositions() override; 
    KDL::JntArray getJointVelocities() override; 
    void sendJointCommand(const KDL::JntArray &aCmd) override; 

private:

    void setJointPositions(const KDL::JntArray& aJointPos);
    void setJointVelocities(const KDL::JntArray& aJointVel); 
    void jointPositionCallback(std_msgs::msg::Float64MultiArray::SharedPtr msg);

    std::mutex mJointPosMutex; 
    KDL::JntArray mJointPositions; 

    std::mutex mJointVelMutex; 
    KDL::JntArray mJointVelocities; 

    KDL::JntArray mPrevJointPos; 
    std::chrono::time_point<std::chrono::steady_clock> mPrevTime; 


   
};
#endif //GAZEBOMANIPCOMMS_H