
#include "GazeboManipComms.h"
#include "RosTopicManager.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "plog/Log.h"

GazeboManipComms::GazeboManipComms() : mJointVelocities(6), mPrevTime(std::chrono::steady_clock::now())
{
    for(int i = 0; i < 6; i++)
    {
        mJointVelocities(i) = 0.0; 
    }

}

GazeboManipComms::~GazeboManipComms()
{

}

bool GazeboManipComms::init()
{
    RosTopicManager::getInstance()->createSubscriber<std_msgs::msg::Float64MultiArray>("UR/joint_positions",
                                                                                       std::bind(&GazeboManipComms::jointPositionCallback,
                                                                                                 this, 
                                                                                                 std::placeholders::_1)); 

    RosTopicManager::getInstance()->createPublisher<std_msgs::msg::Float64MultiArray>("UR/joint_commands"); 
}

KDL::JntArray GazeboManipComms::getJointPositions()
{
    std::lock_guard<std::mutex> lock(mJointPosMutex); 
    return mJointPositions; 
}

void GazeboManipComms::setJointPositions(const KDL::JntArray& aJointPos)
{
    std::lock_guard<std::mutex> lock(mJointPosMutex); 
    mJointPositions = aJointPos; 
}

KDL::JntArray GazeboManipComms::getJointVelocities()
{
    std::lock_guard<std::mutex> lock(mJointVelMutex); 
    return mJointVelocities; 
}

void GazeboManipComms::setJointVelocities(const KDL::JntArray& aJointVel)
{
    std::lock_guard<std::mutex> lock(mJointVelMutex); 
    mJointVelocities = aJointVel; 
}

void GazeboManipComms::sendJointCommand(const KDL::JntArray &aCmd)
{
    // convert KDL::JntArray to ros2 topic msg type 
    std_msgs::msg::Float64MultiArray cmd; 
    
    std::vector<double> posCmd(aCmd.rows()); 

    for(int i = 0; i < aCmd.rows(); i++)
    {
        posCmd[i] = aCmd(i); 
    }

    LOGW << "sending jnt pos cmd"; 
    cmd.set__data(posCmd); 
    RosTopicManager::getInstance()->publishMessage<std_msgs::msg::Float64MultiArray>("UR/joint_commands", cmd); 
}

void GazeboManipComms::jointPositionCallback(std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    // get the ros2 topic data and convert to internal representation
    KDL::JntArray jntArray(msg->data.size());
    KDL::JntArray jntVel(msg->data.size()); 
    auto now = std::chrono::steady_clock::now(); 

    for(int i = 0; i < msg->data.size(); i++)
    {
        //LOGW << "Gazebo Joint POS " << i << ": " << msg->data[i]; 
        jntArray(i) = msg->data[i]; 
        jntVel(i) = (jntArray(i) - mPrevJointPos(i)) / (now.time_since_epoch().count() - mPrevTime.time_since_epoch().count()); 
    }

    setJointPositions(jntArray); 
    setJointVelocities(jntVel); 
    mPrevJointPos = jntArray; 
    mPrevTime = now; 
}