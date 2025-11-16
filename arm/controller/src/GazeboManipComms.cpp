
#include "GazeboManipComms.h"
#include "common/RosTopicManager.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "plog/Log.h"

GazeboManipComms::GazeboManipComms(const std::string& anArmVersion) : 
        mJointVelocities(6), mPrevJointPos(6), 
        mPrevTime(std::chrono::steady_clock::now()), mFirstRcvd(false)
{
    mArmVersion = versionFromString(anArmVersion); 

    int numJoints = 6; 

    switch (mArmVersion)
    {
    case VERSION::UR5:
        numJoints = 8; // TODO: update when gripper control implemented? 
        break;
    default:
        break;
    }

    mPrevJointPos.resize(numJoints); 
    mJointVelocities.resize(numJoints); 

    // Init previous positions and velocities 
    for(int i = 0; i < numJoints; i++)
    {
        mPrevJointPos(i) = 0.0; 
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

    return true; 
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
        jntArray(i) = msg->data[i]; 
        jntVel(i) = ((jntArray(i) - mPrevJointPos(i)) / (now.time_since_epoch().count() - mPrevTime.time_since_epoch().count())); 
    }

    setJointPositions(jntArray); 
    setJointVelocities(jntVel); 
    mPrevJointPos = jntArray; 
    mPrevTime = now; 
}

GazeboManipComms::VERSION GazeboManipComms::versionFromString(const std::string& anArmVersion)
{
    VERSION version = VERSION::UR10; 

    if("ur10" == anArmVersion)
    {
        version = VERSION::UR10; 
    }
    else if ("ur5" == anArmVersion)
    {
        version = VERSION::UR5; 
    }

    return version; 
}