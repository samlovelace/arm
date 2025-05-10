
#include "GazeboManipComms.h"
#include "RosTopicManager.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "plog/Log.h"

GazeboManipComms::GazeboManipComms()
{

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
    
    for(int i = 0; i < msg->data.size(); i++)
    {
        //LOGW << "Gazebo Joint POS " << i << ": " << msg->data[i]; 
        jntArray(i) = msg->data[i]; 
    }

    setJointPositions(jntArray); 
}