
#include "arm_common/RosTopicManager.hpp"
#include "arm_controller/HardwareInterface.h"

HardwareInterface::HardwareInterface()
{

}

HardwareInterface::~HardwareInterface()
{

}

bool HardwareInterface::init()
{
    // Create publishers and subscribers to interface with arm_hardware
    RosTopicManager::getInstance()
}

KDL::JntArray HardwareInterface::getJointPositions()
{

}

KDL::JntArray HardwareInterface::getJointVelocities()
{

}

void HardwareInterface::sendJointCommand(const KDL::JntArray &aCmd)
{

}
