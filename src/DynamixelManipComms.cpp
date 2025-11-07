
#include "DynamixelManipComms.h"

DynamixelManipComms::DynamixelManipComms() : mPortHandler(nullptr)
{

}

DynamixelManipComms::~DynamixelManipComms()
{

}

bool DynamixelManipComms::init()
{
    mPortHandler = dynamixel::PortHandler::getPortHandler("/dev/ttyUSB0"); 
 
    if(nullptr == mPortHandler) throw std::runtime_error("Failed to instantiate PortHandler for device"); 

    if(!mPortHandler->openPort()) throw std::runtime_error("Failed to open port to dynamixel manipulator");  
    if(!mPortHandler->setBaudRate(57600)) throw std::runtime_error("Failed to set Baud Rate"); 

    return true; 
}

KDL::JntArray DynamixelManipComms::getJointPositions()
{

} 

KDL::JntArray DynamixelManipComms::getJointVelocities()
{

} 
void DynamixelManipComms::sendJointCommand(const KDL::JntArray &aCmd)
{

} 