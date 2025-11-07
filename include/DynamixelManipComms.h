#ifndef DYNAMIXELMANIPCOMMS_H
#define DYNAMIXELMANIPCOMMS_H
 
#include <memory> 
#include "IManipComms.h"
#include <dynamixel_sdk/dynamixel_sdk.h>
 
class DynamixelManipComms : public IManipComms
{ 
public:
    DynamixelManipComms();
    ~DynamixelManipComms() override;  

    bool init() override; 
    KDL::JntArray getJointPositions() override; 
    KDL::JntArray getJointVelocities() override; 
    void sendJointCommand(const KDL::JntArray &aCmd) override; 

private:

    dynamixel::PortHandler* mPortHandler; 
   
};
#endif //DYNAMIXELMANIPCOMMS_H