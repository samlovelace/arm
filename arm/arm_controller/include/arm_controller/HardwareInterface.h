#ifndef HARDWAREINTERFACE_H
#define HARDWAREINTERFACE_H

#include <kdl/jntarray.hpp>

class HardwareInterface
{
public:
    HardwareInterface();
    ~HardwareInterface();

    bool init();
    KDL::JntArray getJointPositions();
    KDL::JntArray getJointVelocities();
    void sendJointCommand(const KDL::JntArray &aCmd);

private:

};
#endif
