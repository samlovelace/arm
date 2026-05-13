#ifndef IMANIPCOMMS_H
#define IMANIPCOMMS_H

// TODO: remove dependency on Eigen/KDL, should only use std types in this node
#include <eigen3/Eigen/Dense>
#include <kdl/jntarray.hpp>

class IManipComms
{
public:
    virtual ~IManipComms() = default;

    virtual bool init() = 0;
    virtual KDL::JntArray getJointPositions() = 0;
    virtual KDL::JntArray getJointVelocities() = 0;
    virtual void sendJointCommand(const KDL::JntArray &aCmd) = 0;

};

#endif
