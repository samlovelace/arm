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

    // Gripper joints share the same physical comms channel as the arm (one
    // serial bus / one combined Gazebo joint array) but are addressed
    // separately so they never flow through arm trajectory generation.
    // Default no-op so comms backends without gripper support don't break.
    virtual KDL::JntArray getGripperPositions()  { return KDL::JntArray(0); }
    virtual KDL::JntArray getGripperVelocities() { return KDL::JntArray(0); }
    virtual void sendGripperCommand(const KDL::JntArray &) {}

};

#endif
