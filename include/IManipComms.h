#ifndef IMANIPCOMMS_H
#define IMANIPCOMMS_H

#include <eigen3/Eigen/Dense>
#include <kdl/jntarray.hpp>

class IManipComms
{
public:
    IManipComms(/* args */);
    virtual ~IManipComms() = default; 

    virtual bool init() = 0; 
    virtual Eigen::VectorXd getJointPositions() = 0; 
    virtual void sendJointCommand(const KDL::JntArray &aCmd) = 0; 

};

#endif