#ifndef IRMENTRY_HPP
#define IRMENTRY_HPP

#include "kdl/frames.hpp"
#include "kdl/jntarray.hpp"

struct IrmEntry
{
    KDL::Frame T_ee_base;
    KDL::JntArray jntAngles; 
    double manipulability;  

};

struct IrmEntryBinary {
    double position[3];
    double orientation[9];
    double manipulability;
};


#endif 
