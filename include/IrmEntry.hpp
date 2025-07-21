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

#endif 
