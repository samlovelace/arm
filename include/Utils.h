#ifndef UTILS_H
#define UTILS_H

#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include "plog/Log.h" 

namespace utils
{
    inline void logFrame(const KDL::Frame& frame)
    {
        // Position
        double x = frame.p.x();
        double y = frame.p.y();
        double z = frame.p.z();

        // Orientation (Quaternion)
        double qx, qy, qz, qw;
        frame.M.GetQuaternion(qx, qy, qz, qw);

        LOGD << "Frame Position: x=" << x << ", y=" << y << ", z=" << z;
        LOGD << "Frame Orientation (quaternion): qw=" << qw
            << ", qx=" << qx << ", qy=" << qy << ", qz=" << qz;
    }

    inline void printJntArray(const KDL::JntArray& joints, const std::string& label = "JntArray")
    {
        std::ostringstream oss;
        oss << label << " [";
        for (unsigned int i = 0; i < joints.rows(); ++i) {
            oss << joints(i);
            if (i < joints.rows() - 1) {
                oss << ", ";
            }
        }
        oss << "]";
        LOGD << oss.str();
    }

}

#endif
