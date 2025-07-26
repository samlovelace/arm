#ifndef UTILS_H
#define UTILS_H

#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include "plog/Log.h" 
#include <Eigen/Geometry>

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

    inline KDL::Frame affineToKDLFrame(const Eigen::Affine3f& affine) 
    {
        const Eigen::Matrix3f& rot = affine.rotation();
        const Eigen::Vector3f& trans = affine.translation();

        KDL::Rotation kdl_rot = KDL::Rotation(
            rot(0,0), rot(0,1), rot(0,2),
            rot(1,0), rot(1,1), rot(1,2),
            rot(2,0), rot(2,1), rot(2,2)
        );

        KDL::Vector kdl_trans(trans.x(), trans.y(), trans.z());

        return KDL::Frame(kdl_rot, kdl_trans);
    }


}

#endif
