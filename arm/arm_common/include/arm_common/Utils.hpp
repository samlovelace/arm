#ifndef UTILS_H
#define UTILS_H

#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include "plog/Log.h" 
#include <Eigen/Geometry>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "robot_idl/msg/plan_command.hpp"
#include "robot_idl/msg/plan_response.hpp"

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

    inline void logPos(const KDL::Frame& frame, const std::string& header)
    {
        // Position
        double x = frame.p.x();
        double y = frame.p.y();
        double z = frame.p.z();

        LOGD << header << ": x=" << x << ", y=" << y << ", z=" << z;
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

    inline std::array<float, 6> toArray6(const std::vector<double>& v) 
    {
        if (v.size() < 6) {
            throw std::runtime_error("Vector has fewer than 6 elements");
        }

        std::array<float, 6> arr;
        for (size_t i = 0; i < 6; ++i) {
            arr[i] = static_cast<float>(v[i]);
        }
        return arr;
    }

    inline KDL::JntArray toJntArray(const std::vector<double>& v)
    {
        KDL::JntArray jntArr(v.size()); 
        for(int i = 0; i < v.size(); i++)
        {
            jntArr(i) = v[i];
        }

        return jntArr; 
    }

    inline KDL::Frame toFrame(const geometry_msgs::msg::Pose& aPose)
    {
        geometry_msgs::msg::Quaternion q = aPose.orientation;
        KDL::Rotation rot = KDL::Rotation::Quaternion(q.x, q.y, q.z, q.w);

        KDL::Vector position(aPose.position.x, aPose.position.y, aPose.position.z);
        KDL::Frame pose(rot, position);

        return pose; 
    }

    inline KDL::Twist toTwist(const geometry_msgs::msg::Twist& aTwist)
    {
        KDL::Vector linear(aTwist.linear.x, aTwist.linear.y, aTwist.linear.z);
        KDL::Vector angular(aTwist.linear.x, aTwist.angular.y, aTwist.angular.z);
        KDL::Twist goal(linear, angular); 

        return goal; 
    }

}

#endif
