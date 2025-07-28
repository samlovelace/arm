#ifndef UTILS_H
#define UTILS_H

#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include "plog/Log.h" 
#include <Eigen/Geometry>
#include "IArmTaskPlanner.hpp"
#include "arm_idl/msg/plan_command.hpp"
#include "arm_idl/msg/plan_response.hpp"

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

    inline arm_idl::msg::PlanResponse toIdl(std::vector<IArmTaskPlanner::Plan> aPlans)
    {   
        // TODO: assumes we can only get a single task plan command so vector of plans is only size 1 
        // TODO: fix Plan struct namespace visibility 
        IArmTaskPlanner::Plan plan = aPlans[0]; 

        uint8_t type = plan.mPlanType == "pick" ? arm_idl::msg::PlanCommand::PICK : arm_idl::msg::PlanCommand::PLACE; 
        
        geometry_msgs::msg::Point position;
        position.set__x(plan.mT_G_V.p.x());
        position.set__y(plan.mT_G_V.p.y());
        position.set__z(plan.mT_G_V.p.z()); 

        double x, y, z, w; 
        plan.mT_G_V.M.GetQuaternion(x, y, z, w); 

        geometry_msgs::msg::Quaternion quat; 
        quat.set__w(w); 
        quat.set__x(x); 
        quat.set__y(y); 
        quat.set__z(z); 

        geometry_msgs::msg::Pose T_G_V; 
        T_G_V.set__position(position); 
        T_G_V.set__orientation(quat); 

        arm_idl::msg::PlanResponse response; 
        response.set__operation_type(type); 
        response.set__robot_global_pose(T_G_V); 
        
        return response; 
    }


}

#endif
