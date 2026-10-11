#ifndef BASESTATE_HPP
#define BASESTATE_HPP

#include <cmath>
#include <string>
#include <vector>
#include <stdexcept>

#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>

// ---------------------------------------------------------------------------
// BaseState — generic 6-DOF rigid-body state of a mobile base, plus the
// mapping between that state and the base's virtual joints in the URDF.
//
// The mapping is driven by MobileBaseConfig::jointAxes (one of x, y, z,
// roll, pitch, yaw per joint). Virtual joints are expected to chain
// translations first, then yaw -> pitch -> roll (any subset), so rotational
// joint angles equal ZYX Euler angles (KDL::Rotation::RPY) and their rates
// equal the Euler rates.
// ---------------------------------------------------------------------------
struct BaseState
{
    KDL::Vector p;             // position in world
    double roll = 0.0;         // ZYX Euler angles (KDL::Rotation::RPY convention)
    double pitch = 0.0;
    double yaw = 0.0;
    KDL::Twist twist;          // world frame: linear velocity + angular velocity vector

    KDL::Frame frame() const { return KDL::Frame(KDL::Rotation::RPY(roll, pitch, yaw), p); }

    void setFrame(const KDL::Frame& aFrame)
    {
        p = aFrame.p;
        aFrame.M.GetRPY(roll, pitch, yaw);
    }

    // ZYX Euler rates [roll_dot, pitch_dot, yaw_dot] from the world-frame
    // angular velocity. Singular at pitch = +-90 deg, where cos(pitch) is
    // clamped away from zero.
    KDL::Vector eulerRates() const
    {
        const KDL::Vector& w = twist.rot;
        const double cy = std::cos(yaw), sy = std::sin(yaw);
        const double sp = std::sin(pitch);
        double cp = std::cos(pitch);
        constexpr double kMinCos = 1e-3;
        if(std::fabs(cp) < kMinCos)
        {
            cp = std::copysign(kMinCos, cp);
        }

        const double rollDot  = (w.x() * cy + w.y() * sy) / cp;
        const double pitchDot = -w.x() * sy + w.y() * cy;
        const double yawDot   = w.z() + rollDot * sp;
        return KDL::Vector(rollDot, pitchDot, yawDot);
    }

    // Value of one rigid-body DOF ("x", "y", "z", "roll", "pitch", "yaw")
    double position(const std::string& anAxis) const
    {
        if(anAxis == "x")     return p.x();
        if(anAxis == "y")     return p.y();
        if(anAxis == "z")     return p.z();
        if(anAxis == "roll")  return roll;
        if(anAxis == "pitch") return pitch;
        if(anAxis == "yaw")   return yaw;
        throw std::invalid_argument("BaseState: unknown axis " + anAxis);
    }

    void setPosition(const std::string& anAxis, double aValue)
    {
        if(anAxis == "x")          p.x(aValue);
        else if(anAxis == "y")     p.y(aValue);
        else if(anAxis == "z")     p.z(aValue);
        else if(anAxis == "roll")  roll = aValue;
        else if(anAxis == "pitch") pitch = aValue;
        else if(anAxis == "yaw")   yaw = aValue;
        else throw std::invalid_argument("BaseState: unknown axis " + anAxis);
    }

    // Joint-space positions for the given axis mapping
    KDL::JntArray jointPositions(const std::vector<std::string>& aJointAxes) const
    {
        KDL::JntArray q(aJointAxes.size());
        for(size_t i = 0; i < aJointAxes.size(); i++)
        {
            q(i) = position(aJointAxes[i]);
        }
        return q;
    }

    // Joint-space velocities for the given axis mapping — translations come
    // straight from the world-frame linear velocity, rotations are Euler rates
    KDL::JntArray jointVelocities(const std::vector<std::string>& aJointAxes) const
    {
        const KDL::Vector rates = eulerRates();
        KDL::JntArray qdot(aJointAxes.size());
        for(size_t i = 0; i < aJointAxes.size(); i++)
        {
            const std::string& axis = aJointAxes[i];
            if(axis == "x")          qdot(i) = twist.vel.x();
            else if(axis == "y")     qdot(i) = twist.vel.y();
            else if(axis == "z")     qdot(i) = twist.vel.z();
            else if(axis == "roll")  qdot(i) = rates.x();
            else if(axis == "pitch") qdot(i) = rates.y();
            else if(axis == "yaw")   qdot(i) = rates.z();
            else throw std::invalid_argument("BaseState: unknown axis " + axis);
        }
        return qdot;
    }

    // Overwrite the mapped DOFs with joint positions; unmapped DOFs keep their value
    void setJointPositions(const std::vector<std::string>& aJointAxes, const KDL::JntArray& aQ)
    {
        for(size_t i = 0; i < aJointAxes.size() && i < aQ.rows(); i++)
        {
            setPosition(aJointAxes[i], aQ(i));
        }
    }
};

#endif // BASESTATE_HPP
