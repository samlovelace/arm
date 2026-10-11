// Minimal 6-DOF mobile base simulator: integrates a world-frame twist and
// publishes the base state on robot/state, standing in for the real vehicle
// (ptera) so base-aware arm control can be tested in sim.
//
// Inputs:
//   robot/sim/cmd_vel            geometry_msgs/Twist, world frame, held until it times out
//   <MobileBase command_topic>   ptera_msgs/VehicleWaypoint, reached over execution_duration
//
// Outputs:
//   robot/state                  ptera_msgs/RobotState (velocities in world frame)
//   joint_states                 base virtual joints, for robot_state_publisher

#include <chrono>
#include <mutex>

#include <rclcpp/utilities.hpp>
#include <kdl/frames.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "ptera_msgs/msg/robot_state.hpp"
#include "ptera_msgs/msg/vehicle_waypoint.hpp"

#include "arm_common/BaseState.hpp"
#include "arm_common/ConfigManager.h"
#include "arm_common/RateController.hpp"
#include "arm_common/RosTopicManager.hpp"

static constexpr const char* ROBOT_STATE_TOPIC = "robot/state";
static constexpr const char* CMD_VEL_TOPIC = "robot/sim/cmd_vel";
static constexpr const char* JOINT_STATES_TOPIC = "joint_states";

class BaseSim
{
public:
    explicit BaseSim(const ConfigManager::MobileBaseConfig& aConfig)
        : mConfig(aConfig)
    {
        mState.setJointPositions(mConfig.jointAxes, toJntArray(mConfig.initialPositions));
    }

    void init()
    {
        auto* ros = RosTopicManager::getInstance();

        ros->createPublisher<ptera_msgs::msg::RobotState>(ROBOT_STATE_TOPIC);
        ros->createPublisher<sensor_msgs::msg::JointState>(JOINT_STATES_TOPIC);

        ros->createSubscriber<geometry_msgs::msg::Twist>(CMD_VEL_TOPIC,
            std::bind(&BaseSim::cmdVelCallback, this, std::placeholders::_1));

        ros->createSubscriber<ptera_msgs::msg::VehicleWaypoint>(mConfig.commandTopic,
            std::bind(&BaseSim::waypointCallback, this, std::placeholders::_1));

        RCLCPP_INFO(ros->get_logger(), "Base sim publishing %s, listening on %s and %s",
                    ROBOT_STATE_TOPIC, CMD_VEL_TOPIC, mConfig.commandTopic.c_str());
    }

    void run(int aRateHz)
    {
        RateController rate(aRateHz);
        const double dt = 1.0 / aRateHz;

        while(rclcpp::ok())
        {
            rate.start();
            step(dt);
            publish();
            rate.block();
        }
    }

private:
    enum class Mode { Idle, Velocity, Waypoint };

    void step(double aDt)
    {
        std::lock_guard<std::mutex> lock(mMutex);
        const auto now = std::chrono::steady_clock::now();

        // cmd_vel is held only while it keeps arriving; a waypoint runs for its duration
        if((mMode == Mode::Velocity && now - mModeStart > kCmdVelTimeout) ||
           (mMode == Mode::Waypoint && now >= mModeEnd))
        {
            mMode = Mode::Idle;
        }

        mState.twist = (mMode == Mode::Idle) ? KDL::Twist::Zero() : mCommandTwist;
        mState.setFrame(KDL::addDelta(mState.frame(), mState.twist, aDt));
    }

    void publish()
    {
        BaseState state;
        {
            std::lock_guard<std::mutex> lock(mMutex);
            state = mState;
        }

        auto* ros = RosTopicManager::getInstance();

        ptera_msgs::msg::RobotState msg;
        msg.timestamp = ros->now();
        msg.position.x = state.p.x();
        msg.position.y = state.p.y();
        msg.position.z = state.p.z();
        msg.velocity.x = state.twist.vel.x();
        msg.velocity.y = state.twist.vel.y();
        msg.velocity.z = state.twist.vel.z();
        msg.euler.roll  = state.roll;
        msg.euler.pitch = state.pitch;
        msg.euler.yaw   = state.yaw;
        state.frame().M.GetQuaternion(msg.quat.x, msg.quat.y, msg.quat.z, msg.quat.w);
        msg.angular_velocity.x = state.twist.rot.x();
        msg.angular_velocity.y = state.twist.rot.y();
        msg.angular_velocity.z = state.twist.rot.z();
        ros->publishMessage(ROBOT_STATE_TOPIC, msg);

        const KDL::JntArray q = state.jointPositions(mConfig.jointAxes);
        const KDL::JntArray qdot = state.jointVelocities(mConfig.jointAxes);

        sensor_msgs::msg::JointState js;
        js.header.stamp = msg.timestamp;
        js.name = mConfig.jointNames;
        for(unsigned int i = 0; i < q.rows(); i++)
        {
            js.position.push_back(q(i));
            js.velocity.push_back(qdot(i));
            js.effort.push_back(0.0);
        }
        ros->publishMessage(JOINT_STATES_TOPIC, js);
    }

    void cmdVelCallback(geometry_msgs::msg::Twist::SharedPtr aMsg)
    {
        std::lock_guard<std::mutex> lock(mMutex);
        mCommandTwist = KDL::Twist(KDL::Vector(aMsg->linear.x, aMsg->linear.y, aMsg->linear.z),
                                   KDL::Vector(aMsg->angular.x, aMsg->angular.y, aMsg->angular.z));
        mMode = Mode::Velocity;
        mModeStart = std::chrono::steady_clock::now();
    }

    void waypointCallback(ptera_msgs::msg::VehicleWaypoint::SharedPtr aMsg)
    {
        const KDL::Frame target(KDL::Rotation::RPY(aMsg->euler.roll, aMsg->euler.pitch, aMsg->euler.yaw),
                                KDL::Vector(aMsg->position.x, aMsg->position.y, aMsg->position.z));

        std::lock_guard<std::mutex> lock(mMutex);
        const auto now = std::chrono::steady_clock::now();

        if(aMsg->execution_duration <= 0.0)
        {
            mState.setFrame(target);
            mMode = Mode::Idle;
            return;
        }

        // constant world-frame twist that reaches the target over the duration
        mCommandTwist = KDL::diff(mState.frame(), target, aMsg->execution_duration);
        mMode = Mode::Waypoint;
        mModeStart = now;
        mModeEnd = now + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                             std::chrono::duration<double>(aMsg->execution_duration));
    }

    static KDL::JntArray toJntArray(const std::vector<double>& aVec)
    {
        KDL::JntArray q(aVec.size());
        for(size_t i = 0; i < aVec.size(); i++)
        {
            q(i) = aVec[i];
        }
        return q;
    }

    static constexpr std::chrono::milliseconds kCmdVelTimeout{500};

    ConfigManager::MobileBaseConfig mConfig;

    std::mutex mMutex;
    BaseState mState;
    KDL::Twist mCommandTwist;
    Mode mMode = Mode::Idle;
    std::chrono::steady_clock::time_point mModeStart;
    std::chrono::steady_clock::time_point mModeEnd;
};

int main()
{
    rclcpp::init(0, nullptr);
    RosTopicManager::getInstance("base_sim")->spinNode();

    ConfigManager::getInstance()->loadConfig();
    const auto& config = ConfigManager::getInstance()->getConfig();

    if(!config.mobileBase)
    {
        std::cout << "No MobileBase configured, base sim has nothing to do" << std::endl;
        rclcpp::shutdown();
        return 0;
    }

    BaseSim sim(*config.mobileBase);
    sim.init();
    sim.run(50);

    rclcpp::shutdown();
    return 0;
}
