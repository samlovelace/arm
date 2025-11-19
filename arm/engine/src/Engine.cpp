
#include "Engine.h"

#include "common/RosTopicManager.hpp"
#include "common/RateController.hpp"
#include "common/ConfigManager.h"

#include "PointCloudHandler.h"

#include "plog/Log.h"

#include "SequenceNode.h"
#include "PlanGraspNode.h"
#include "PlanPickTaskNode.h"

#include "PlannerFactory.h"
#include <yaml-cpp/yaml.h>

Engine::Engine() : mActiveTree(nullptr), mGraspPlanner(nullptr), mKinematicsHandler(std::make_shared<KinematicsHandler>())
{
    RosTopicManager::getInstance()->createSubscriber<robot_idl::msg::ManipulationCommand>("/arm/command", 
                                                                                          std::bind(&Engine::commandCallback, 
                                                                                                    this, 
                                                                                                    std::placeholders::_1)); 
    RosTopicManager::getInstance()->spinNode(); 
}

Engine::~Engine()
{

}

bool Engine::init()
{
    if(!mKinematicsHandler->init(ConfigManager::getInstance()->getConfig().urdfPath))
    {
        throw std::runtime_error("Failed to initialize kinematics"); 
    }

    // grasp planner config 
    auto config = ConfigManager::getInstance()->getValue<YAML::Node>("Engine.GraspPlanning").as<YAML::Node>(); 
    mGraspPlanner = PlannerFactory::createGraspPlanner(config);
    auto taskPlannerConfig = ConfigManager::getInstance()->getValue<YAML::Node>("Engine.Planning").as<YAML::Node>(); 
    mTaskPlanner = PlannerFactory::createArmTaskPlanner(taskPlannerConfig["type"].as<std::string>(), mKinematicsHandler); 

    if(nullptr == mGraspPlanner)
    {
        LOGE << "Failed to create GraspPlanner from config"; 
        return false; 
    }

    if(nullptr == mTaskPlanner)
    {
        LOGE << "Failed to create ArmTaskPlanner from config"; 
        return false; 
    }

    while(!rclcpp::ok())
    {
        std::this_thread::sleep_for(std::chrono::seconds(1)); 
    }

    return true; 
}

void Engine::run()
{
    if(!init())
    {
        LOGE << "Failed to initialize Arm Engine"; 
        return; 
    }

    LOGI << "Engine Initialized Successfully!"; 

    int rate_hz = ConfigManager::getInstance()->getValue<int>("Engine.rate");
    RateController rate(rate_hz); 

    mStatus = INode::Status::RUNNING; 

    // main program loop, should never exit 
    while(true)
    {
        while(INode::Status::RUNNING == mStatus)
        {
            rate.start(); 
            
            // tick tree if set
            if(mActiveTree)
                mStatus = mActiveTree->tick(); 
            
            rate.block(); 
        }
    }

}

void Engine::commandCallback(robot_idl::msg::ManipulationCommand::SharedPtr aCmd)
{
    using namespace robot_idl::msg; 

    switch (aCmd->cmd)
    {
        case ManipulationCommand::CMD_PICK:
        {
            LOGI << "Received Pick Command";

            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
            PointCloudHandler::toPCL<pcl::PointXYZ>(aCmd->pick_obj_point_cloud_gl, *cloud); 

            // TODO: probably need to respond somehow 
            if(cloud->empty())
            {
                LOGE << "Cannot plan on empty object cloud"; 
                return; 
            }

            // TODO: get object global pose from msg and populate PickContext with it 
            auto ctx = std::make_shared<PickContext>(cloud); 
            ctx->mT_G_O.p = KDL::Vector(1, 1, 1);  
            
            // instantiate nodes 
            auto planGraspNode = std::make_shared<PlanGraspNode>(ctx, mGraspPlanner); 
            auto planPickTaskNode = std::make_shared<PlanPickTaskNode>(ctx, mTaskPlanner); 

            // vector of nodes for SequenceNode 
            std::vector<NodePtr> nodes = {planGraspNode, planPickTaskNode}; 
            auto tree = std::make_shared<SequenceNode>(nodes); 

            mActiveTree = tree; 
            break;
        }
        case ManipulationCommand::CMD_PLACE: 
            LOGD << "GOT PLACE CMD"; 
            break; 
        case ManipulationCommand::CMD_PLACE_REL:
            LOGD << "GOT PLACE REL CMD"; 
            break; 
        default:
            mActiveTree = nullptr; 
            break;
    }
}