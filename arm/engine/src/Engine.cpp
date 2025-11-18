
#include "Engine.h"

#include "common/RosTopicManager.hpp"
#include "common/RateController.hpp"
#include "common/ConfigManager.h"

#include "PointCloudHandler.h"

#include "plog/Log.h"

#include "SequenceNode.h"
#include "PlanGraspNode.h"

#include "PlannerFactory.h"
#include <yaml-cpp/yaml.h>

Engine::Engine() : mActiveTree(nullptr), mGraspPlanner(nullptr)
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
    // grasp planner config 
    auto config = ConfigManager::getInstance()->getValue<YAML::Node>("Engine.GraspPlanning").as<YAML::Node>(); 
    mGraspPlanner = PlannerFactory::createGraspPlanner(config);

    if(nullptr == mGraspPlanner)
    {
        LOGE << "Failed to create GraspPlanner from config"; 
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
            LOGI << "Recieved Pick Command";

            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
            PointCloudHandler::toPCL<pcl::PointXYZ>(aCmd->pick_obj_point_cloud_gl, *cloud); 

            // TODO: probably need to respond somehow 
            if(cloud->empty())
            {
                LOGE << "Cannot plan on empty object cloud"; 
                return; 
            }

            auto ctx = std::make_shared<PickContext>(cloud);  
            auto planGraspNode = std::make_shared<PlanGraspNode>(ctx, mGraspPlanner); 

            std::vector<NodePtr> nodes = {planGraspNode}; 
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