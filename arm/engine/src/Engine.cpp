
#include "Engine.h"

#include <yaml-cpp/yaml.h>
#include "common/RosTopicManager.hpp"
#include "common/RateController.hpp"
#include "common/ConfigManager.h"
#include "plog/Log.h"
#include "PointCloudHandler.h"

#include "SequenceNode.h"
#include "PlanGraspNode.h"
#include "PlanPickTaskNode.h"
#include "ExecutePathNode.h"
#include "SendWaypointNode.h"
#include "MovingNode.h"
#include "MoveVehicleNode.h"

#include "PlannerFactory.h"

Engine::Engine() : mActiveTree(nullptr), mGraspPlanner(nullptr), mKinematicsHandler(std::make_shared<KinematicsHandler>())
{
    RosTopicManager::getInstance()->createSubscriber<robot_idl::msg::ManipulationCommand>("/arm/command", 
                                                                                          std::bind(&Engine::commandCallback, 
                                                                                                    this, 
                                                                                                    std::placeholders::_1)); 
    RosTopicManager::getInstance()->spinNode(); 
    mStatus = INode::Status::RUNNING; 
}

Engine::~Engine()
{

}

bool Engine::init()
{
    std::string manipUrdfPath = ConfigManager::getInstance()->getConfig().urdfPath; 
    std::string robotUrdfpath = ConfigManager::getInstance()->getConfig().robotUrdfPath; 
    if(!mKinematicsHandler->init(manipUrdfPath, robotUrdfpath))
    {
        throw std::runtime_error("Failed to initialize kinematics"); 
    }

    mTrajectoryPlanner = std::make_shared<TrajectoryPlanner>(mKinematicsHandler);

    // grasp planner config 
    auto config = ConfigManager::getInstance()->getValue<YAML::Node>("Engine.GraspPlanning").as<YAML::Node>(); 
    mGraspPlanner = PlannerFactory::createGraspPlanner(config);
    auto taskPlannerConfig = ConfigManager::getInstance()->getValue<YAML::Node>("Engine.Planning").as<YAML::Node>(); 
    mTaskPlanner = PlannerFactory::createArmTaskPlanner(taskPlannerConfig["type"].as<std::string>(), 
                                                        mKinematicsHandler, 
                                                        mTrajectoryPlanner); 

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

    // main program loop, should never exit 
    while (true) 
    {
        rate.start();

        INode::Status status;
        {
            std::lock_guard<std::mutex> lock(mTreeMutex);
            status = mStatus;

            if (mActiveTree && mStatus == INode::Status::RUNNING)
            {
                mStatus = mActiveTree->tick();
            }
        }

        rate.block();
    }

}

INode::Status Engine::tickActiveTree()
{
    std::lock_guard<std::mutex> lock(mTreeMutex); 
    return mActiveTree->tick(); 
}

void Engine::commandCallback(robot_idl::msg::ManipulationCommand::SharedPtr aCmd)
{
    using namespace robot_idl::msg; 
    NodePtr tree; 
    auto cmd = *aCmd; 

    switch (cmd.cmd)
    {
        case ManipulationCommand::CMD_PICK:
        {
            LOGI << "Received Pick Command";

            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
            PointCloudHandler::toPCL<pcl::PointXYZ>(cmd.pick_obj_point_cloud_gl, *cloud); 

            // TODO: probably need to respond somehow 
            if(cloud->empty())
            {
                LOGE << "Cannot plan on empty object cloud"; 
                return; 
            }

            // TODO: get object global pose from msg and populate PickContext with it 
            mPickContext.reset(); 
            mPickContext = std::make_shared<PickContext>(cloud); 
            mPickContext->mT_G_O.p = KDL::Vector(1, 1, 1);  

            // instantiate nodes 
            auto planGraspNode = std::make_shared<PlanGraspNode>(mPickContext, mGraspPlanner); 
            auto planPickTaskNode = std::make_shared<PlanPickTaskNode>(mPickContext, mTaskPlanner); 
            auto moveVehNode = std::make_shared<MoveVehicleNode>(mPickContext);

            // vector of nodes for SequenceNode 
            std::vector<NodePtr> planNodes = {planGraspNode, planPickTaskNode, moveVehNode};
            auto planTree = std::make_shared<SequenceNode>(planNodes); 

            // execute tree 
            auto wpPtr = std::make_shared<KDL::JntArray>(); 
             
            auto sendWpNode = std::make_shared<SendWaypointNode>(wpPtr); 
            auto movingNode = std::make_shared<MovingNode>(); 
            std::vector<NodePtr> execNodes = {sendWpNode, movingNode};

            auto seqNode = std::make_shared<SequenceNode>(execNodes);
            auto execTree = std::make_shared<ExecutePathNode>(mPickContext, wpPtr, seqNode); 

            std::vector<NodePtr> fullSeq = {planTree, execTree}; 
            auto topNode = std::make_shared<SequenceNode>(fullSeq); 
            tree = topNode;
            break;
        }
        case ManipulationCommand::CMD_PLACE: 
        {
            LOGD << "GOT PLACE CMD"; 
            break;
        }     
        case ManipulationCommand::CMD_PLACE_REL:
        {
            LOGD << "GOT PLACE REL CMD"; 
            break;
        }     
        // case ManipulationCommand::EXE_PICK:
        // {
        //     LOGV << "Executing Pick operation!"; 
        //     if(nullptr == mPickContext)
        //     {
        //         // could probably plan pick here before executing 
        //         return; 
        //     }

        //     auto wpPtr = std::make_shared<KDL::JntArray>(); 
            
        //     auto sendWpNode = std::make_shared<SendWaypointNode>(wpPtr); 
        //     std::vector<NodePtr> nodes = {sendWpNode};

        //     auto seqNode = std::make_shared<SequenceNode>(nodes);
        //     auto execPathNode = std::make_shared<ExecutePathNode>(mPickContext->mPath, wpPtr); 
        //     tree = execPathNode; 
        //     break;
        // }
        default:
        {
            LOGV << "Setting tree to null"; 
            tree = nullptr; 
            break;
        }       
    }
 
    {
        std::lock_guard<std::mutex> lock(mTreeMutex);
        LOGV << "Setting active tree";  
        mActiveTree = tree; 
        mStatus = INode::Status::RUNNING; // set status to running so main loop will tick the new tree
    }
}