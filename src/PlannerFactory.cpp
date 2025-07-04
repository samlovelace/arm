
#include "PlannerFactory.h"
#include "MobileArmTaskPlanner.h"
#include "PcaGraspPlanner.h"
#include "plog/Log.h"

std::shared_ptr<IArmTaskPlanner> PlannerFactory::createArmTaskPlanner(const std::string& aPlannerType)
{
    if("mobile" == aPlannerType)
    {
        return std::make_shared<MobileArmTaskPlanner>(); 
    }
    else
    {
        LOGE << "Unsupported planner type: " << aPlannerType; 
        return nullptr; 
    }
}

std::shared_ptr<IGraspPlanner> PlannerFactory::createGraspPlanner(const YAML::Node& aGraspPlanningConfig)
{
    std::string type = aGraspPlanningConfig["type"].as<std::string>(); 

    if("pca" == type)
    {
        return std::make_shared<PcaGraspPlanner>(aGraspPlanningConfig); 
    }
    else
    {
        LOGE << "Unsupported grasp planner type: " << type; 
        return nullptr; 
    }
}