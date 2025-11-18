
#include "PlannerFactory.h"
#include "SimpleMobileTaskPlanner.h"
#include "PcaGraspPlanner.h"
#include "plog/Log.h"

std::shared_ptr<IArmTaskPlanner> PlannerFactory::createArmTaskPlanner(const std::string& aPlannerType)
{
    if("mobile" == aPlannerType)
    {
        // TODO: add this class from controller 
        //return std::make_shared<MobileArmTaskPlanner>(); 
    }
    if("simple" == aPlannerType)
    {
        LOGV << "Using Simple Mobile Task Planner"; 
        return std::make_shared<SimpleMobileTaskPlanner>(); 
    }
    else
    {
        LOGE << "Unsupported planner type: " << aPlannerType; 
        return nullptr; 
    }
}

std::shared_ptr<IGraspPlanner> PlannerFactory::createGraspPlanner(const YAML::Node& aGraspPlanningConfig)
{
    LOGV << YAML::Dump(aGraspPlanningConfig); 
    std::string type = aGraspPlanningConfig["type"].as<std::string>(); 

    if("pca" == type)
    {
        LOGV << "Using PCA GraspPlanner"; 
        return std::make_shared<PcaGraspPlanner>(aGraspPlanningConfig); 
    }
    else
    {
        LOGE << "Unsupported grasp planner type: " << type; 
        return nullptr; 
    }
}