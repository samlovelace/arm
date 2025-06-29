
#include "PlannerFactory.h"
#include "MobileArmTaskPlanner.h"
#include "plog/Log.h"

std::shared_ptr<IArmTaskPlanner> PlannerFactory::create(const std::string& aPlannerType)
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