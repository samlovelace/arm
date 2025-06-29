
#include "MobileArmTaskPlanner.h"
#include "plog/Log.h"

MobileArmTaskPlanner::MobileArmTaskPlanner()
{

}

MobileArmTaskPlanner::~MobileArmTaskPlanner()
{

}

bool MobileArmTaskPlanner::plan(const std::string& aTaskType)
{
    LOGD << "MobileArmTaskPLanner planning for task: " << aTaskType; 
    return true; 
}