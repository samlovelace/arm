
#include "ManipulatorFactory.h"
#include "plog/Log.h"
#include "GazeboManipComms.h"
#include "DynamixelManipComms.h"

std::shared_ptr<IManipComms> ManipulatorFactory::create(const YAML::Node& aConfig)
{
    std::string manipType = aConfig["type"].as<std::string>();
    std::string commsType = aConfig["comms"].as<std::string>();

    if(("ur10" == manipType || "ur5" == manipType) && "simulated" == commsType)
    {
        LOGW << "Using GazeboManipComms"; 
        return std::make_shared<GazeboManipComms>(manipType);  
    }
    else if ("dynamixel" == manipType && "dynamixel" == commsType)
    {
        LOGW << "Using DynamixelManipComms"; 
        return std::make_shared<DynamixelManipComms>(aConfig["comms_configs"].as<YAML::Node>()); 
    }
    else
    {
        LOGE << "Manip type: " << manipType << " not yet supported"; 
    }

}