#include "plog/Log.h"

#include "arm_hardware/ManipulatorFactory.h"
#include "arm_hardware/GazeboManipComms.h"
#include "arm_hardware/DynamixelManipComms.h"
#include "arm_hardware/SimManipComms.hpp"

std::shared_ptr<IManipComms> ManipulatorFactory::create(const YAML::Node& aConfig)
{
    std::string manipType = aConfig["type"].as<std::string>();
    std::string commsType = aConfig["comms"].as<std::string>();

    if(("ur10" == manipType || "ur5" == manipType) && "gazebo" == commsType)
    {
        LOGW << "Using GazeboManipComms";
        return std::make_shared<GazeboManipComms>(manipType);
    }
    else if ("dynamixel" == manipType && "dynamixel" == commsType)
    {
        LOGW << "Using DynamixelManipComms";
        return std::make_shared<DynamixelManipComms>(aConfig["comms_configs"].as<YAML::Node>());
    }
    else if("simulated" == commsType)
    {
        // any manip type can use this comms type
        return std::make_shared<SimManipComms>(aConfig["joint_names"].as<std::vector<std::string>>());
    }
    else
    {
        LOGE << "Manip type: " << manipType << " not yet supported";
    }

}
