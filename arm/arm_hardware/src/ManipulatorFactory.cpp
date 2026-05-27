#include "plog/Log.h"

#include "arm_common/ConfigManager.h"

#include "arm_hardware/ManipulatorFactory.h"
#include "arm_hardware/GazeboManipComms.h"
#include "arm_hardware/DynamixelManipComms.h"
#include "arm_hardware/SimManipComms.hpp"

std::shared_ptr<IManipComms> ManipulatorFactory::create()
{
    auto config = ConfigManager::getInstance()->getConfig(); 

    if(("ur10" == config.manipType || "ur5" == config.manipType) && "gazebo" == config.commsMode)
    {
        LOGW << "Using GazeboManipComms";
        return std::make_shared<GazeboManipComms>(config.manipType);
    }
    else if ("dynamixel" == config.manipType && "dynamixel" == config.commsMode)
    {
        LOGW << "Using DynamixelManipComms";
        return std::make_shared<DynamixelManipComms>(ConfigManager::getInstance()->getRawNode()["comms_config"]);
    }
    else if("simulated" == config.commsMode)
    {
        // any manip type can use this comms type
        return std::make_shared<SimManipComms>(config.jointNames);
    }
    else
    {
        LOGE << "Manip type: " << config.manipType << " not yet supported";
    }

}
