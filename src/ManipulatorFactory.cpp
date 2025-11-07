
#include "ManipulatorFactory.h"
#include "plog/Log.h"
#include "GazeboManipComms.h"
#include "DynamixelManipComms.h"

std::shared_ptr<IManipComms> ManipulatorFactory::create(const std::string& aManipType, const std::string& aCommsType)
{
    if(("ur10" == aManipType || "ur5" == aManipType) && "simulated" == aCommsType)
    {
        return std::make_shared<GazeboManipComms>(aManipType);  
    }
    else if ("dynamixel" == aManipType && "dynamixel" == aCommsType)
    {
        return std::make_shared<DynamixelManipComms>(); 
    }
    else
    {
        LOGE << "Manip type: " << aManipType << " not yet supported"; 
    }

}