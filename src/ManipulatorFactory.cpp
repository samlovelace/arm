
#include "ManipulatorFactory.h"
#include "plog/Log.h"
#include "GazeboManipComms.h"

std::shared_ptr<IManipComms> ManipulatorFactory::create(const std::string& aManipType)
{
    if("UR" == aManipType)
    {
        return std::make_shared<GazeboManipComms>(); 
    }
    else
    {
        LOGE << "Manip type: " << aManipType << " not yet supported"; 
    }

}