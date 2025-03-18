
#include "ManipulatorFactory.h"
#include "plog/Log.h"

IManipComms* ManipulatorFactory::create(const std::string& aManipType)
{
    if("test" == aManipType)
    {
        return nullptr; 
    }
    else
    {
        LOGE << "Manip type: " << aManipType << " not yet supported"; 
    }

}