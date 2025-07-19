#ifndef IRMGENERATOR_H
#define IRMGENERATOR_H
 
#include "ConfigManager.h"
#include "KinematicsHandler.h" 
#include "nlohmann/json.hpp"

/**
 * @brief top level class for handling the generation of a Inverse Reachability Map (IRM)
 */
class IrmGenerator 
{ 
public:
    IrmGenerator(ConfigManager::Config aConfig);
    ~IrmGenerator();

    bool generate(); 

private:

    std::shared_ptr<KinematicsHandler> mKinematicsHandler; 
    ConfigManager::Config mConfig; 

    nlohmann::json mIrmEntries; 

    void compute(const KDL::JntArray& aJntCfg); 

    bool toFile(); 
   
};
#endif //IRMGENERATOR_H