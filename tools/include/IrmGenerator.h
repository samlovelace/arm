#ifndef IRMGENERATOR_H
#define IRMGENERATOR_H
 
#include "ConfigManager.h"
#include "KinematicsHandler.h" 

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

    void compute(const KDL::JntArray& aJntCfg); 
   
};
#endif //IRMGENERATOR_H