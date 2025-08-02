
#include "IrmGenerator.h"
#include <iostream> 
#include <fstream> 
#include <nlohmann/json.hpp>

IrmGenerator::IrmGenerator(ConfigManager::Config aConfig) : mConfig(aConfig)
{
    std::string urdfFilePath = mConfig.shareDir + "manipulators/" + mConfig.manipType + "/manipulator.urdf";
    std::cout << "urdfFilePath: " << urdfFilePath << std::endl; 

    mKinematicsHandler = std::make_shared<KinematicsHandler>(); 
    if(!mKinematicsHandler->init(urdfFilePath))
    {
        throw(std::runtime_error("Failed to initialize kinematics handler\n"));  
    } 

    std::cout << "Initialized kinematics handler!\n"; 

    // init the json array 
    mIrmEntries = nlohmann::json::array(); 
}

IrmGenerator::~IrmGenerator()
{

}

bool IrmGenerator::generate()
{
    std::cout << "Generating Inverse Reachability Map for " << mConfig.manipType << std::endl; 

    KDL::JntArray jntMaxLimits = mKinematicsHandler->getJointLimits("upper"); 
    KDL::JntArray jntMinLimits = mKinematicsHandler->getJointLimits("lower"); 

    int numJoints = jntMinLimits.rows(); 
    std::cout << "NumJoints: " << numJoints << std::endl; 
    std::cout << "Lower Joint Limits: " << jntMinLimits.data << std::endl;
    std::cout << "Upper Joint Limits: " << jntMaxLimits.data << std::endl;

    // TODO: make configurable 
    float resolution = 12;
    KDL::JntArray jntCfg(numJoints); 
    std::vector<size_t> indices(numJoints, 0); 

    size_t total_samples = 1;
    for (int r = 0; r < numJoints; r++)
        total_samples *= (resolution + 1);

    std::cout << "Generating IRM with " << total_samples << " samples" << std::endl; 

    for(int samples = 0; samples < total_samples; samples++)
    {
        for(int i = 0; i < numJoints; i++)
        {
            float step = (jntMaxLimits(i) - jntMinLimits(i)) / resolution; 
            jntCfg(i) = jntMinLimits(i) + step * indices[i]; 
        }

        // this is where the fun begins 
        compute(jntCfg); 
    
        // Increment multi-digit counter
        for (size_t j = 0; j < numJoints; ++j) 
        {
            if (++indices[j] <= resolution)
                break;
            
            indices[j] = 0;
        }
    
    }

    if(!toFile())
    {
        std::cerr << "Failed to write IRM to file\n"; 
        return false; 
    }

}

void IrmGenerator::compute(const KDL::JntArray& aJntCfg)
{
    // compute the manipulability for this jnt config 
    double manipulability = mKinematicsHandler->computeManipulability(aJntCfg); 
    
    // TODO: make this smarter/adaptive
    double manipulabilityThreshold = 0.0;  
    if(isnan(manipulability) || manipulability < manipulabilityThreshold)
    {
        //std::cout << "Low manipulability joint config. Skipping..." << std::endl; 
        return;         
    }

    KDL::Frame frame; 
    if(!mKinematicsHandler->solveFk(aJntCfg, frame))
    {
        std::cout << "Failed to compute forward kinematics for jntCfg: " << aJntCfg.data << ". Skipping..."; 
        return; 
    }

    using json = nlohmann::json; 

    KDL::Frame inverse = frame.Inverse(); 

    json irmEntry; 
    irmEntry["P_ee_base"] = inverse.p.data;
    irmEntry["R_ee_base"] = inverse.M.data; 
    irmEntry["joints"] = aJntCfg.data; 
    irmEntry["manipulability"] = manipulability; 

    mIrmEntries.push_back(irmEntry); 
}

bool IrmGenerator::toFile()
{
    using Json = nlohmann::json;
    nlohmann::json finalOutput;
    finalOutput["irm"] = mIrmEntries;
    
    std::ofstream outfile("IRM.json"); 

    if(!outfile.is_open())
    {
        std::cerr << "Failed to open json file for write\n"; 
        return false; 
    }

    outfile << finalOutput.dump(2); 
    outfile.close();
    return true;  
}

