
#include "IrmGenerator.h"
#include <iostream> 

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

    float resolution = 10;
    KDL::JntArray jntCfg(numJoints); 
    std::vector<size_t> indices(numJoints, 0); 

    size_t total_samples = 1;
    for (int r = 0; r < numJoints; r++)
        total_samples *= (resolution + 1);

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

}

void IrmGenerator::compute(const KDL::JntArray& aJntCfg)
{
    KDL::Frame frame; 
    if(!mKinematicsHandler->solveFk(aJntCfg, frame))
    {
        std::cout << "Failed to compute forward kinematics for jntCfg: " << aJntCfg.data << ". Skipping..."; 
        return; 
    }
    std::cout << "JointCfg: " << aJntCfg.data << std::endl; 
    std::cout << "Frame Pos: " << frame.p.x() << ", " << frame.p.y() << ", " << frame.p.z() << std::endl; 

    // do something with the frame? 
}

