#pragma once

#include "ConfigManager.h"
#include "KinematicsHandler.h"
#include <kdl/jntarray.hpp>
#include <kdl/frames.hpp>
#include <memory>
#include <vector>
#include <fstream>
#include <cmath>
#include <iostream>

struct IrmEntryBinary
{
    double position[3];        // P_ee_base (x, y, z)
    double orientation[9];     // R_ee_base (row-major 3x3 rotation matrix)
    double manipulability;     // manipulability measure
};

class IrmGenerator
{
public:
    IrmGenerator(ConfigManager::Config aConfig);
    ~IrmGenerator();

    bool generate();
    bool toFile();

private:
    bool compute(const KDL::JntArray& aJntCfg);

    ConfigManager::Config mConfig;
    std::shared_ptr<KinematicsHandler> mKinematicsHandler;
    std::vector<IrmEntryBinary> mIrmEntries;
};
