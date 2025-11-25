#ifndef PLANNERFACTORY_H
#define PLANNERFACTORY_H
 
#include "IArmTaskPlanner.hpp"
#include "IGraspPlanner.hpp"
#include "TrajectoryPlanner.h"
#include <memory>  
#include <yaml-cpp/yaml.h>

class PlannerFactory 
{ 
public:
    PlannerFactory();
    ~PlannerFactory();

    static std::shared_ptr<IArmTaskPlanner> createArmTaskPlanner(const std::string& aPlannerType, 
                                                                 std::shared_ptr<KinematicsHandler> aKine, 
                                                                std::shared_ptr<TrajectoryPlanner> aTrajPlanner); 
    static std::shared_ptr<IGraspPlanner> createGraspPlanner(const YAML::Node& aGraspPlanningConfig); 

private:
   
};
#endif //PLANNERFACTORY_H