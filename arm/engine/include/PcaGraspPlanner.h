#ifndef GRASPPLANNER
#define GRASPPLANNER

#include "IGraspPlanner.hpp"
#include <yaml-cpp/yaml.h>
#include <thread> 
#include <mutex> 
 
class PcaGraspPlanner : public IGraspPlanner
{ 
public:
    PcaGraspPlanner(const YAML::Node& aConfig);
    ~PcaGraspPlanner();

    bool plan(pcl::PointCloud<pcl::PointXYZ>::Ptr aCloud, Eigen::Affine3f& aT_G_ee) override; 

private:

    pcl::PointCloud<pcl::PointXYZ>::Ptr mCloud; 
    Eigen::Affine3f mT_G_ee; 

    bool mVisualize; 
    std::mutex mVisMtx; 
    std::atomic<bool> mStopVisualization {false};

    void visualizeGraspPlan();
    std::thread mVisualizeThread; 
   
};
#endif //GRASPPLANNER