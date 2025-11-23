#ifndef POINTCLOUDHANDLER_H
#define POINTCLOUDHANDLER_H
 
#include <string>  
#include <pcl/io/ply_io.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "plog/Log.h"

class PointCloudHandler 
{ 
    
public:
    PointCloudHandler();
    ~PointCloudHandler();

    static bool fromFile(const std::string& aFilePath, sensor_msgs::msg::PointCloud2& aCloudOut); 
    static bool toFile(const std::string& aFilePath, sensor_msgs::msg::PointCloud2& aCloud); 
    
    template<typename PointT>
    static bool toPCL(sensor_msgs::msg::PointCloud2& aRosCloud, pcl::PointCloud<PointT>& aPclCloudOut)
    {
        LOGV << "Converting point cloud to PCL"; 
        pcl::PCLPointCloud2 pcl_pc2;
        pcl_conversions::toPCL(aRosCloud, pcl_pc2);
        pcl::fromPCLPointCloud2(pcl_pc2, aPclCloudOut);
        LOGV << "Done converting"; 
    }

private:
   
};
#endif //POINTCLOUDHANDLER_H