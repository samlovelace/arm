#ifndef POINTCLOUDHANDLER_H
#define POINTCLOUDHANDLER_H
 
#include <string>  
#include <pcl/io/ply_io.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/msg/point_cloud2.hpp>

class PointCloudHandler 
{ 
    
public:
    PointCloudHandler();
    ~PointCloudHandler();

    static bool fromFile(const std::string& aFilePath, sensor_msgs::msg::PointCloud2& aCloudOut); 
    static bool toFile(const std::string& aFilePath, sensor_msgs::msg::PointCloud2& aCloud); 

private:
   
};
#endif //POINTCLOUDHANDLER_H