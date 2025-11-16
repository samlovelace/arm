
#include "PointCloudHandler.h"
#include "plog/Log.h"

PointCloudHandler::PointCloudHandler()
{

}

PointCloudHandler::~PointCloudHandler()
{

}

bool PointCloudHandler::fromFile(const std::string& aFilePath, sensor_msgs::msg::PointCloud2& aCloudOut)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud; 
    if(pcl::io::loadPLYFile<pcl::PointXYZ>(aFilePath, *cloud) == -1)
    {
        std::cerr << "Failed to load object point cloud from " << aFilePath; 
        return false; 
    }

    pcl::PCLPointCloud2 pcl_pc2;
    pcl::toPCLPointCloud2(*cloud, pcl_pc2);

    pcl_conversions::fromPCL(pcl_pc2, aCloudOut);
    aCloudOut.header.frame_id = "map";

    return true; 
}

bool PointCloudHandler::toFile(const std::string& aFilePath, sensor_msgs::msg::PointCloud2& aCloud)
{
    // Convert to pcl::PCLPointCloud2
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(aCloud, pcl_pc2);

    // Convert to pcl::PointCloud<pcl::PointXYZ>
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2, *cloud);

    if(pcl::io::savePLYFileASCII<pcl::PointXYZ>(aFilePath, *cloud))
    {
        LOGW << "Failed to save point cloud to file"; 
        return false; 
    }

    return true; 
}