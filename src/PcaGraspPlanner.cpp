
#include "PcaGraspPlanner.h"
#include "plog/Log.h"

#include <pcl/common/pca.h>
#include <pcl/common/centroid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "SimpleGripperVisualizer.h"

PcaGraspPlanner::PcaGraspPlanner(const YAML::Node& aConfig) : mVisualize(false)
{
    mVisualize = aConfig["visualize"].as<bool>(); 
}

PcaGraspPlanner::~PcaGraspPlanner()
{
    if (mVisualizeThread.joinable())
    {
        mStopVisualization = true;
        mVisualizeThread.join();
    }
}

bool PcaGraspPlanner::plan(pcl::PointCloud<pcl::PointXYZ>::Ptr aCloud, Eigen::Affine3f& aT_G_ee)
{
    // save object cloud
    mCloud = aCloud; 

    // Reset planned grasp pose
    mT_G_ee = Eigen::Affine3f::Identity(); 

    // PCA
    pcl::PCA<pcl::PointXYZ> pca;
    pca.setInputCloud(aCloud);

    Eigen::Matrix3f eigenvectors = pca.getEigenVectors();
    Eigen::Vector3f eigenvalues = pca.getEigenValues();
    Eigen::Vector4f centroid;

    if(0 == pcl::compute3DCentroid(*aCloud, centroid))
    {
        LOGE << "Failed to compute object cloud centroid"; 
        return false; 
    }

    LOGD << "Object Centroid from PCL: " << centroid.head<3>(); 

    Eigen::Vector4f manual_centroid(0.0f, 0.0f, 0.0f, 0.0f);
    int valid_points = 0;

    for (const auto& pt : aCloud->points) 
    {
        if (!std::isnan(pt.x) && !std::isnan(pt.y) && !std::isnan(pt.z)) 
        {
            manual_centroid[0] += pt.x;
            manual_centroid[1] += pt.y;
            manual_centroid[2] += pt.z;
            ++valid_points;
        }
    }

    if (valid_points > 0) {
        manual_centroid /= valid_points;
        manual_centroid[3] = 1.0f; // Homogeneous coordinate
    }
    
    LOGD << "Manually computed object centroid: " << manual_centroid.head<3>(); 

    if (!eigenvectors.allFinite())
    {
        LOGE << "Eigenvectors contain NaN or Inf!";
        return false;
    }

    Eigen::MatrixXf cloud_matrix(aCloud->points.size(), 3);
    for (size_t i = 0; i < aCloud->points.size(); i++)
    {
        cloud_matrix.row(i) << aCloud->points[i].x,
                               aCloud->points[i].y,
                               aCloud->points[i].z;
    }

    cloud_matrix.rowwise() -= centroid.head<3>().transpose();

    Eigen::MatrixXf cloud_pca = cloud_matrix * eigenvectors;

    Eigen::Vector3f min_pca = cloud_pca.colwise().minCoeff();
    Eigen::Vector3f max_pca = cloud_pca.colwise().maxCoeff();
    Eigen::Vector3f extents = max_pca - min_pca;

    int thin_axis = 0; 
    extents.minCoeff(&thin_axis); 
    Eigen::Vector3f a_min = eigenvectors.col(thin_axis); 

    // Step 1: closing_dir = min axis
    Eigen::Vector3f gripper_y = a_min.normalized();

    // Step 2: pick approach_dir perpendicular to closing_dir
    Eigen::Vector3f temp(1, 0, 0);
    if (std::abs(gripper_y.dot(temp)) > 0.99)
        temp = Eigen::Vector3f(0, 1, 0);

    Eigen::Vector3f gripper_z = gripper_y.cross(temp);
    gripper_z.normalize();

    // Step 3: compute grasp axis
    Eigen::Vector3f gripper_x = gripper_y.cross(gripper_z);
    gripper_x.normalize();

    // Step 4: build rotation matrix
    Eigen::Matrix3f R;
    R.col(1) = gripper_x; // grasp_axis_dir
    R.col(0) = gripper_y; // closing_dir
    R.col(2) = gripper_z; // approach_dir

    std::cout << "R = \n" << R.format(Eigen::IOFormat(3, 0, ", ", "\n", "[", "]")) << std::endl;

    // Approach direction in PCA frame
    Eigen::Vector3f approach_dir_in_pca = eigenvectors.transpose() * gripper_z;
    approach_dir_in_pca.normalize();

    // Compute extent along approach axis
    float proj_min = min_pca.dot(approach_dir_in_pca);
    float proj_max = max_pca.dot(approach_dir_in_pca);

    float extent_along_approach = std::abs(proj_max - proj_min);
    float half_extent_along_approach = extent_along_approach / 2.0f;

    std::cout << "Extent along approach axis: " << extent_along_approach << " m\n";
    std::cout << "Half extent along approach axis: " << half_extent_along_approach << " m\n";

    // Choose offset
    float clearance = 0.05f; // meters
    float offset_along_approach = half_extent_along_approach + clearance;

    Eigen::Vector3f p_local(0, 0, -offset_along_approach);
    Eigen::Vector3f grasp_position = R * p_local + centroid.head<3>();

    mT_G_ee.translate(grasp_position); 
    mT_G_ee.rotate(R); 
    aT_G_ee = mT_G_ee; 

    if (mVisualize)
    {
        // Stop any previous visualization thread
        if (mVisualizeThread.joinable())
        {
            mStopVisualization = true;
            mVisualizeThread.join();
        }
        mStopVisualization = false;
        //mVisualizeThread = std::thread(&PcaGraspPlanner::visualizeGraspPlan, this);

        // TODO: extract visualizer out of here, should be agnostic to the specific GraspPlanner
        mVisualizeThread = std::thread([this]() {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_copy(new pcl::PointCloud<pcl::PointXYZ>(*mCloud));
            Eigen::Affine3f pose_copy = mT_G_ee;

            pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Grasp Pose Viewer"));
            viewer->setBackgroundColor(0, 0, 0);
            viewer->addPointCloud<pcl::PointXYZ>(cloud_copy, "object_cloud");
            viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "object_cloud");
            viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, "object_cloud");
            viewer->addCoordinateSystem(0.05);
            SimpleGripperVisualizer::draw_gripper(viewer, pose_copy.translation(), pose_copy.rotation());

            while (!viewer->wasStopped() && !mStopVisualization) {
                viewer->spinOnce(10);
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }

            viewer->close();
        });
    }

    return true;
}   

void PcaGraspPlanner::visualizeGraspPlan()
{
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Grasp Pose Viewer"));
    viewer->setBackgroundColor(0, 0, 0);

    viewer->addPointCloud<pcl::PointXYZ>(mCloud, "object_cloud");
    viewer->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
        2,
        "object_cloud");
    viewer->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_COLOR,
        0.0, 1.0, 0.0,
        "object_cloud");

    viewer->addCoordinateSystem(0.05);

    float axis_length = 0.05f;

    SimpleGripperVisualizer::draw_gripper(viewer, mT_G_ee.translation(), mT_G_ee.rotation()); 

    // Spin viewer
    while (!viewer->wasStopped() && !mStopVisualization)
    {
        std::lock_guard<std::mutex> lock(mVisMtx); 
        viewer->spinOnce(10);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));  
    }

    viewer->close(); 
}