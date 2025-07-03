#include "MobileArmTaskPlanner.h"
#include "plog/Log.h"

#include <pcl/common/pca.h>
#include <pcl/common/centroid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "SimpleGripperVisualizer.h"
#include <thread>
#include <chrono>

MobileArmTaskPlanner::MobileArmTaskPlanner()
{

}

MobileArmTaskPlanner::~MobileArmTaskPlanner()
{

}

bool MobileArmTaskPlanner::planPick(const Eigen::Vector3d& /*aCentroid_G*/, pcl::PointCloud<pcl::PointXYZ>::Ptr aCloud)
{
    LOGD << "MobileArmTaskPlanner planning for pick task"; 

    if (!aCloud || aCloud->empty())
    {
        LOGE << "Input cloud is empty!";
        return false;
    }

    // PCA
    pcl::PCA<pcl::PointXYZ> pca;
    pca.setInputCloud(aCloud);

    Eigen::Matrix3f eigenvectors = pca.getEigenVectors();
    Eigen::Vector3f eigenvalues = pca.getEigenValues();
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*aCloud, centroid);

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

    // Find axis of smallest extent
    int thin_axis = 0;
    extents.minCoeff(&thin_axis);
    Eigen::Vector3f grasp_axis_dir = eigenvectors.col(thin_axis);

    // Choose closing axis among the other two
    std::vector<int> axes = {0, 1, 2};
    axes.erase(axes.begin() + thin_axis);

    int closing_axis =
        (extents(axes[0]) > extents(axes[1])) ? axes[0] : axes[1];

    Eigen::Vector3f closing_dir = eigenvectors.col(closing_axis);

    // Compute approach direction
    Eigen::Vector3f approach_dir = grasp_axis_dir.cross(closing_dir).normalized();

    LOGD << "Object extents along PCA axes: " << extents.transpose();

    Eigen::Matrix3f R;
    R.col(0) = approach_dir;
    R.col(1) = closing_dir;
    R.col(2) = grasp_axis_dir;

    float angle_deg = 45.0f;
    float angle_rad = angle_deg * M_PI / 180.0f;

    Eigen::Matrix3f rotZ;
    rotZ = Eigen::AngleAxisf(angle_rad, Eigen::Vector3f::UnitY());

    // Rotate
    R = rotZ * R;

    Eigen::Vector3f grasp_position = centroid.head<3>();

    Eigen::Matrix4f grasp_pose = Eigen::Matrix4f::Identity();
    grasp_pose.block<3,3>(0,0) = R;
    grasp_pose.block<3,1>(0,3) = grasp_position;

    LOGD << "Computed grasp pose at centroid: " 
         << grasp_position.transpose();

    // -------------------------
    // VISUALIZATION
    // -------------------------

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Grasp Pose Viewer"));
    viewer->setBackgroundColor(0, 0, 0);

    viewer->addPointCloud<pcl::PointXYZ>(aCloud, "object_cloud");
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

    pcl::PointXYZ origin(grasp_position(0), grasp_position(1), grasp_position(2));

    pcl::PointXYZ x_end(
        grasp_position(0) + axis_length * approach_dir(0),
        grasp_position(1) + axis_length * approach_dir(1),
        grasp_position(2) + axis_length * approach_dir(2));
    pcl::PointXYZ y_end(
        grasp_position(0) + axis_length * closing_dir(0),
        grasp_position(1) + axis_length * closing_dir(1),
        grasp_position(2) + axis_length * closing_dir(2));
    pcl::PointXYZ z_end(
        grasp_position(0) + axis_length * grasp_axis_dir(0),
        grasp_position(1) + axis_length * grasp_axis_dir(1),
        grasp_position(2) + axis_length * grasp_axis_dir(2));

    viewer->addLine(origin, x_end, 1.0, 0.0, 0.0, "grasp_x");
    viewer->addLine(origin, y_end, 0.0, 1.0, 0.0, "grasp_y");
    viewer->addLine(origin, z_end, 0.0, 0.0, 1.0, "grasp_z");

    SimpleGripperVisualizer::draw_gripper(viewer, grasp_position, R); 

    // Spin viewer
    while (!viewer->wasStopped())
    {
        viewer->spin(); 
        //viewer->spinOnce(100);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    return true;
}

bool MobileArmTaskPlanner::planPlace(const Eigen::Vector3d& /*aPlacePos_G*/)
{
    LOGD << "MobileArmTaskPlanner planning for place task"; 
    return true; 
}
