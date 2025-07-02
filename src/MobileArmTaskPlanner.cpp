#include "MobileArmTaskPlanner.h"
#include "plog/Log.h"

#include <pcl/common/pca.h>
#include <pcl/common/centroid.h>
#include <pcl/visualization/pcl_visualizer.h>
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

    int grasp_axis = 0;
    extents.minCoeff(&grasp_axis);

    Eigen::Vector3f grasp_axis_dir = eigenvectors.col(grasp_axis);

    std::vector<int> axes = {0, 1, 2};
    axes.erase(axes.begin() + grasp_axis);

    Eigen::Vector3f approach_dir = eigenvectors.col(axes[0]);
    Eigen::Vector3f closing_dir  = eigenvectors.col(axes[1]);

    if (!approach_dir.allFinite() || !closing_dir.allFinite() || !grasp_axis_dir.allFinite())
    {
        LOGE << "Computed grasp directions contain NaN or Inf!";
        return false;
    }

    Eigen::Matrix3f R;
    R.col(0) = approach_dir;
    R.col(1) = closing_dir;
    R.col(2) = grasp_axis_dir;

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

    // -------------------------
    // GRIPPER VISUALIZATION
    // -------------------------

    Eigen::Affine3f gripper_tf;
    gripper_tf.linear() = R;
    gripper_tf.translation() = grasp_position;

    Eigen::Vector3f palm_translation = gripper_tf.translation();
    Eigen::Quaternionf palm_rotation(gripper_tf.linear());

    viewer->addCube(
        palm_translation,
        palm_rotation,
        0.05,  // width along approach (X)
        0.02,  // height along closing dir (Y)
        0.04,  // depth along grasp axis (Z)
        "gripper_palm"
    );
    viewer->setShapeRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_COLOR,
        0.9, 0.9, 0.1,
        "gripper_palm");
    viewer->setShapeRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_OPACITY,
        0.5,
        "gripper_palm");

    float gripper_opening = 0.15; // meters
    float finger_offset = gripper_opening / 2.0;

    Eigen::Vector3f finger_pos_left = grasp_position + closing_dir * finger_offset;
    Eigen::Vector3f finger_pos_right = grasp_position - closing_dir * finger_offset;

    Eigen::Affine3f finger_left_tf = Eigen::Affine3f::Identity();
    finger_left_tf.linear() = R;
    finger_left_tf.translation() = finger_pos_left;

    Eigen::Vector3f finger_left_translation = finger_left_tf.translation();
    Eigen::Quaternionf finger_left_rotation(finger_left_tf.linear());

    viewer->addCube(
        finger_left_translation,
        finger_left_rotation,
        0.01,
        0.02,
        0.04,
        "gripper_finger_left"
    );
    viewer->setShapeRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_COLOR,
        1.0, 0.2, 0.2,
        "gripper_finger_left");
    viewer->setShapeRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_OPACITY,
        0.5,
        "gripper_finger_left");

    Eigen::Affine3f finger_right_tf = Eigen::Affine3f::Identity();
    finger_right_tf.linear() = R;
    finger_right_tf.translation() = finger_pos_right;

    Eigen::Vector3f finger_right_translation = finger_right_tf.translation();
    Eigen::Quaternionf finger_right_rotation(finger_right_tf.linear());

    viewer->addCube(
        finger_right_translation,
        finger_right_rotation,
        0.01,
        0.02,
        0.04,
        "gripper_finger_right"
    );
    viewer->setShapeRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_COLOR,
        1.0, 0.2, 0.2,
        "gripper_finger_right");
    viewer->setShapeRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_OPACITY,
        0.5,
        "gripper_finger_right");

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
