#ifndef SIMPLEGRIPPERVISUALIZER_H
#define SIMPLEGRIPPERVISUALIZER_H
 
#include <pcl/visualization/pcl_visualizer.h>
#include <Eigen/Dense>
#include <cmath>
#include <string>
#include <vtkCubeSource.h>
#include <vtkTransform.h>
#include <vtkTransformPolyDataFilter.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkNamedColors.h> 

class SimpleGripperVisualizer 
{ 
public:
    SimpleGripperVisualizer();
    ~SimpleGripperVisualizer();

    static void addOrientedCube(pcl::visualization::PCLVisualizer::Ptr viewer,
                        const Eigen::Vector3f& local_center,
                        const Eigen::Matrix3f& rotation,
                        const Eigen::Vector3f& translation,
                        float dx, float dy, float dz,
                        double r, double g, double b,
                        const std::string& id);

    static void draw_gripper(pcl::visualization::PCLVisualizer::Ptr viewer,
                      const Eigen::Vector3f& gripper_translation,
                      const Eigen::Matrix3f& gripper_rotation,
                      const std::string& gripper_id_prefix = "gripper");


private:
   
};
#endif //SIMPLEGRIPPERVISUALIZER_H