
#include "SimpleGripperVisualizer.h"

SimpleGripperVisualizer::SimpleGripperVisualizer()
{

}

SimpleGripperVisualizer::~SimpleGripperVisualizer()
{

}

void SimpleGripperVisualizer::addOrientedCube(
    pcl::visualization::PCLVisualizer::Ptr viewer,
    const Eigen::Vector3f& local_center,
    const Eigen::Matrix3f& rotation,
    const Eigen::Vector3f& translation,
    float dx, float dy, float dz,
    double r, double g, double b,
    const std::string& id)
{
    // Create cube centered at origin with specified size
    auto cube = vtkSmartPointer<vtkCubeSource>::New();
    cube->SetXLength(dx);
    cube->SetYLength(dy);
    cube->SetZLength(dz);
    cube->Update();

    // Compute final center position
    Eigen::Vector3f global_center = rotation * local_center + translation;

    // Convert Eigen to VTK matrix
    vtkSmartPointer<vtkMatrix4x4> vtk_mat = vtkSmartPointer<vtkMatrix4x4>::New();
    vtk_mat->Identity();

    vtk_mat->SetElement(0, 0, rotation(0, 0));
    vtk_mat->SetElement(0, 1, rotation(0, 1));
    vtk_mat->SetElement(0, 2, rotation(0, 2));
    vtk_mat->SetElement(1, 0, rotation(1, 0));
    vtk_mat->SetElement(1, 1, rotation(1, 1));
    vtk_mat->SetElement(1, 2, rotation(1, 2));
    vtk_mat->SetElement(2, 0, rotation(2, 0));
    vtk_mat->SetElement(2, 1, rotation(2, 1));
    vtk_mat->SetElement(2, 2, rotation(2, 2));

    vtk_mat->SetElement(0, 3, global_center(0));
    vtk_mat->SetElement(1, 3, global_center(1));
    vtk_mat->SetElement(2, 3, global_center(2));

    // Transform the cube
    auto transform = vtkSmartPointer<vtkTransform>::New();
    transform->SetMatrix(vtk_mat);

    auto transformFilter = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
    transformFilter->SetTransform(transform);
    transformFilter->SetInputConnection(cube->GetOutputPort());
    transformFilter->Update();

    viewer->addModelFromPolyData(transformFilter->GetOutput(), id);

    // Color the actor
    viewer->setShapeRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_COLOR,
        r, g, b,
        id
    );
}

void SimpleGripperVisualizer::draw_gripper(pcl::visualization::PCLVisualizer::Ptr viewer,
                                           const Eigen::Vector3f& gripper_translation,
                                           const Eigen::Matrix3f& gripper_rotation,
                                           const std::string& gripper_id_prefix)
{
    // TODO: make finger spacing and finger length configurable, could probably use this for simple vis of grasp plan for any gripper
    // Gripper dimensions
    float wrist_size = 0.02;
    float finger_width = 0.02;
    float finger_length = 0.1;
    float finger_height = 0.02;
    float finger_spacing = 0.15;

    float crossbar_height = finger_height;
    float crossbar_thickness = finger_width;
    float crossbar_width = finger_spacing + finger_width * 2;

    // Local centers (in gripper's own frame)

    // Wrist
    Eigen::Vector3f wrist_center(0.0f, 0.0f, -0.035f);

    // Left finger
    Eigen::Vector3f left_finger_center(
        -finger_spacing/2.0f - finger_width/2.0f,
        0.0f,
        finger_length/2.0f
    );

    // Right finger
    Eigen::Vector3f right_finger_center(
        +finger_spacing/2.0f + finger_width/2.0f,
        0.0f,
        finger_length/2.0f
    );

    // Crossbar connecting the two fingers at base
    Eigen::Vector3f crossbar_center(
        0.0f,
        0.0f,
        crossbar_height/2.0f
    );

    // Draw wrist
    addOrientedCube(
        viewer,
        wrist_center,
        gripper_rotation,
        gripper_translation,
        wrist_size,
        wrist_size,
        5*wrist_size,
        0.0, 0.0, 1.0,
        gripper_id_prefix + "_wrist"
    );

    // Draw left finger
    addOrientedCube(
        viewer,
        left_finger_center,
        gripper_rotation,
        gripper_translation,
        finger_width,
        finger_height,
        finger_length,
        1.0, 0.0, 0.0,
        gripper_id_prefix + "_left_finger"
    );

    // Draw right finger
    addOrientedCube(
        viewer,
        right_finger_center,
        gripper_rotation,
        gripper_translation,
        finger_width,
        finger_height,
        finger_length,
        0.0, 1.0, 0.0,
        gripper_id_prefix + "_right_finger"
    );

    // Draw crossbar
    addOrientedCube(
        viewer,
        crossbar_center,
        gripper_rotation,
        gripper_translation,
        crossbar_width,
        crossbar_thickness,
        crossbar_height,
        1.0, 1.0, 0.0,
        gripper_id_prefix + "_crossbar"
    );
}