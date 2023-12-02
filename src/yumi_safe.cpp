#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_interface/planning_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <visualization_msgs/Marker.h>
#include <interactive_markers/interactive_marker_server.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/impl/utils.h>

#include <tf2_eigen/tf2_eigen.h>

#include "tf2_ros/message_filter.h"
#include "message_filters/subscriber.h"

#include <moveit/collision_detection_bullet/collision_env_bullet.h>
#include <moveit/collision_detection_bullet/collision_detector_allocator_bullet.h>
#include <moveit/collision_detection/collision_tools.h>

#include <iostream>
#include <fstream>
#include <string>

#include <memory>

#include <hri_interface/hri_interface.h>

using namespace std;

const double tau = 2 * M_PI;

void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface &planning_scene_interface)
{
    // BEGIN_SUB_TUTORIAL table1
    //
    // Creating Environment
    // ^^^^^^^^^^^^^^^^^^^^
    // Create vector to hold 3 collision objects.
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.resize(3);

    // Add the first table where the cube will originally be kept.
    collision_objects[0].id = "wall1";
    collision_objects[0].header.frame_id = "world";

    /* Define the primitive and its dimensions. */
    collision_objects[0].primitives.resize(1);
    collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
    collision_objects[0].primitives[0].dimensions.resize(3);
    collision_objects[0].primitives[0].dimensions[0] = 0.01;
    collision_objects[0].primitives[0].dimensions[1] = 2;
    collision_objects[0].primitives[0].dimensions[2] = 2;

    /* Define the pose of the table. */
    collision_objects[0].primitive_poses.resize(1);
    collision_objects[0].primitive_poses[0].position.x = -0.44;
    collision_objects[0].primitive_poses[0].position.y = 0;
    collision_objects[0].primitive_poses[0].position.z = 0;
    collision_objects[0].primitive_poses[0].orientation.w = 1.0;
    // END_SUB_TUTORIAL

    collision_objects[0].operation = collision_objects[0].ADD;

    // BEGIN_SUB_TUTORIAL table2
    // Add the second table where we will be placing the cube.
    collision_objects[1].id = "wall2";
    collision_objects[1].header.frame_id = "world";

    /* Define the primitive and its dimensions. */
    collision_objects[1].primitives.resize(1);
    collision_objects[1].primitives[0].type = collision_objects[1].primitives[0].BOX;
    collision_objects[1].primitives[0].dimensions.resize(3);
    collision_objects[1].primitives[0].dimensions[0] = 2;
    collision_objects[1].primitives[0].dimensions[1] = 0.01;
    collision_objects[1].primitives[0].dimensions[2] = 2;

    /* Define the pose of the table. */
    collision_objects[1].primitive_poses.resize(1);
    collision_objects[1].primitive_poses[0].position.x = 0;
    collision_objects[1].primitive_poses[0].position.y = -1.0;
    collision_objects[1].primitive_poses[0].position.z = 0.0;
    collision_objects[1].primitive_poses[0].orientation.w = 1.0;
    // END_SUB_TUTORIAL

    collision_objects[1].operation = collision_objects[1].ADD;

    // BEGIN_SUB_TUTORIAL table2
    // Add the second table where we will be placing the cube.
    collision_objects[2].id = "floor";
    collision_objects[2].header.frame_id = "world";

    /* Define the primitive and its dimensions. */
    collision_objects[2].primitives.resize(1);
    collision_objects[2].primitives[0].type = collision_objects[2].primitives[2].BOX;
    collision_objects[2].primitives[0].dimensions.resize(3);
    collision_objects[2].primitives[0].dimensions[0] = 2;
    collision_objects[2].primitives[0].dimensions[1] = 2;
    collision_objects[2].primitives[0].dimensions[2] = 0.01;

    /* Define the pose of the table. */
    collision_objects[2].primitive_poses.resize(1);
    collision_objects[2].primitive_poses[0].position.x = 0;
    collision_objects[2].primitive_poses[0].position.y = 0;
    collision_objects[2].primitive_poses[0].position.z = -0.12;
    collision_objects[2].primitive_poses[0].orientation.w = 1.0;
    // END_SUB_TUTORIAL

    collision_objects[2].operation = collision_objects[2].ADD;

    planning_scene_interface.applyCollisionObjects(collision_objects);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hri_interface");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    auto planning_scene_interface = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();

    auto both_arms_mgi = std::make_shared<moveit::planning_interface::MoveGroupInterface>("both_arms");
    const moveit::core::JointModelGroup *both_arms_jmg = both_arms_mgi->getCurrentState()->getJointModelGroup("both_arms");

    auto right_arm_mgi = std::make_shared<moveit::planning_interface::MoveGroupInterface>("right_arm");
    const moveit::core::JointModelGroup *right_arm_jmg = right_arm_mgi->getCurrentState()->getJointModelGroup("right_arm");

    auto left_arm_mgi = std::make_shared<moveit::planning_interface::MoveGroupInterface>("left_arm");
    const moveit::core::JointModelGroup *left_arm_jmg = left_arm_mgi->getCurrentState()->getJointModelGroup("left_arm");

    auto right_gripper_mgi = std::make_shared<moveit::planning_interface::MoveGroupInterface>("right_gripper");
    const moveit::core::JointModelGroup *right_gripper_jmg = right_gripper_mgi->getCurrentState()->getJointModelGroup("right_gripper");

    auto left_gripper_mgi = std::make_shared<moveit::planning_interface::MoveGroupInterface>("left_gripper");
    const moveit::core::JointModelGroup *left_gripper_jmg = left_gripper_mgi->getCurrentState()->getJointModelGroup("left_gripper");

    right_arm_mgi->setMaxVelocityScalingFactor(1.0);
    right_arm_mgi->setMaxAccelerationScalingFactor(1.0);
    right_arm_mgi->setPlanningTime(5);
    right_arm_mgi->allowReplanning(true);
    right_gripper_mgi->setMaxVelocityScalingFactor(1.0);
    right_gripper_mgi->setMaxAccelerationScalingFactor(1.0);
    right_gripper_mgi->setPlanningTime(5);
    right_gripper_mgi->allowReplanning(true);

    left_arm_mgi->setMaxVelocityScalingFactor(1.0);
    left_arm_mgi->setMaxAccelerationScalingFactor(1.0);
    left_arm_mgi->setPlanningTime(5);
    left_arm_mgi->allowReplanning(true);
    left_gripper_mgi->setMaxVelocityScalingFactor(1.0);
    left_gripper_mgi->setMaxAccelerationScalingFactor(1.0);
    left_gripper_mgi->setPlanningTime(5);
    left_gripper_mgi->allowReplanning(true);

    namespace rvt = rviz_visual_tools;
    auto visual_tools = std::make_shared<moveit_visual_tools::MoveItVisualTools>("yumi_base_link");
    visual_tools->deleteAllMarkers();
    visual_tools->loadRemoteControl();
    visual_tools->setGlobalScale(0.4);

    // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().z() = 0.75;
    visual_tools->publishText(text_pose, "NON VERBAL MOVEMENT TESTING", rvt::WHITE, rvt::XXLARGE);
    visual_tools->trigger();

    addCollisionObjects(*planning_scene_interface);

    // HRI_Interface right_arm_hri(node_handle, right_arm_mgi, right_arm_jmg, right_gripper_mgi, right_gripper_jmg, planning_scene_interface, visual_tools);
    // HRI_Interface left_arm_hri(node_handle, left_arm_mgi, left_arm_jmg, left_gripper_mgi, left_gripper_jmg, planning_scene_interface, visual_tools);

    int choice;
    string input;
    do
    {
        cout << "MENU:\n"
             << "0. EXIT\n"
             << "Enter your choice: ";
        cin >> choice;

        switch (choice)
        {
        case 0:
            cout << "Exiting the program...\n";
            break;
        default:
            cout << "Invalid choice. Please try again.\n";
            break;
        }
    } while (choice != 0);

    ros::shutdown();
    return 0;
}