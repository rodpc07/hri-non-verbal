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

#include <iostream>
#include <string>

using namespace std;

const double tau = 2 * M_PI;

void nonverbal(moveit::planning_interface::MoveGroupInterface &move_group_interface, moveit_visual_tools::MoveItVisualTools &visual_tools, Eigen::Isometry3d &text_pose, moveit::core::JointModelGroup joint_model_group, geometry_msgs::Pose pose)
{

    pose.position.z = 0.25;
    pose.orientation.x = 0;
    pose.orientation.y = 0;
    pose.orientation.z = 0;
    pose.orientation.w = 1;

    moveit_msgs::Constraints path_constraints;

    // Add joint constraints
    moveit_msgs::JointConstraint joint_constraint;

    joint_constraint.joint_name = "yumi_joint_1_r";
    joint_constraint.position = 125 * (M_PI / 180);
    joint_constraint.tolerance_above = 0.2;
    joint_constraint.tolerance_below = 0.2;
    path_constraints.joint_constraints.push_back(joint_constraint);

    joint_constraint.joint_name = "yumi_joint_2_r";
    joint_constraint.position = -115 * (M_PI / 180);
    joint_constraint.tolerance_above = 0.1;
    joint_constraint.tolerance_below = 0.1;
    path_constraints.joint_constraints.push_back(joint_constraint);

    joint_constraint.joint_name = "yumi_joint_3_r";
    joint_constraint.position = 25 * (M_PI / 180);
    joint_constraint.tolerance_above = 0.2;
    joint_constraint.tolerance_below = 0.2;
    path_constraints.joint_constraints.push_back(joint_constraint);

    // Set the path constraints for the goal
    move_group_interface.setPathConstraints(path_constraints);

    move_group_interface.setPoseTarget(pose);

    // FALTA DAR SETUP PARA APONTAR PARA O TARGET

    move_group_interface.setGoalPositionTolerance(0.2);

    moveit::planning_interface::MoveGroupInterface::Plan plan;

    bool success = (move_group_interface.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    // visual_tools.deleteAllMarkers();
    // visual_tools.publishTrajectoryPath(plan.trajectory_, plan.start_state_, true);
    // visual_tools.trigger();

    move_group_interface.move();
    move_group_interface.clearPathConstraints();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "exagerate_trajectory");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    static const std::string PLANNING_GROUP = "right_arm";
    moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);
    move_group_interface.setPlanningTime(45.0);

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    const moveit::core::JointModelGroup *joint_model_group =
        move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("yumi_base_link");
    visual_tools.deleteAllMarkers();
    visual_tools.loadRemoteControl();

    // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().z() = 0.75;
    visual_tools.publishText(text_pose, "NON VERBAL MOVEMENT TESTING", rvt::WHITE, rvt::XXLARGE);
    visual_tools.trigger();

    move_group_interface.setNamedTarget("home");
    move_group_interface.move();

    geometry_msgs::Pose pose;

    pose = move_group_interface.getCurrentPose().pose;

    int choice;
    string str1, str2;
    do
    {
        cout << "MENU:\n"
             << "1. GO TO TARGET\n"
             << "2. CHOOSE TARGET\n"
             << "3. EXIT\n"
             << "Enter your choice: ";
        cin >> choice;

        switch (choice)
        {
        case 1:
            cout << "You chose to GO TO TARGET.\n";
            nonverbal(move_group_interface, visual_tools, text_pose, *joint_model_group, pose);
            break;
        case 2:
            cout << "ENTER NEW POSE VALUES\n";
            cout << "X Y:";
            cin >> str1 >> str2;
            if (str1 != "_")
                pose.position.x = stod(str1);
            if (str2 != "_")
                pose.position.y = stod(str2);
            break;
        case 3:
            cout << "Exiting the program...\n";
            break;
        default:
            cout << "Invalid choice. Please try again.\n";
            break;
        }
    } while (choice != 3);

    ros::shutdown();
    return 0;
}