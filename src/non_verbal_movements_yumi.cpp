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

#include "tf2_ros/message_filter.h"
#include "message_filters/subscriber.h"

#include <iostream>
#include <string>

#include <memory>

#include "planner/TransformListener.h"

using namespace std;

const double tau = 2 * M_PI;

class HRI_Interface
{
public:
    HRI_Interface(ros::NodeHandle n, std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface,
                  std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface,
                  const moveit::core::JointModelGroup *joint_model_group,
                  std::shared_ptr<moveit_visual_tools::MoveItVisualTools> visual_tools)
        : n_(n), move_group_interface_(move_group_interface), planning_scene_interface_(planning_scene_interface), joint_model_group_(joint_model_group), visual_tools_(visual_tools)
    {
        robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
        kinematic_model_ = robot_model_loader.getModel();

        transform_listener = n_.serviceClient<planner::TransformListener>("transform_listener_srv");
    }

    void exagerateTrajectory(moveit::planning_interface::MoveGroupInterface::Plan plan, std::string planning_group, Eigen::Vector3d xyz)
    {

        // REFERENCE: https://github.com/ros-planning/moveit/issues/1556

        robot_state::RobotStatePtr kinematic_state(move_group_interface_->getCurrentState());

        int num_waypoints = plan.trajectory_.joint_trajectory.points.size(); // gets the number of waypoints in the trajectory
        int mid_waypoint = num_waypoints / 2;

        const std::vector<std::string> joint_names = plan.trajectory_.joint_trajectory.joint_names; // gets the names of the joints being updated in the trajectory
        Eigen::Affine3d current_end_effector_state;
        std::vector<double> previous_joint_values = plan.trajectory_.joint_trajectory.points.at(0).positions;

        double scale = 0;
        double previous_max_scale = 0;

        Eigen::Vector3d xyz_max = xyz;

        Eigen::Isometry3d target_pose = Eigen::Isometry3d::Identity();

        bool success_IK = true;

        // YUMI_SPECIFIC
        //  std::string gripper = planning_group == "right_arm" ? "gripper_r_base" : "gripper_l_base";
        std::string gripper = std::string("gripper_") + planning_group[0] + std::string("_base");
        ROS_INFO("Gripper: %s", gripper.c_str());
        // FIRST LOOPS TO SET THE MAXIMUM RANGE OF THE ARM
        {
            previous_max_scale = 0;

            for (int i = 0; i < num_waypoints; i++)
            {
                // set joint positions of waypoint
                kinematic_state->setVariablePositions(joint_names, plan.trajectory_.joint_trajectory.points.at(i).positions);
                current_end_effector_state = kinematic_state->getGlobalLinkTransform(gripper);

                // FIRST LOOP TO FIND THE MAX TRANSLATION

                target_pose = Eigen::Isometry3d::Identity();
                target_pose.translation() = current_end_effector_state.translation();

                if (i < mid_waypoint)
                {
                    scale = i / static_cast<double>(mid_waypoint);
                }
                else if (i > mid_waypoint)
                {
                    scale = 1 - (i - mid_waypoint) / static_cast<double>(mid_waypoint);
                }
                else
                {
                    scale = 1;
                }

                ROS_INFO_STREAM(scale);

                target_pose.translate(xyz_max * scale);
                target_pose.linear() = current_end_effector_state.rotation();

                visual_tools_->publishAxisLabeled(target_pose, "Pose");
                visual_tools_->trigger();

                kinematic_state->setJointGroupPositions(kinematic_model_->getJointModelGroup(planning_group), previous_joint_values);
                success_IK = kinematic_state->setFromIK(kinematic_model_->getJointModelGroup(planning_group), target_pose);

                std::stringstream tmp;
                tmp << i;
                char const *iteration = tmp.str().c_str();
                ROS_INFO_NAMED("First Loop", "Iteration %s", success_IK ? iteration : "FAILED");

                visual_tools_->prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

                if (!success_IK)
                {
                    xyz_max = xyz_max * previous_max_scale;
                    break;
                }

                previous_max_scale = i <= mid_waypoint ? scale : previous_max_scale;
            }
        }
        while (!success_IK)
            ;

        visual_tools_->deleteAllMarkers();

        for (int i = 0; i < num_waypoints; i++)
        {
            // set joint positions of waypoint
            kinematic_state->setVariablePositions(joint_names, plan.trajectory_.joint_trajectory.points.at(i).positions);
            current_end_effector_state = kinematic_state->getGlobalLinkTransform(gripper);

            bool success_IK = true;
            // FIRST LOOP TO FIND THE MAX TRANSLATION

            target_pose = Eigen::Isometry3d::Identity();
            target_pose.translation() = current_end_effector_state.translation();

            if (i < mid_waypoint)
            {
                scale = i / static_cast<double>(mid_waypoint);
            }
            else if (i > mid_waypoint)
            {
                scale = 1 - (i - mid_waypoint) / static_cast<double>(mid_waypoint);
            }
            else
            {
                scale = 1;
            }

            ROS_INFO_STREAM(scale);

            target_pose.translate(xyz_max * scale);
            target_pose.linear() = current_end_effector_state.rotation();

            visual_tools_->publishAxisLabeled(target_pose, "Pose");
            visual_tools_->trigger();

            kinematic_state->setJointGroupPositions(kinematic_model_->getJointModelGroup(planning_group), previous_joint_values);
            success_IK = kinematic_state->setFromIK(kinematic_model_->getJointModelGroup(planning_group), target_pose);

            std::stringstream tmp;
            tmp << i;
            char const *iteration = tmp.str().c_str();
            ROS_INFO_NAMED("tutorial", "Iteration %s", success_IK ? iteration : "FAILED");

            visual_tools_->prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

            kinematic_state->copyJointGroupPositions(kinematic_model_->getJointModelGroup(planning_group), plan.trajectory_.joint_trajectory.points.at(i).positions);

            previous_joint_values = plan.trajectory_.joint_trajectory.points.at(i).positions;
        }

        move_group_interface_->execute(plan);
    }

    void yes_no(std::string target_frame)
    {

        move_group_interface_->setPlannerId("stomp");
        // Set constraints for arm pose
        // These constraints are meant to set the arm in a V shape (approximately)
        moveit_msgs::Constraints path_constraints;
        // Add joint constraints
        moveit_msgs::JointConstraint joint_constraint;

        joint_constraint.joint_name = move_group_interface_->getActiveJoints().at(0); // "yumi_joint_1_r"
        joint_constraint.position = 125 * (M_PI / 180);
        joint_constraint.tolerance_above = 0.2;
        joint_constraint.tolerance_below = 0.2;
        joint_constraint.weight = 1.0;
        path_constraints.joint_constraints.push_back(joint_constraint);

        joint_constraint.joint_name = move_group_interface_->getActiveJoints().at(1); // "yumi_joint_2_r"
        joint_constraint.position = -115 * (M_PI / 180);
        joint_constraint.tolerance_above = 0.1;
        joint_constraint.tolerance_below = 0.1;
        joint_constraint.weight = 1.0;
        path_constraints.joint_constraints.push_back(joint_constraint);

        joint_constraint.joint_name = move_group_interface_->getActiveJoints().at(3); // "yumi_joint_3_r"
        joint_constraint.position = 45 * (M_PI / 180);
        joint_constraint.tolerance_above = 0.2;
        joint_constraint.tolerance_below = 0.2;
        joint_constraint.weight = 1.0;
        path_constraints.joint_constraints.push_back(joint_constraint);
        // Set the path constraints for the goal
        move_group_interface_->setPathConstraints(path_constraints);
        move_group_interface_->setGoalPositionTolerance(0.02);

        geometry_msgs::TransformStamped targetTransform;
        geometry_msgs::TransformStamped linkTransform;

        planner::TransformListener transformListenerMsg;

        transformListenerMsg.request.target_frame = "yumi_base_link";
        transformListenerMsg.request.source_frame = target_frame;
        transform_listener.call(transformListenerMsg);
        targetTransform = transformListenerMsg.response.transformStamped;

        transformListenerMsg.request.target_frame = "yumi_base_link";
        transformListenerMsg.request.source_frame = move_group_interface_->getLinkNames().at(1); //"yumi_link_2_r"
        transform_listener.call(transformListenerMsg);
        linkTransform = transformListenerMsg.response.transformStamped;

        double xTarget = targetTransform.transform.translation.x - linkTransform.transform.translation.x;
        double yTarget = targetTransform.transform.translation.y - linkTransform.transform.translation.y;

        double dist_ratio = 0.3 / sqrt(pow(xTarget, 2) + pow(yTarget, 2));
        double angle = atan2(yTarget, xTarget);

        // The eef's final pose will always be upright and about 0.4 units high.

        geometry_msgs::Pose final_pose;
        final_pose.position.z = 0.40;

        final_pose.position.x = linkTransform.transform.translation.x + xTarget * dist_ratio;
        final_pose.position.y = linkTransform.transform.translation.y + yTarget * dist_ratio;

        tf2::Quaternion q(tf2::Vector3(0, 0, 1), angle);

        geometry_msgs::Quaternion q_final_pose_msg;

        tf2::convert(q, q_final_pose_msg);
        final_pose.orientation = q_final_pose_msg;

        visual_tools_->publishAxis(final_pose);
        visual_tools_->trigger();

        move_group_interface_->setPoseTarget(final_pose);

        moveit::planning_interface::MoveGroupInterface::Plan plan;

        bool success = (move_group_interface_->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

        move_group_interface_->move();
        move_group_interface_->clearPathConstraints();
    }

    // bool mode (true = screw, false = unscrew)
    void screw_unscrew(bool mode, geometry_msgs::Pose input_pose)
    {
        double distance = 0.05;

        double maxBound = kinematic_model_->getVariableBounds(move_group_interface_->getActiveJoints().at(6)).max_position_;
        double minBound = kinematic_model_->getVariableBounds(move_group_interface_->getActiveJoints().at(6)).min_position_;

        Eigen::Isometry3d unscrew_inital_eigen;
        visual_tools_->convertPoseSafe(input_pose, unscrew_inital_eigen);
        unscrew_inital_eigen.translate(Eigen::Vector3d(0, 0, -distance));
        geometry_msgs::Pose unscrew_inital_pose = visual_tools_->convertPose(unscrew_inital_eigen);

        // Set Initial Position
        geometry_msgs::Pose pose = mode ? unscrew_inital_pose : input_pose;

        std::vector<double> joint_group_positions;
        moveit::core::RobotState start_state(*move_group_interface_->getCurrentState());
        start_state.copyJointGroupPositions(joint_model_group_, joint_group_positions);

        joint_group_positions.back() = mode ? minBound : maxBound;
        move_group_interface_->setJointValueTarget(joint_group_positions);

        moveit::planning_interface::MoveGroupInterface::Plan plan_Set_Initial_Position;

        bool success = (move_group_interface_->plan(plan_Set_Initial_Position) == moveit::core::MoveItErrorCode::SUCCESS);
        move_group_interface_->move();

        // Perform action of EndEffector
        geometry_msgs::Pose start_end_effector_pose = move_group_interface_->getCurrentPose().pose;
        Eigen::Isometry3d goal_end_effector_eigen;

        double turn_angle = mode ? M_PI_2 : -M_PI_2;
        int distance_ratio = 6;
        Eigen::Vector3d translation(0, 0, mode ? distance / distance_ratio : -distance / distance_ratio);

        double fraction;
        moveit_msgs::RobotTrajectory trajectory;

        do
        {

            geometry_msgs::Pose goal_end_effector_pose = start_end_effector_pose;
            std::vector<geometry_msgs::Pose> waypoints;

            for (double angle = 0; angle <= 458 * (M_PI / 180); angle += abs(turn_angle))
            {
                // Translation
                visual_tools_->convertPoseSafe(goal_end_effector_pose, goal_end_effector_eigen);
                goal_end_effector_eigen.translate(translation);
                goal_end_effector_pose = visual_tools_->convertPose(goal_end_effector_eigen);

                // Rotation
                tf2::Quaternion q_orig, q_rot, q_new;
                tf2::convert(goal_end_effector_pose.orientation, q_orig);
                q_rot.setRPY(0, 0, turn_angle);
                q_new = q_orig * q_rot;
                geometry_msgs::Quaternion rotated_quat;
                tf2::convert(q_new, rotated_quat);
                goal_end_effector_pose.orientation = rotated_quat;

                waypoints.push_back(goal_end_effector_pose);
            }

            const double jump_threshold = 0.0;
            const double eef_step = 0.1;
            fraction = move_group_interface_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

            visual_tools_->deleteAllMarkers();

            for (std::size_t i = 0; i < waypoints.size(); ++i)
                visual_tools_->publishAxis(waypoints[i]);
            visual_tools_->trigger();

            if (fraction < 0.8)
            {
                translation.z() = translation.z() * 0.95;
                ROS_INFO_NAMED("tutorial", "Cartesian path FAILED (%.2f%% achieved)", fraction * 100.0);
            }
            else
            {
                ROS_INFO_NAMED("tutorial", "Visualizing Cartesian path (%.2f%% achieved)", fraction * 100.0);
            }
        } while (fraction < 0.8);
        move_group_interface_->execute(trajectory);
    }

    void pointTo(std::string object_id)
    {

        std::map<std::string, moveit_msgs::CollisionObject> objects = planning_scene_interface_->getObjects();

        visual_tools_->publishAxis(objects[object_id].pose);
        visual_tools_->trigger();
    }

private:
    ros::NodeHandle n_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
    std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;
    const moveit::core::JointModelGroup *joint_model_group_;
    std::shared_ptr<moveit_visual_tools::MoveItVisualTools> visual_tools_;

    robot_model::RobotModelPtr kinematic_model_;
    ros::ServiceClient transform_listener;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hri_interface");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    static const std::string PLANNING_GROUP = "right_arm";
    auto move_group_interface = std::make_shared<moveit::planning_interface::MoveGroupInterface>(PLANNING_GROUP);
    // move_group_interface->setPlannerId("stomp");
    move_group_interface->setPlanningTime(10.0);
    move_group_interface->setMaxVelocityScalingFactor(1);

    auto planning_scene_interface = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();
    const moveit::core::JointModelGroup *joint_model_group =
        move_group_interface->getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    namespace rvt = rviz_visual_tools;
    auto visual_tools = std::make_shared<moveit_visual_tools::MoveItVisualTools>("yumi_base_link");
    visual_tools->deleteAllMarkers();
    visual_tools->loadRemoteControl();

    // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().z() = 0.75;
    visual_tools->publishText(text_pose, "NON VERBAL MOVEMENT TESTING", rvt::WHITE, rvt::XXLARGE);
    visual_tools->trigger();

    HRI_Interface hri_interface(node_handle, move_group_interface, planning_scene_interface, joint_model_group, visual_tools);

    move_group_interface->setNamedTarget("home");
    move_group_interface->move();

    geometry_msgs::Pose pose = move_group_interface->getCurrentPose().pose;

    // for (const std::string &link_name : move_group_interface->getLinkNames())
    // {
    //     std::cout << "Link name: " << link_name << std::endl;
    // }

    // for (const std::string &joint_name : move_group_interface->getActiveJoints())
    // {
    //     std::cout << "Joint name: " << joint_name << std::endl;
    // }

    int choice;
    bool mode = true;
    string input;
    do
    {
        cout << "MENU:\n"
             << "1. GO TO TARGET\n"
             << "2. EXECUTE SCREW UNSCREW\n"
             << "3. CHANGE MODE SCREW UNSCREW\n"
             << "4. POINT TO\n"
             << "5. EXIT\n"
             << "Enter your choice: ";
        cin >> choice;

        switch (choice)
        {
        case 1:
            cout << "You chose to GO TO TARGET.\n";
            hri_interface.yes_no("my_marker");
            break;
        case 2:
            hri_interface.screw_unscrew(mode, move_group_interface->getCurrentPose().pose);
            break;
        case 3:
            cout << "Enter 0 (unscrew) or 1 (screw): ";
            cin >> input;
            if (stoi(input) == 0)
            {
                mode = false;
            }
            else if (stoi(input) == 1)
            {
                mode = true;
            }
            else
            {
                cout << "Invalid input. Mode remains unchanged.\n";
            }
            break;
        case 4:
            cout << "Enter \"object id\" to point to:";
            cin >> input;
            hri_interface.pointTo(input);
            break;
        case 5:
            cout << "Exiting the program...\n";
            break;
        default:
            cout << "Invalid choice. Please try again.\n";
            break;
        }
    } while (choice != 5);

    ros::shutdown();
    return 0;
}