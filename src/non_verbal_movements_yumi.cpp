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
#include <moveit_msgs/GetPositionIK.h>

using namespace std;

const double tau = 2 * M_PI;

class HRI_Interface
{
public:
    HRI_Interface(ros::NodeHandle n,
                  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_mgi,
                  const moveit::core::JointModelGroup *arm_jmg,
                  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> gripper_mgi,
                  const moveit::core::JointModelGroup *gripper_jmg,
                  std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface,
                  std::shared_ptr<moveit_visual_tools::MoveItVisualTools> visual_tools)
        : n_(n), arm_mgi_(arm_mgi), arm_jmg_(arm_jmg), gripper_mgi_(gripper_mgi), gripper_jmg_(gripper_jmg), planning_scene_interface_(planning_scene_interface), visual_tools_(visual_tools)
    {
        ROS_INFO("Initializing HRI_INTERFACE");
        // Subscribe to Transform Service
        transform_listener = n_.serviceClient<planner::TransformListener>("transform_listener_srv");
        ikService = n_.serviceClient<moveit_msgs::GetPositionIK>("/compute_ik");

        prePlanPick();
    }

    void exagerateTrajectory(moveit::planning_interface::MoveGroupInterface::Plan plan, Eigen::Vector3d xyz)
    {
        // REFERENCE: https://github.com/ros-planning/moveit/issues/1556

        robot_state::RobotStatePtr kinematic_state(arm_mgi_->getCurrentState());

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

        std::string gripper = gripper_mgi_->getLinkNames().at(0);
        ROS_INFO("Gripper: %s", gripper.c_str());
        // FIRST LOOPS TO SET THE MAXIMUM RANGE OF THE ARM
        do
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

                kinematic_state->setJointGroupPositions(arm_jmg_, previous_joint_values);
                success_IK = kinematic_state->setFromIK(arm_jmg_, target_pose);

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
        } while (!success_IK);

        visual_tools_->deleteAllMarkers();

        for (int i = 0; i < num_waypoints; i++)
        {
            // set joint positions of waypoint
            kinematic_state->setVariablePositions(joint_names, plan.trajectory_.joint_trajectory.points.at(i).positions);
            current_end_effector_state = kinematic_state->getGlobalLinkTransform(gripper);

            bool success_IK = true;

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

            kinematic_state->setJointGroupPositions(arm_jmg_, previous_joint_values);
            success_IK = kinematic_state->setFromIK(arm_jmg_, target_pose);

            std::stringstream tmp;
            tmp << i;
            char const *iteration = tmp.str().c_str();
            ROS_INFO_NAMED("tutorial", "Iteration %s", success_IK ? iteration : "FAILED");

            visual_tools_->prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

            kinematic_state->copyJointGroupPositions(arm_jmg_, plan.trajectory_.joint_trajectory.points.at(i).positions);

            previous_joint_values = plan.trajectory_.joint_trajectory.points.at(i).positions;
        }

        arm_mgi_->execute(plan);
    }

    void attention(std::string target_frame)
    {
        // TODO - Finish Movement

        arm_mgi_->setStartStateToCurrentState();

        geometry_msgs::TransformStamped targetTransform;
        geometry_msgs::TransformStamped linkTransform;

        planner::TransformListener transformListenerMsg;

        // Get transform from human position in reference to base_link
        transformListenerMsg.request.target_frame = "yumi_base_link";
        transformListenerMsg.request.source_frame = target_frame;
        transform_listener.call(transformListenerMsg);
        targetTransform = transformListenerMsg.response.transformStamped;

        // Get transform from shoulder position in reference to base_link
        transformListenerMsg.request.target_frame = "yumi_base_link";
        transformListenerMsg.request.source_frame = arm_mgi_->getLinkNames().at(1); //"yumi_link_2"
        transform_listener.call(transformListenerMsg);
        linkTransform = transformListenerMsg.response.transformStamped;

        // This joint values set the pose of the arm in a V shape (depends on robot morphology)
        std::vector<double> joint_group_positions = {125 * (M_PI / 180), -115 * (M_PI / 180), 0, 60 * (M_PI / 180), 90 * (M_PI / 180), 0, 0};
        moveit::core::RobotState arm_state(*arm_mgi_->getCurrentState());
        arm_state.setJointGroupPositions(arm_jmg_, joint_group_positions);

        const Eigen::Isometry3d &end_effector_state = arm_state.getGlobalLinkTransform(gripper_mgi_->getLinkNames().at(0));

        // Next is done the calculations to obtain the angle for yumi_joint_2 needed to set the arm aligned with the human

        double xTarget = targetTransform.transform.translation.x - linkTransform.transform.translation.x;
        double yTarget = targetTransform.transform.translation.y - linkTransform.transform.translation.y;

        double xEE = end_effector_state.translation().x() - linkTransform.transform.translation.x;
        double yEE = end_effector_state.translation().y() - linkTransform.transform.translation.y;

        double target_angle = atan2(yTarget, xTarget);
        double ee_angle = atan2(yEE, xEE);

        double angle = ee_angle - target_angle;

        joint_group_positions.at(2) = angle;
        arm_mgi_->setJointValueTarget(joint_group_positions);

        moveit::planning_interface::MoveGroupInterface::Plan arm_plan_1;

        arm_mgi_->plan(arm_plan_1);
        arm_mgi_->execute(arm_plan_1);

        // Next are planned in advance 4 motions to simulate waving, later executed sequecially

        arm_state.setJointGroupPositions(arm_jmg_, arm_plan_1.trajectory_.joint_trajectory.points.back().positions);
        joint_group_positions.at(5) = -45 * (M_PI / 180);
        arm_mgi_->setStartState(arm_state);
        arm_mgi_->setJointValueTarget(joint_group_positions);
        moveit::planning_interface::MoveGroupInterface::Plan arm_plan_2;
        arm_mgi_->plan(arm_plan_2);

        arm_state.setJointGroupPositions(arm_jmg_, arm_plan_2.trajectory_.joint_trajectory.points.back().positions);
        joint_group_positions.at(5) = 45 * (M_PI / 180);
        arm_mgi_->setStartState(arm_state);
        arm_mgi_->setJointValueTarget(joint_group_positions);
        moveit::planning_interface::MoveGroupInterface::Plan arm_plan_3;
        arm_mgi_->plan(arm_plan_3);

        arm_state.setJointGroupPositions(arm_jmg_, arm_plan_3.trajectory_.joint_trajectory.points.back().positions);
        joint_group_positions.at(5) = -45 * (M_PI / 180);
        arm_mgi_->setStartState(arm_state);
        arm_mgi_->setJointValueTarget(joint_group_positions);
        moveit::planning_interface::MoveGroupInterface::Plan arm_plan_4;
        arm_mgi_->plan(arm_plan_4);

        arm_state.setJointGroupPositions(arm_jmg_, arm_plan_4.trajectory_.joint_trajectory.points.back().positions);
        joint_group_positions.at(5) = 0 * (M_PI / 180);
        arm_mgi_->setStartState(arm_state);
        arm_mgi_->setJointValueTarget(joint_group_positions);
        moveit::planning_interface::MoveGroupInterface::Plan arm_plan_5;
        arm_mgi_->plan(arm_plan_5);

        arm_mgi_->execute(arm_plan_2);
        arm_mgi_->execute(arm_plan_3);
        arm_mgi_->execute(arm_plan_4);
        arm_mgi_->execute(arm_plan_5);
    }

    // bool mode (true = screw, false = unscrew)
    void screw_unscrew(bool mode, geometry_msgs::Pose input_pose)
    {

        arm_mgi_->setStartStateToCurrentState();

        double distance = 0.05;

        double maxBound = arm_mgi_->getRobotModel()->getVariableBounds(arm_mgi_->getActiveJoints().at(6)).max_position_;
        double minBound = arm_mgi_->getRobotModel()->getVariableBounds(arm_mgi_->getActiveJoints().at(6)).min_position_;

        Eigen::Isometry3d unscrew_inital_eigen;
        visual_tools_->convertPoseSafe(input_pose, unscrew_inital_eigen);
        unscrew_inital_eigen.translate(Eigen::Vector3d(0, 0, -distance));
        geometry_msgs::Pose unscrew_inital_pose = visual_tools_->convertPose(unscrew_inital_eigen);

        // Set Initial Position to same as argument or offset
        geometry_msgs::Pose pose = mode ? unscrew_inital_pose : input_pose;

        std::vector<double> joint_group_positions;
        moveit::core::RobotState start_state(*arm_mgi_->getCurrentState());
        start_state.copyJointGroupPositions(arm_jmg_, joint_group_positions);

        // Set gripper joint to limit to allow a full range of rotation

        joint_group_positions.back() = mode ? minBound : maxBound;
        arm_mgi_->setJointValueTarget(joint_group_positions);

        moveit::planning_interface::MoveGroupInterface::Plan plan_Set_Initial_Position;

        bool success = (arm_mgi_->plan(plan_Set_Initial_Position) == moveit::core::MoveItErrorCode::SUCCESS);
        arm_mgi_->execute(plan_Set_Initial_Position);

        // Perform action of EndEffector
        geometry_msgs::Pose start_end_effector_pose = arm_mgi_->getCurrentPose().pose;
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
            fraction = arm_mgi_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

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
        arm_mgi_->execute(trajectory);
        visual_tools_->deleteAllMarkers();
    }

    void pointToObject(std::string object_id, string target_frame)
    {

        visual_tools_->deleteAllMarkers();

        // Obtain object from scene
        std::map<std::string, moveit_msgs::CollisionObject> objects = planning_scene_interface_->getObjects();

        // Find position of object
        moveit_msgs::CollisionObject object = objects[object_id];
        geometry_msgs::Pose object_pose = object.pose;

        double radius = sqrt(pow(object.primitives[0].dimensions[0], 2) + pow(object.primitives[0].dimensions[1], 2) + pow(object.primitives[0].dimensions[2], 2));
        visual_tools_->publishSphere(object_pose, rviz_visual_tools::colors::PINK, radius);
        visual_tools_->publishAxis(object.pose);
        visual_tools_->trigger();

        // Create a vector from shoulder to object to calculate pose

        geometry_msgs::TransformStamped linkTransform;

        planner::TransformListener transformListenerMsg;

        transformListenerMsg.request.target_frame = "yumi_base_link";
        transformListenerMsg.request.source_frame = arm_mgi_->getLinkNames().at(0); //"yumi_link_1"
        transform_listener.call(transformListenerMsg);
        linkTransform = transformListenerMsg.response.transformStamped;

        double xTarget = object_pose.position.x - linkTransform.transform.translation.x;
        double yTarget = object_pose.position.y - linkTransform.transform.translation.y;
        double zTarget = object_pose.position.z - linkTransform.transform.translation.z;

        double distance = sqrt(pow(xTarget, 2) + pow(yTarget, 2) + pow(zTarget, 2));

        double targetDistance = distance - radius - 0.15 >= 0.6 ? 0.6 : distance - radius - 0.15;

        double scalingFactor = (targetDistance) / distance;

        xTarget *= scalingFactor;
        yTarget *= scalingFactor;
        zTarget *= scalingFactor;

        double sideAngle = atan2(yTarget, xTarget);
        double tiltAngle = M_PI_2 - atan2(zTarget, sqrt(pow(xTarget, 2) + pow(yTarget, 2)));

        tf2::Quaternion q1(tf2::Vector3(0, 0, 1), sideAngle);
        tf2::Quaternion q2(tf2::Vector3(0, 1, 0), tiltAngle);
        tf2::Quaternion qresult = q1 * q2;
        qresult.normalize();

        geometry_msgs::Quaternion q_msg;
        tf2::convert(qresult, q_msg);

        geometry_msgs::Pose lookPose;
        lookPose.position.x = linkTransform.transform.translation.x + xTarget;
        lookPose.position.y = linkTransform.transform.translation.y + yTarget;
        lookPose.position.z = linkTransform.transform.translation.z + zTarget;
        lookPose.orientation = q_msg;

        visual_tools_->publishAxis(lookPose);
        visual_tools_->trigger();

        arm_mgi_->setStartStateToCurrentState();
        arm_mgi_->setPoseTarget(lookPose);
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        arm_mgi_->plan(plan);
    }

    void pointToHuman(string target_frame)
    {
        // TODO - Finish pointToHuman

        visual_tools_->deleteAllMarkers();

        // Obtain object from scene
        std::map<std::string, moveit_msgs::CollisionObject> objects = planning_scene_interface_->getObjects();

        // Create a vector from shoulder to object to calculate pose

        // Calculate New Pose Looking at Human
        geometry_msgs::TransformStamped targetTransform;
        geometry_msgs::TransformStamped linkTransform;

        planner::TransformListener transformListenerMsg;

        transformListenerMsg.request.target_frame = "yumi_base_link";
        transformListenerMsg.request.source_frame = target_frame;
        transform_listener.call(transformListenerMsg);
        targetTransform = transformListenerMsg.response.transformStamped;

        transformListenerMsg.request.target_frame = "yumi_base_link";
        transformListenerMsg.request.source_frame = arm_mgi_->getLinkNames().at(0); //"yumi_link_7"
        transform_listener.call(transformListenerMsg);
        linkTransform = transformListenerMsg.response.transformStamped;

        double xTarget = targetTransform.transform.translation.x - linkTransform.transform.translation.x;
        double yTarget = targetTransform.transform.translation.y - linkTransform.transform.translation.y;
        double zTarget = targetTransform.transform.translation.z - linkTransform.transform.translation.z;

        double distance = sqrt(pow(xTarget, 2) + pow(yTarget, 2) + pow(zTarget, 2));

        double targetDistance = distance - 0.5 >= 0.6 ? 0.6 : distance - 0.5;

        double scalingFactor = (targetDistance) / distance;

        xTarget *= scalingFactor;
        yTarget *= scalingFactor;
        zTarget *= scalingFactor;

        double sideAngle = atan2(yTarget, xTarget);
        double tiltAngle = M_PI_2 - atan2(zTarget, sqrt(pow(xTarget, 2) + pow(yTarget, 2)));

        tf2::Quaternion q1(tf2::Vector3(0, 0, 1), sideAngle);
        tf2::Quaternion q2(tf2::Vector3(0, 1, 0), tiltAngle);
        tf2::Quaternion qresult = q1 * q2;
        qresult.normalize();

        geometry_msgs::Quaternion q_msg;
        tf2::convert(qresult, q_msg);

        geometry_msgs::Pose lookPose;
        lookPose.position.x = linkTransform.transform.translation.x + xTarget;
        lookPose.position.y = linkTransform.transform.translation.y + yTarget;
        lookPose.position.z = linkTransform.transform.translation.z + zTarget;
        lookPose.orientation = q_msg;

        visual_tools_->publishAxis(lookPose);
        visual_tools_->trigger();

        arm_mgi_->setStartStateToCurrentState();
        arm_mgi_->setPoseTarget(lookPose);
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        arm_mgi_->plan(plan);
    }

    void signalPick()
    {
        std::vector<moveit::planning_interface::MoveGroupInterface::Plan> planList;
        gripper_mgi_->setStartStateToCurrentState();

        if (abs(gripper_mgi_->getCurrentJointValues().at(0) - openedJointValues.at(0)) > 0.001)
            planList.push_back(openPlan);

        planList.push_back(closePlan);
        planList.push_back(openPlan);
        planList.push_back(closePlan);
        planList.push_back(openPlan);

        for (const auto &planValue : planList)
        {
            gripper_mgi_->execute(planValue);
        }
    }

    void signalRotateLeft()
    {
        // TODO - change target_frame also in Right

        // TODO - Fix bug when instead of rotating the gripper ir rotates the whole arm

        visual_tools_->deleteAllMarkers();
        arm_mgi_->setStartStateToCurrentState();

        double bound = arm_mgi_->getRobotModel()->getVariableBounds(arm_mgi_->getActiveJoints().back()).min_position_;

        // Get Initial Joint Values
        std::vector<double> initial_joint_values;
        moveit::core::RobotState arm_state(*arm_mgi_->getCurrentState());
        arm_state.copyJointGroupPositions(arm_jmg_, initial_joint_values);

        // Calculate New Pose Looking at Human
        geometry_msgs::TransformStamped targetTransform;
        geometry_msgs::TransformStamped linkTransform;

        planner::TransformListener transformListenerMsg;

        transformListenerMsg.request.target_frame = "yumi_base_link";
        transformListenerMsg.request.source_frame = "my_marker";
        transform_listener.call(transformListenerMsg);
        targetTransform = transformListenerMsg.response.transformStamped;

        transformListenerMsg.request.target_frame = "yumi_base_link";
        transformListenerMsg.request.source_frame = arm_mgi_->getLinkNames().back(); //"yumi_link_7"
        transform_listener.call(transformListenerMsg);
        linkTransform = transformListenerMsg.response.transformStamped;

        double xTarget = targetTransform.transform.translation.x - linkTransform.transform.translation.x;
        double yTarget = targetTransform.transform.translation.y - linkTransform.transform.translation.y;
        double zTarget = targetTransform.transform.translation.z - linkTransform.transform.translation.z;

        double sideAngle = atan2(yTarget, xTarget);
        double tiltAngle = M_PI_2 - atan2(zTarget, sqrt(pow(xTarget, 2) + pow(yTarget, 2)));

        tf2::Quaternion q1(tf2::Vector3(0, 0, 1), sideAngle);
        tf2::Quaternion q2(tf2::Vector3(0, 1, 0), tiltAngle);
        tf2::Quaternion qresult = q1 * q2;
        qresult.normalize();

        geometry_msgs::Quaternion q_msg;
        tf2::convert(qresult, q_msg);

        geometry_msgs::Pose lookPose;
        lookPose.position.x = linkTransform.transform.translation.x;
        lookPose.position.y = linkTransform.transform.translation.y;
        lookPose.position.z = linkTransform.transform.translation.z;
        lookPose.orientation = q_msg;

        arm_mgi_->setStartStateToCurrentState();
        arm_mgi_->setGoalPositionTolerance(0.05);
        arm_mgi_->setGoalOrientationTolerance(0.05);
        arm_mgi_->setPoseTarget(lookPose);
        moveit::planning_interface::MoveGroupInterface::Plan first_movement;
        arm_mgi_->plan(first_movement) == moveit::planning_interface::MoveItErrorCode::SUCCESS;

        std::vector<double> lookPose_joint_values = first_movement.trajectory_.joint_trajectory.points.back().positions;
        arm_state.setJointGroupPositions(arm_jmg_, lookPose_joint_values);
        if (abs(bound - lookPose_joint_values.back()) < M_PI_2)
        {
            lookPose_joint_values.back() += M_PI;
        }

        arm_mgi_->setStartState(arm_state);
        arm_mgi_->setJointValueTarget(lookPose_joint_values);
        moveit::planning_interface::MoveGroupInterface::Plan second_movement;
        arm_mgi_->plan(second_movement) == moveit::planning_interface::MoveItErrorCode::SUCCESS;

        // Get Pre-Rotation Joint Values
        std::vector<double> pre_rotation_joint_values = second_movement.trajectory_.joint_trajectory.points.back().positions;

        // Change End-Effector Rotation
        std::vector<double> rotated_joint_values = pre_rotation_joint_values;
        rotated_joint_values.back() += -M_PI_2;

        // Set Rotated Position as JointValueTarget and Plan
        arm_state.setJointGroupPositions(arm_jmg_, pre_rotation_joint_values);
        arm_mgi_->setStartState(arm_state);
        arm_mgi_->setJointValueTarget(rotated_joint_values);
        moveit::planning_interface::MoveGroupInterface::Plan first_rotation;
        arm_mgi_->plan(first_rotation) == moveit::planning_interface::MoveItErrorCode::SUCCESS;

        // Set Pre-Rotation Position as JointValueTarget and Plan
        arm_state.setJointGroupPositions(arm_jmg_, rotated_joint_values);
        arm_mgi_->setStartState(arm_state);
        arm_mgi_->setJointValueTarget(pre_rotation_joint_values);
        moveit::planning_interface::MoveGroupInterface::Plan second_rotation;
        arm_mgi_->plan(second_rotation) == moveit::planning_interface::MoveItErrorCode::SUCCESS;

        // Set Initial Position as JointValueTarget and Plan
        arm_state.setJointGroupPositions(arm_jmg_, pre_rotation_joint_values);
        arm_mgi_->setStartState(arm_state);
        arm_mgi_->setJointValueTarget(initial_joint_values);
        moveit::planning_interface::MoveGroupInterface::Plan last_movement;
        arm_mgi_->plan(last_movement) == moveit::planning_interface::MoveItErrorCode::SUCCESS;

        arm_mgi_->execute(first_movement);
        arm_mgi_->execute(second_movement);
        arm_mgi_->execute(first_rotation);
        arm_mgi_->execute(second_rotation);
        arm_mgi_->execute(last_movement);
    }

    void signalRotateRight()
    {
        visual_tools_->deleteAllMarkers();
        arm_mgi_->setStartStateToCurrentState();

        double bound = arm_mgi_->getRobotModel()->getVariableBounds(arm_mgi_->getActiveJoints().back()).max_position_;

        // Get Initial Joint Values
        std::vector<double> initial_joint_values;
        moveit::core::RobotState arm_state(*arm_mgi_->getCurrentState());
        arm_state.copyJointGroupPositions(arm_jmg_, initial_joint_values);

        // Calculate New Pose Looking at Human
        geometry_msgs::TransformStamped targetTransform;
        geometry_msgs::TransformStamped linkTransform;

        planner::TransformListener transformListenerMsg;

        transformListenerMsg.request.target_frame = "yumi_base_link";
        transformListenerMsg.request.source_frame = "my_marker";
        transform_listener.call(transformListenerMsg);
        targetTransform = transformListenerMsg.response.transformStamped;

        transformListenerMsg.request.target_frame = "yumi_base_link";
        transformListenerMsg.request.source_frame = arm_mgi_->getLinkNames().back(); //"yumi_link_7"
        transform_listener.call(transformListenerMsg);
        linkTransform = transformListenerMsg.response.transformStamped;

        double xTarget = targetTransform.transform.translation.x - linkTransform.transform.translation.x;
        double yTarget = targetTransform.transform.translation.y - linkTransform.transform.translation.y;
        double zTarget = targetTransform.transform.translation.z - linkTransform.transform.translation.z;

        double sideAngle = atan2(yTarget, xTarget);
        double tiltAngle = M_PI_2 - atan2(zTarget, sqrt(pow(xTarget, 2) + pow(yTarget, 2)));

        tf2::Quaternion q1(tf2::Vector3(0, 0, 1), sideAngle);
        tf2::Quaternion q2(tf2::Vector3(0, 1, 0), tiltAngle);
        tf2::Quaternion qresult = q1 * q2;
        qresult.normalize();

        geometry_msgs::Quaternion q_msg;
        tf2::convert(qresult, q_msg);

        geometry_msgs::Pose lookPose;
        lookPose.position.x = linkTransform.transform.translation.x;
        lookPose.position.y = linkTransform.transform.translation.y;
        lookPose.position.z = linkTransform.transform.translation.z;
        lookPose.orientation = q_msg;

        arm_mgi_->setStartStateToCurrentState();
        arm_mgi_->setGoalPositionTolerance(0.05);
        arm_mgi_->setGoalOrientationTolerance(0.05);
        arm_mgi_->setPoseTarget(lookPose);
        moveit::planning_interface::MoveGroupInterface::Plan first_movement;
        arm_mgi_->plan(first_movement) == moveit::planning_interface::MoveItErrorCode::SUCCESS;

        std::vector<double> lookPose_joint_values = first_movement.trajectory_.joint_trajectory.points.back().positions;
        arm_state.setJointGroupPositions(arm_jmg_, lookPose_joint_values);
        if (abs(bound - lookPose_joint_values.back()) < M_PI_2)
        {
            lookPose_joint_values.back() -= M_PI;
        }

        arm_mgi_->setStartState(arm_state);
        arm_mgi_->setJointValueTarget(lookPose_joint_values);
        moveit::planning_interface::MoveGroupInterface::Plan second_movement;
        arm_mgi_->plan(second_movement) == moveit::planning_interface::MoveItErrorCode::SUCCESS;

        // Get Pre-Rotation Joint Values
        std::vector<double> pre_rotation_joint_values = second_movement.trajectory_.joint_trajectory.points.back().positions;

        // Change End-Effector Rotation
        std::vector<double> rotated_joint_values = pre_rotation_joint_values;
        rotated_joint_values.back() += M_PI_2;

        // Set Rotated Position as JointValueTarget and Plan
        arm_state.setJointGroupPositions(arm_jmg_, pre_rotation_joint_values);
        arm_mgi_->setStartState(arm_state);
        arm_mgi_->setJointValueTarget(rotated_joint_values);
        moveit::planning_interface::MoveGroupInterface::Plan first_rotation;
        arm_mgi_->plan(first_rotation) == moveit::planning_interface::MoveItErrorCode::SUCCESS;

        // Set Pre-Rotation Position as JointValueTarget and Plan
        arm_state.setJointGroupPositions(arm_jmg_, rotated_joint_values);
        arm_mgi_->setStartState(arm_state);
        arm_mgi_->setJointValueTarget(pre_rotation_joint_values);
        moveit::planning_interface::MoveGroupInterface::Plan second_rotation;
        arm_mgi_->plan(second_rotation) == moveit::planning_interface::MoveItErrorCode::SUCCESS;

        // Set Initial Position as JointValueTarget and Plan
        arm_state.setJointGroupPositions(arm_jmg_, pre_rotation_joint_values);
        arm_mgi_->setStartState(arm_state);
        arm_mgi_->setJointValueTarget(initial_joint_values);
        moveit::planning_interface::MoveGroupInterface::Plan last_movement;
        arm_mgi_->plan(last_movement) == moveit::planning_interface::MoveItErrorCode::SUCCESS;

        arm_mgi_->execute(first_movement);
        arm_mgi_->execute(second_movement);
        arm_mgi_->execute(first_rotation);
        arm_mgi_->execute(second_rotation);
        arm_mgi_->execute(last_movement);
    }

private:
    ros::NodeHandle n_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_mgi_;
    const moveit::core::JointModelGroup *arm_jmg_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> gripper_mgi_;
    const moveit::core::JointModelGroup *gripper_jmg_;
    std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;
    std::shared_ptr<moveit_visual_tools::MoveItVisualTools> visual_tools_;

    ros::ServiceClient transform_listener;
    ros::ServiceClient ikService;

    moveit::planning_interface::MoveGroupInterface::Plan openPlan;
    moveit::planning_interface::MoveGroupInterface::Plan closePlan;
    std::vector<double> closedJointValues;
    std::vector<double> openedJointValues;

    void prePlanPick()
    {
        // Pre-plan close and open gripper motions
        closedJointValues = {gripper_mgi_->getNamedTargetValues("close").begin()->second};
        openedJointValues = {gripper_mgi_->getNamedTargetValues("open").begin()->second};

        moveit::core::RobotState gripper_state(*gripper_mgi_->getCurrentState());

        // Plan Close Trajectory
        gripper_state.setJointGroupActivePositions(gripper_jmg_, openedJointValues);
        gripper_mgi_->setStartState(gripper_state);
        gripper_mgi_->setNamedTarget("close");
        gripper_mgi_->plan(closePlan);

        // Plan Open Trajectory
        gripper_state.setJointGroupActivePositions(gripper_jmg_, closedJointValues);
        gripper_mgi_->setStartState(gripper_state);
        gripper_mgi_->setNamedTarget("open");
        gripper_mgi_->plan(openPlan);
    }
};

void pick(moveit::planning_interface::MoveGroupInterface &arm_mgi, moveit::planning_interface::MoveGroupInterface &gripper_mgi)
{
    // BEGIN_SUB_TUTORIAL pick1
    // Create a vector of grasps to be attempted, currently only creating single grasp.
    // This is essentially useful when using a grasp generator to generate and test multiple grasps.
    std::vector<moveit_msgs::Grasp> grasps;
    grasps.resize(1);

    // Actual Grasping Pose
    grasps[0].grasp_pose.header.frame_id = "yumi_base_link";
    grasps[0].grasp_pose.pose.position.x = 0.383890;
    grasps[0].grasp_pose.pose.position.y = -0.008552;
    grasps[0].grasp_pose.pose.position.z = 0.491650;
    grasps[0].grasp_pose.pose.orientation.x = 0.471129;
    grasps[0].grasp_pose.pose.orientation.y = 0.538489;
    grasps[0].grasp_pose.pose.orientation.z = 0.500098;
    grasps[0].grasp_pose.pose.orientation.w = 0.487822;

    // Pre Grasp (vector representing the direction of approach)
    grasps[0].pre_grasp_approach.direction.header.frame_id = "yumi_base_link";
    grasps[0].pre_grasp_approach.direction.vector.x = 1.0;
    grasps[0].pre_grasp_approach.min_distance = 0.095;
    grasps[0].pre_grasp_approach.desired_distance = 0.115;

    // Post Grasp (vector representing the direction of retreat)
    grasps[0].post_grasp_retreat.direction.header.frame_id = "yumi_base_link";
    grasps[0].post_grasp_retreat.direction.vector.z = 1.0;
    grasps[0].post_grasp_retreat.min_distance = 0.1;
    grasps[0].post_grasp_retreat.desired_distance = 0.25;

    // Open Gripper Pose
    for (const std::string joint_name : gripper_mgi.getJointNames())
    {
        grasps[0].pre_grasp_posture.joint_names.push_back(joint_name);
    };

    grasps[0].pre_grasp_posture.points.resize(1);
    grasps[0].pre_grasp_posture.points[0].positions.resize(2);
    grasps[0].pre_grasp_posture.points[0].positions[0] = 0.025;
    grasps[0].pre_grasp_posture.points[0].positions[1] = 0.025;
    grasps[0].pre_grasp_posture.points[0].time_from_start = ros::Duration(0.5);

    // Close Gripper Pose
    for (const std::string joint_name : gripper_mgi.getJointNames())
    {
        grasps[0].grasp_posture.joint_names.push_back(joint_name);
    };

    grasps[0].grasp_posture.points.resize(1);
    grasps[0].grasp_posture.points[0].positions.resize(2);
    grasps[0].grasp_posture.points[0].positions[0] = 0.00;
    grasps[0].grasp_posture.points[0].positions[1] = 0.00;
    grasps[0].grasp_posture.points[0].time_from_start = ros::Duration(0.5);

    arm_mgi.setSupportSurfaceName("table1");
    arm_mgi.pick("object", grasps);
}

void place(moveit::planning_interface::MoveGroupInterface &arm_mgi, moveit::planning_interface::MoveGroupInterface &gripper_mgi)
{
    std::vector<moveit_msgs::PlaceLocation> place_location;
    place_location.resize(1);

    // Actual Placing Pose (this is the object pose and not the gripper pose)
    place_location[0].place_pose.header.frame_id = "yumi_base_link";
    place_location[0].place_pose.pose.position.x = 0.02;
    place_location[0].place_pose.pose.position.y = 0.46;
    place_location[0].place_pose.pose.position.z = 0.5;
    place_location[0].place_pose.pose.orientation.x = 0;
    place_location[0].place_pose.pose.orientation.y = 0;
    place_location[0].place_pose.pose.orientation.z = 0.720831;
    place_location[0].place_pose.pose.orientation.w = 0.693111;

    // Pre Place Pose (vector representing the direction of approach, simetric to the pick)
    place_location[0].pre_place_approach.direction.header.frame_id = "yumi_base_link";
    place_location[0].pre_place_approach.direction.vector.z = -1.0;
    place_location[0].pre_place_approach.min_distance = 0.095;
    place_location[0].pre_place_approach.desired_distance = 0.115;

    // Pre Place Pose (vector representing the direction of retreat)
    place_location[0].post_place_retreat.direction.header.frame_id = "yumi_base_link";
    place_location[0].post_place_retreat.direction.vector.z = 1.0;
    place_location[0].post_place_retreat.min_distance = 0.1;
    place_location[0].post_place_retreat.desired_distance = 0.25;

    // Open Gripper Pose
    for (const std::string joint_name : gripper_mgi.getJointNames())
    {
        place_location[0].post_place_posture.joint_names.push_back(joint_name);
    };

    place_location[0].post_place_posture.points.resize(1);
    place_location[0].post_place_posture.points[0].positions.resize(2);
    place_location[0].post_place_posture.points[0].positions[0] = 0.025;
    place_location[0].post_place_posture.points[0].positions[1] = 0.025;
    place_location[0].post_place_posture.points[0].time_from_start = ros::Duration(0.5);

    arm_mgi.setSupportSurfaceName("table2");
    arm_mgi.place("object", place_location);
}

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
    collision_objects[0].id = "table1";
    collision_objects[0].header.frame_id = "world";

    /* Define the primitive and its dimensions. */
    collision_objects[0].primitives.resize(1);
    collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
    collision_objects[0].primitives[0].dimensions.resize(3);
    collision_objects[0].primitives[0].dimensions[0] = 0.2;
    collision_objects[0].primitives[0].dimensions[1] = 0.4;
    collision_objects[0].primitives[0].dimensions[2] = 0.4;

    /* Define the pose of the table. */
    collision_objects[0].primitive_poses.resize(1);
    collision_objects[0].primitive_poses[0].position.x = 0.5;
    collision_objects[0].primitive_poses[0].position.y = 0;
    collision_objects[0].primitive_poses[0].position.z = 0.2;
    collision_objects[0].primitive_poses[0].orientation.w = 1.0;
    // END_SUB_TUTORIAL

    collision_objects[0].operation = collision_objects[0].ADD;

    // BEGIN_SUB_TUTORIAL table2
    // Add the second table where we will be placing the cube.
    collision_objects[1].id = "table2";
    collision_objects[1].header.frame_id = "world";

    /* Define the primitive and its dimensions. */
    collision_objects[1].primitives.resize(1);
    collision_objects[1].primitives[0].type = collision_objects[1].primitives[0].BOX;
    collision_objects[1].primitives[0].dimensions.resize(3);
    collision_objects[1].primitives[0].dimensions[0] = 0.4;
    collision_objects[1].primitives[0].dimensions[1] = 0.2;
    collision_objects[1].primitives[0].dimensions[2] = 0.4;

    /* Define the pose of the table. */
    collision_objects[1].primitive_poses.resize(1);
    collision_objects[1].primitive_poses[0].position.x = 0;
    collision_objects[1].primitive_poses[0].position.y = 0.5;
    collision_objects[1].primitive_poses[0].position.z = 0.2;
    collision_objects[1].primitive_poses[0].orientation.w = 1.0;
    // END_SUB_TUTORIAL

    collision_objects[1].operation = collision_objects[1].ADD;

    // BEGIN_SUB_TUTORIAL object
    // Define the object that we will be manipulating
    collision_objects[2].header.frame_id = "world";
    collision_objects[2].id = "object";

    /* Define the primitive and its dimensions. */
    collision_objects[2].primitives.resize(1);
    collision_objects[2].primitives[0].type = collision_objects[1].primitives[0].BOX;
    collision_objects[2].primitives[0].dimensions.resize(3);
    collision_objects[2].primitives[0].dimensions[0] = 0.02;
    collision_objects[2].primitives[0].dimensions[1] = 0.02;
    collision_objects[2].primitives[0].dimensions[2] = 0.2;

    /* Define the pose of the object. */
    collision_objects[2].primitive_poses.resize(1);
    collision_objects[2].primitive_poses[0].position.x = 0.5;
    collision_objects[2].primitive_poses[0].position.y = 0;
    collision_objects[2].primitive_poses[0].position.z = 0.5;
    collision_objects[2].primitive_poses[0].orientation.w = 1.0;
    // END_SUB_TUTORIAL

    collision_objects[2].operation = collision_objects[2].ADD;

    planning_scene_interface.applyCollisionObjects(collision_objects);
}

void printPose(moveit::planning_interface::MoveGroupInterface &group)
{
    geometry_msgs::PoseStamped poseStamped = group.getCurrentPose();

    std::cout << "PoseStamped: " << group.getName().c_str() << ";\n";
    std::cout << "pose.position.x = " << poseStamped.pose.position.x << ";\n";
    std::cout << "pose.position.y = " << poseStamped.pose.position.y << ";\n";
    std::cout << "pose.position.z = " << poseStamped.pose.position.z << ";\n";
    std::cout << "pose.orientation.x = " << poseStamped.pose.orientation.x << ";\n";
    std::cout << "pose.orientation.y = " << poseStamped.pose.orientation.y << ";\n";
    std::cout << "pose.orientation.z = " << poseStamped.pose.orientation.z << ";\n";
    std::cout << "pose.orientation.w = " << poseStamped.pose.orientation.w << ";\n";
}

void printPose(moveit_msgs::CollisionObject object)
{
    geometry_msgs::Pose pose = object.pose;

    std::cout << "pose.position.x = " << pose.position.x << ";\n";
    std::cout << "pose.position.y = " << pose.position.y << ";\n";
    std::cout << "pose.position.z = " << pose.position.z << ";\n";
    std::cout << "pose.orientation.x = " << pose.orientation.x << ";\n";
    std::cout << "pose.orientation.y = " << pose.orientation.y << ";\n";
    std::cout << "pose.orientation.z = " << pose.orientation.z << ";\n";
    std::cout << "pose.orientation.w = " << pose.orientation.w << ";\n";
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
    right_gripper_mgi->setMaxVelocityScalingFactor(1.0);
    right_gripper_mgi->setMaxAccelerationScalingFactor(1.0);

    left_arm_mgi->setMaxVelocityScalingFactor(1.0);
    left_arm_mgi->setMaxAccelerationScalingFactor(1.0);
    left_gripper_mgi->setMaxVelocityScalingFactor(1.0);
    left_gripper_mgi->setMaxAccelerationScalingFactor(1.0);

    // right_arm_mgi->setPlannerId("AnytimePathShortening");
    // right_gripper_mgi->setPlannerId("AnytimePathShortening");
    // left_arm_mgi->setPlannerId("AnytimePathShortening");
    // left_gripper_mgi->setPlannerId("AnytimePathShortening");

    namespace rvt = rviz_visual_tools;
    auto visual_tools = std::make_shared<moveit_visual_tools::MoveItVisualTools>("yumi_base_link");
    visual_tools->deleteAllMarkers();
    visual_tools->loadRemoteControl();

    // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().z() = 0.75;
    visual_tools->publishText(text_pose, "NON VERBAL MOVEMENT TESTING", rvt::WHITE, rvt::XXLARGE);
    visual_tools->trigger();

    addCollisionObjects(*planning_scene_interface);

    HRI_Interface right_arm_hri(node_handle, right_arm_mgi, right_arm_jmg, right_gripper_mgi, right_gripper_jmg, planning_scene_interface, visual_tools);
    HRI_Interface left_arm_hri(node_handle, left_arm_mgi, left_arm_jmg, left_gripper_mgi, left_gripper_jmg, planning_scene_interface, visual_tools);

    geometry_msgs::Pose pose = right_arm_mgi->getCurrentPose().pose;

    // for (const std::string &link_name : right_arm_mgi->getLinkNames())
    // {
    //     std::cout << "Right Arm \n";
    //     std::cout << "Link name: " << link_name << std::endl;
    // }

    // for (const std::string &link_name : right_gripper_mgi->getLinkNames())
    // {
    //     std::cout << "Right Gripper \n";
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
             << "5. ROTATE LEFT\n"
             << "6. ROTATE RIGHT\n"
             << "7. SIGNAL PICK\n"
             << "0. EXIT\n"
             << "Enter your choice: ";
        cin >> choice;

        switch (choice)
        {
        case 1:
            cout << "You chose to GO TO TARGET.\n";
            right_arm_hri.attention("my_marker");
            left_arm_hri.attention("my_marker");

            break;
        case 2:
            right_arm_hri.screw_unscrew(mode, right_arm_mgi->getCurrentPose().pose);
            left_arm_hri.screw_unscrew(mode, left_arm_mgi->getCurrentPose().pose);
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
            right_arm_hri.pointToObject(input, "my_marker");
            left_arm_hri.pointToObject(input, "my_marker");
            break;
        case 5:
            right_arm_hri.signalRotateLeft();
            left_arm_hri.signalRotateLeft();

            break;
        case 6:
            right_arm_hri.signalRotateRight();
            left_arm_hri.signalRotateRight();
            break;
        case 7:
            right_arm_hri.signalPick();
            left_arm_hri.signalPick();
            break;
        case 8:
            right_arm_hri.pointToHuman("my_marker");
            left_arm_hri.pointToHuman("my_marker");
            break;
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