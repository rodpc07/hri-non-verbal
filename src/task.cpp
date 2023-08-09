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
#include <string>

#include <memory>
#include <functional>

#include <algorithm>
#include <Eigen/Geometry>

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
        planningSceneClient = n_.serviceClient<moveit_msgs::GetPlanningScene>("get_planning_scene");

        planning_scene_ = std::make_shared<planning_scene::PlanningScene>(arm_mgi_->getRobotModel());
        planning_scene_->setActiveCollisionDetector(collision_detection::CollisionDetectorAllocatorBullet::create(), true);

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

    bool waving(std::string target_frame)
    {
        geometry_msgs::TransformStamped targetTransform;

        // Get transform from human position in reference to base_link

        if (!transformListener(target_frame, "yumi_base_link", targetTransform))
        {
            ROS_ERROR("Can't perform transform");
            return false;
        }

        arm_mgi_->setStartStateToCurrentState();
        moveit::core::RobotState arm_state(*arm_mgi_->getCurrentState());

        Eigen::Isometry3d linkTransform = arm_state.getGlobalLinkTransform(arm_mgi_->getLinkNames().at(1));

        // This joint values set the pose of the arm in a V shape (depends on robot morphology)

        std::vector<double> joint_group_positions;
        if (!arm_mgi_->getName().compare("right_arm"))
        {
            joint_group_positions = {125 * (M_PI / 180), -115 * (M_PI / 180), 0, 60 * (M_PI / 180), 90 * (M_PI / 180), 0, 0};
        }
        else
        {
            joint_group_positions = {-125 * (M_PI / 180), -115 * (M_PI / 180), 0, 60 * (M_PI / 180), -90 * (M_PI / 180), 0, 0};
        }

        arm_state.setJointGroupPositions(arm_jmg_, joint_group_positions);

        const Eigen::Isometry3d &end_effector_state = arm_state.getGlobalLinkTransform(gripper_mgi_->getLinkNames().at(0));

        // Next is done the calculations to obtain the angle for yumi_joint_2 needed to set the arm aligned with the human

        double xTarget = targetTransform.transform.translation.x - linkTransform.translation().x();
        double yTarget = targetTransform.transform.translation.y - linkTransform.translation().y();

        double xEE = end_effector_state.translation().x() - linkTransform.translation().x();
        double yEE = end_effector_state.translation().y() - linkTransform.translation().y();

        double target_angle = atan2(yTarget, xTarget);
        double ee_angle = atan2(yEE, xEE);

        double angle = ee_angle - target_angle;

        joint_group_positions.at(2) = angle;
        arm_mgi_->setJointValueTarget(joint_group_positions);

        moveit::planning_interface::MoveGroupInterface::Plan arm_plan_1;
        if (!(arm_mgi_->plan(arm_plan_1) == moveit::core::MoveItErrorCode::SUCCESS))
        {
            ROS_ERROR("Can't plan for first movement");
            return false;
        }

        // Next are planned in advance 4 motions to simulate waving, later executed sequecially

        arm_state.setJointGroupPositions(arm_jmg_, arm_plan_1.trajectory_.joint_trajectory.points.back().positions);
        joint_group_positions.at(5) = -25 * (M_PI / 180);
        arm_mgi_->setStartState(arm_state);
        arm_mgi_->setJointValueTarget(joint_group_positions);
        moveit::planning_interface::MoveGroupInterface::Plan arm_plan_2;
        if (!(arm_mgi_->plan(arm_plan_2) == moveit::core::MoveItErrorCode::SUCCESS))
        {
            ROS_ERROR("Can't plan for second movement");
            return false;
        }

        arm_state.setJointGroupPositions(arm_jmg_, arm_plan_2.trajectory_.joint_trajectory.points.back().positions);
        joint_group_positions.at(5) = 25 * (M_PI / 180);
        arm_mgi_->setStartState(arm_state);
        arm_mgi_->setJointValueTarget(joint_group_positions);
        moveit::planning_interface::MoveGroupInterface::Plan arm_plan_3;
        if (!(arm_mgi_->plan(arm_plan_3) == moveit::core::MoveItErrorCode::SUCCESS))
        {
            ROS_ERROR("Can't plan for third movement");
            return false;
        }

        arm_state.setJointGroupPositions(arm_jmg_, arm_plan_3.trajectory_.joint_trajectory.points.back().positions);
        joint_group_positions.at(5) = -25 * (M_PI / 180);
        arm_mgi_->setStartState(arm_state);
        arm_mgi_->setJointValueTarget(joint_group_positions);
        moveit::planning_interface::MoveGroupInterface::Plan arm_plan_4;
        if (!(arm_mgi_->plan(arm_plan_4) == moveit::core::MoveItErrorCode::SUCCESS))
        {
            ROS_ERROR("Can't plan for fourth movement");
            return false;
        }

        arm_state.setJointGroupPositions(arm_jmg_, arm_plan_4.trajectory_.joint_trajectory.points.back().positions);
        joint_group_positions.at(5) = 0 * (M_PI / 180);
        arm_mgi_->setStartState(arm_state);
        arm_mgi_->setJointValueTarget(joint_group_positions);
        moveit::planning_interface::MoveGroupInterface::Plan arm_plan_5;
        if (!(arm_mgi_->plan(arm_plan_5) == moveit::core::MoveItErrorCode::SUCCESS))
        {
            ROS_ERROR("Can't plan for fifth movement");
            return false;
        }

        arm_mgi_->setStartStateToCurrentState();
        arm_mgi_->execute(arm_plan_1);
        arm_mgi_->execute(arm_plan_2);
        arm_mgi_->execute(arm_plan_3);
        arm_mgi_->execute(arm_plan_4);
        arm_mgi_->execute(arm_plan_5);

        return true;
    }

    bool comeClose(std::string target_frame)
    {
        geometry_msgs::TransformStamped targetTransform;

        // Get transform from human position in reference to base_link

        if (!transformListener(target_frame, "yumi_base_link", targetTransform))
        {
            ROS_ERROR("Can't perform transform");
            return false;
        }

        arm_mgi_->setStartStateToCurrentState();
        moveit::core::RobotState arm_state(*arm_mgi_->getCurrentState());

        Eigen::Isometry3d linkTransform = arm_state.getGlobalLinkTransform(arm_mgi_->getLinkNames().at(1));

        // This joint values set the pose of the arm in a V shape (depends on robot morphology)

        std::vector<double> joint_group_positions;
        if (!arm_mgi_->getName().compare("right_arm"))
        {
            joint_group_positions = {125 * (M_PI / 180), -115 * (M_PI / 180), 0, 60 * (M_PI / 180), 0, 0, -90 * (M_PI / 180)};
        }
        else
        {
            joint_group_positions = {-125 * (M_PI / 180), -115 * (M_PI / 180), 0, 60 * (M_PI / 180), 0, 0, -90 * (M_PI / 180)};
        }

        arm_state.setJointGroupPositions(arm_jmg_, joint_group_positions);

        const Eigen::Isometry3d &end_effector_state = arm_state.getGlobalLinkTransform(gripper_mgi_->getLinkNames().at(0));

        // Next is done the calculations to obtain the angle for yumi_joint_2 needed to set the arm aligned with the human

        double xTarget = targetTransform.transform.translation.x - linkTransform.translation().x();
        double yTarget = targetTransform.transform.translation.y - linkTransform.translation().y();

        double xEE = end_effector_state.translation().x() - linkTransform.translation().x();
        double yEE = end_effector_state.translation().y() - linkTransform.translation().y();

        double target_angle = atan2(yTarget, xTarget);
        double ee_angle = atan2(yEE, xEE);

        double angle = ee_angle - target_angle;

        joint_group_positions.at(2) = angle;
        arm_mgi_->setJointValueTarget(joint_group_positions);

        moveit::planning_interface::MoveGroupInterface::Plan arm_plan_1;
        if (!(arm_mgi_->plan(arm_plan_1) == moveit::core::MoveItErrorCode::SUCCESS))
        {
            ROS_ERROR("Can't plan for first movement");
            return false;
        }

        // Next are planned in advance 4 motions to simulate waving, later executed sequecially

        arm_state.setJointGroupPositions(arm_jmg_, arm_plan_1.trajectory_.joint_trajectory.points.back().positions);
        joint_group_positions.at(5) = 45 * (M_PI / 180);
        arm_mgi_->setStartState(arm_state);
        arm_mgi_->setJointValueTarget(joint_group_positions);
        moveit::planning_interface::MoveGroupInterface::Plan arm_plan_2;
        if (!(arm_mgi_->plan(arm_plan_2) == moveit::core::MoveItErrorCode::SUCCESS))
        {
            ROS_ERROR("Can't plan for second movement");
            return false;
        }

        arm_state.setJointGroupPositions(arm_jmg_, arm_plan_2.trajectory_.joint_trajectory.points.back().positions);
        joint_group_positions.at(5) = 0 * (M_PI / 180);
        arm_mgi_->setStartState(arm_state);
        arm_mgi_->setJointValueTarget(joint_group_positions);
        moveit::planning_interface::MoveGroupInterface::Plan arm_plan_3;
        if (!(arm_mgi_->plan(arm_plan_3) == moveit::core::MoveItErrorCode::SUCCESS))
        {
            ROS_ERROR("Can't plan for third movement");
            return false;
        }

        arm_mgi_->setStartStateToCurrentState();
        arm_mgi_->execute(arm_plan_1);
        arm_mgi_->execute(arm_plan_2);
        arm_mgi_->execute(arm_plan_3);
        arm_mgi_->execute(arm_plan_2);
        arm_mgi_->execute(arm_plan_3);

        return true;
    }

    bool goAway(std::string target_frame)
    {
        geometry_msgs::TransformStamped targetTransform;

        // Get transform from human position in reference to base_link

        if (!transformListener(target_frame, "yumi_base_link", targetTransform))
        {
            ROS_ERROR("Can't perform transform");
            return false;
        }

        arm_mgi_->setStartStateToCurrentState();
        moveit::core::RobotState arm_state(*arm_mgi_->getCurrentState());

        Eigen::Isometry3d linkTransform = arm_state.getGlobalLinkTransform(arm_mgi_->getLinkNames().at(1));

        // This joint values set the pose of the arm in a V shape (depends on robot morphology)

        std::vector<double> joint_group_positions;
        if (!arm_mgi_->getName().compare("right_arm"))
        {
            joint_group_positions = {125 * (M_PI / 180), -115 * (M_PI / 180), 0, 60 * (M_PI / 180), 0, 0, -90 * (M_PI / 180)};
        }
        else
        {
            joint_group_positions = {-125 * (M_PI / 180), -115 * (M_PI / 180), 0, 60 * (M_PI / 180), 0, 0, -90 * (M_PI / 180)};
        }

        arm_state.setJointGroupPositions(arm_jmg_, joint_group_positions);

        const Eigen::Isometry3d &end_effector_state = arm_state.getGlobalLinkTransform(gripper_mgi_->getLinkNames().at(0));

        // Next is done the calculations to obtain the angle for yumi_joint_2 needed to set the arm aligned with the human

        double xTarget = targetTransform.transform.translation.x - linkTransform.translation().x();
        double yTarget = targetTransform.transform.translation.y - linkTransform.translation().y();

        double xEE = end_effector_state.translation().x() - linkTransform.translation().x();
        double yEE = end_effector_state.translation().y() - linkTransform.translation().y();

        double target_angle = atan2(yTarget, xTarget);
        double ee_angle = atan2(yEE, xEE);

        double angle = ee_angle - target_angle;

        joint_group_positions.at(2) = angle;
        arm_mgi_->setJointValueTarget(joint_group_positions);

        moveit::planning_interface::MoveGroupInterface::Plan arm_plan_1;
        if (!(arm_mgi_->plan(arm_plan_1) == moveit::core::MoveItErrorCode::SUCCESS))
        {
            ROS_ERROR("Can't plan for first movement");
            return false;
        }

        // Next are planned in advance 4 motions to simulate waving, later executed sequecially

        arm_state.setJointGroupPositions(arm_jmg_, arm_plan_1.trajectory_.joint_trajectory.points.back().positions);
        joint_group_positions.at(5) = -45 * (M_PI / 180);
        arm_mgi_->setStartState(arm_state);
        arm_mgi_->setJointValueTarget(joint_group_positions);
        moveit::planning_interface::MoveGroupInterface::Plan arm_plan_2;
        if (!(arm_mgi_->plan(arm_plan_2) == moveit::core::MoveItErrorCode::SUCCESS))
        {
            ROS_ERROR("Can't plan for second movement");
            return false;
        }

        arm_state.setJointGroupPositions(arm_jmg_, arm_plan_2.trajectory_.joint_trajectory.points.back().positions);
        joint_group_positions.at(5) = 0 * (M_PI / 180);
        arm_mgi_->setStartState(arm_state);
        arm_mgi_->setJointValueTarget(joint_group_positions);
        moveit::planning_interface::MoveGroupInterface::Plan arm_plan_3;
        if (!(arm_mgi_->plan(arm_plan_3) == moveit::core::MoveItErrorCode::SUCCESS))
        {
            ROS_ERROR("Can't plan for third movement");
            return false;
        }

        arm_mgi_->setStartStateToCurrentState();
        arm_mgi_->execute(arm_plan_1);
        arm_mgi_->execute(arm_plan_2);
        arm_mgi_->execute(arm_plan_3);
        arm_mgi_->execute(arm_plan_2);
        arm_mgi_->execute(arm_plan_3);

        return true;
    }

    // bool mode (true = screw, false = unscrew)
    bool screw_unscrew(bool mode, geometry_msgs::Pose input_pose)
    {

        arm_mgi_->setStartStateToCurrentState();

        double distance = 0.05;

        double maxBound = arm_mgi_->getRobotModel()->getVariableBounds(arm_mgi_->getActiveJoints().at(6)).max_position_;
        double minBound = arm_mgi_->getRobotModel()->getVariableBounds(arm_mgi_->getActiveJoints().at(6)).min_position_;

        // Set Initial Position to same as argument or offset
        geometry_msgs::Pose initial_pose;
        if (mode)
        {
            Eigen::Isometry3d unscrew_inital_eigen;
            visual_tools_->convertPoseSafe(input_pose, unscrew_inital_eigen);
            unscrew_inital_eigen.translate(Eigen::Vector3d(0, 0, -distance));

            initial_pose = visual_tools_->convertPose(unscrew_inital_eigen);
        }
        else
        {
            initial_pose = input_pose;
        }

        moveit::core::RobotState arm_state(*arm_mgi_->getCurrentState());

        if (!computeLookPose(arm_state, initial_pose, input_pose, 20, 0.5, 0.5))
        {
            return false;
        }

        std::vector<double> joint_group_positions;
        arm_state.copyJointGroupPositions(arm_jmg_, joint_group_positions);

        // Set gripper joint to limit to allow a full range of rotation

        joint_group_positions.back() = mode ? minBound : maxBound;
        arm_mgi_->setJointValueTarget(joint_group_positions);

        moveit::planning_interface::MoveGroupInterface::Plan planSetInitialPosition;

        if (!(arm_mgi_->plan(planSetInitialPosition) == moveit::core::MoveItErrorCode::SUCCESS))
        {
            ROS_ERROR("Can't plan for initial position");
            return false;
        }

        // Perform action of EndEffector

        arm_state.setJointGroupPositions(arm_jmg_, planSetInitialPosition.trajectory_.joint_trajectory.points.back().positions);
        arm_mgi_->setStartState(arm_state);

        // Perform action of EndEffector
        geometry_msgs::Pose start_end_effector_pose = visual_tools_->convertPose(arm_state.getGlobalLinkTransform(arm_mgi_->getLinkNames().back()));
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
                q_new.normalize();
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

        arm_mgi_->setStartStateToCurrentState();
        arm_mgi_->execute(planSetInitialPosition);
        arm_mgi_->execute(trajectory);
        visual_tools_->deleteAllMarkers();

        return true;
    }

    bool pointToPoint(geometry_msgs::Point point)
    {
        visual_tools_->deleteAllMarkers();

        // Create a vector from shoulder to object to calculate pose

        arm_mgi_->setStartStateToCurrentState();
        moveit::core::RobotState arm_state(*arm_mgi_->getCurrentState());

        Eigen::Isometry3d linkTransform = arm_state.getGlobalLinkTransform(arm_mgi_->getLinkNames().at(0));

        double xTarget = point.x - linkTransform.translation().x();
        double yTarget = point.y - linkTransform.translation().y();
        double zTarget = point.z - linkTransform.translation().z();

        double distance = sqrt(pow(xTarget, 2) + pow(yTarget, 2) + pow(zTarget, 2));

        double targetDistance = distance - 0.20 >= 0.6 ? 0.6 : distance - 0.20;

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
        lookPose.position.x = linkTransform.translation().x() + xTarget;
        lookPose.position.y = linkTransform.translation().y() + yTarget;
        lookPose.position.z = linkTransform.translation().z() + zTarget;
        lookPose.orientation = q_msg;

        visual_tools_->publishAxis(lookPose);
        visual_tools_->trigger();

        geometry_msgs::Pose pointPose;
        pointPose.position = point;
        pointPose.orientation.w = 1.0;

        if (!computeLookPose(arm_state, lookPose, pointPose, 10, 2.0, 2.0))
        {
            return false;
        }

        std::vector<double> lookPose_joint_positions;
        arm_state.copyJointGroupPositions(arm_jmg_, lookPose_joint_positions);
        arm_mgi_->setJointValueTarget(lookPose_joint_positions);
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        if (!(arm_mgi_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS))
        {
            ROS_ERROR("Can't plan for pointing movement");
            return false;
        }
        arm_mgi_->execute(plan);
        return true;
    }

    bool pointToObject(std::string object_id)
    {
        visual_tools_->deleteAllMarkers();

        // Obtain object from scene
        std::map<std::string, moveit_msgs::CollisionObject> objects = planning_scene_interface_->getObjects();

        // Find position of object
        moveit_msgs::CollisionObject object = objects[object_id];
        geometry_msgs::Pose object_pose = object.pose;

        double radius = sqrt(pow(object.primitives[0].dimensions[0], 2) + pow(object.primitives[0].dimensions[1], 2) + pow(object.primitives[0].dimensions[2], 2));
        visual_tools_->publishAxis(object.pose);
        visual_tools_->trigger();

        // Create a vector from shoulder to object to calculate pose

        arm_mgi_->setStartStateToCurrentState();
        moveit::core::RobotState arm_state(*arm_mgi_->getCurrentState());

        Eigen::Isometry3d linkTransform = arm_state.getGlobalLinkTransform(arm_mgi_->getLinkNames().at(0));

        double xTarget = object_pose.position.x - linkTransform.translation().x();
        double yTarget = object_pose.position.y - linkTransform.translation().y();
        double zTarget = object_pose.position.z - linkTransform.translation().z();

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
        lookPose.position.x = linkTransform.translation().x() + xTarget;
        lookPose.position.y = linkTransform.translation().y() + yTarget;
        lookPose.position.z = linkTransform.translation().z() + zTarget;
        lookPose.orientation = q_msg;

        visual_tools_->publishAxis(lookPose);
        visual_tools_->trigger();

        if (!computeLookPose(arm_state, lookPose, object_pose, 10, 2.0, 2.0))
        {
            return false;
        }

        std::vector<double> lookPose_joint_positions;
        arm_state.copyJointGroupPositions(arm_jmg_, lookPose_joint_positions);
        arm_mgi_->setJointValueTarget(lookPose_joint_positions);
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        if (!(arm_mgi_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS))
        {
            ROS_ERROR("Can't plan for pointing movement");
            return false;
        }
        arm_mgi_->execute(plan);
        return true;
    }

    bool pointToObjectSide(std::string object_id, Eigen::Vector3d sideInfo)
    {

        visual_tools_->deleteAllMarkers();

        // Obtain object from scene
        std::map<std::string, moveit_msgs::CollisionObject> objects = planning_scene_interface_->getObjects();

        const double tolerance = 1e-6;
        if (objects.count(object_id) == 0)
        {
            ROS_ERROR("Object does not exist.");
            return false;
        }
        if (!(sideInfo.isApprox(Eigen::Vector3d(1, 0, 0), tolerance) || sideInfo.isApprox(Eigen::Vector3d(-1, 0, 0), tolerance) ||
              sideInfo.isApprox(Eigen::Vector3d(0, 1, 0), tolerance) || sideInfo.isApprox(Eigen::Vector3d(0, -1, 0), tolerance) ||
              sideInfo.isApprox(Eigen::Vector3d(0, 0, 1), tolerance) || sideInfo.isApprox(Eigen::Vector3d(0, 0, -1), tolerance)))
        {
            ROS_ERROR("Side information is invalid! Must only provide information about one axis [x, y or z] and direction [1 or -1]");
            return false;
        }

        // Find position of object
        moveit_msgs::CollisionObject object = objects[object_id];
        geometry_msgs::Pose object_pose = object.pose;

        double radius = sqrt(pow(object.primitives[0].dimensions[0], 2) + pow(object.primitives[0].dimensions[1], 2) + pow(object.primitives[0].dimensions[2], 2));
        visual_tools_->publishAxis(object_pose);
        visual_tools_->trigger();

        // Create a vector from shoulder to object to calculate pose

        Eigen::Isometry3d objectEigen;
        visual_tools_->convertPoseSafe(object_pose, objectEigen);

        objectEigen.translate(sideInfo * (radius + 0.1));

        visual_tools_->publishAxis(objectEigen);
        visual_tools_->trigger();

        double xTarget = object_pose.position.x - objectEigen.translation().x();
        double yTarget = object_pose.position.y - objectEigen.translation().y();
        double zTarget = object_pose.position.z - objectEigen.translation().z();

        double sideAngle = atan2(yTarget, xTarget);
        double tiltAngle = M_PI_2 - atan2(zTarget, sqrt(pow(xTarget, 2) + pow(yTarget, 2)));

        tf2::Quaternion q1(tf2::Vector3(0, 0, 1), sideAngle);
        tf2::Quaternion q2(tf2::Vector3(0, 1, 0), tiltAngle);
        tf2::Quaternion qresult = q1 * q2;
        qresult.normalize();

        geometry_msgs::Quaternion q_msg;
        tf2::convert(qresult, q_msg);

        geometry_msgs::Pose lookPose;
        lookPose.position.x = objectEigen.translation().x();
        lookPose.position.y = objectEigen.translation().y();
        lookPose.position.z = objectEigen.translation().z();
        lookPose.orientation = q_msg;

        visual_tools_->publishAxis(lookPose);
        visual_tools_->trigger();

        arm_mgi_->setStartStateToCurrentState();
        moveit::core::RobotState arm_state(*arm_mgi_->getCurrentState());

        if (!computeLookPose(arm_state, lookPose, object_pose, 10, 0.75, 0.75))
        {
            return false;
        }

        std::vector<double>
            lookPose_joint_values;
        arm_state.copyJointGroupPositions(arm_jmg_, lookPose_joint_values);

        arm_mgi_->setJointValueTarget(lookPose_joint_values);
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        if (!(arm_mgi_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS))
        {
            ROS_ERROR("Can't plan for pointing movement");
            return false;
        }
        arm_mgi_->execute(plan);

        return true;
    }

    bool pointToHuman(std::string target_frame)
    {
        visual_tools_->deleteAllMarkers();

        // Calculate New Pose Looking at Human
        geometry_msgs::TransformStamped targetTransform;

        // Get transform from human position in reference to base_link

        if (!transformListener(target_frame, "yumi_base_link", targetTransform))
        {
            ROS_ERROR("Can't perform transform");
            return false;
        }

        arm_mgi_->setStartStateToCurrentState();
        moveit::core::RobotState arm_state(*arm_mgi_->getCurrentState());

        Eigen::Isometry3d linkTransform = arm_state.getGlobalLinkTransform(arm_mgi_->getLinkNames().at(0));

        double xTarget = targetTransform.transform.translation.x - linkTransform.translation().x();
        double yTarget = targetTransform.transform.translation.y - linkTransform.translation().y();
        double zTarget = targetTransform.transform.translation.z - linkTransform.translation().z();

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
        lookPose.position.x = linkTransform.translation().x() + xTarget;
        lookPose.position.y = linkTransform.translation().y() + yTarget;
        lookPose.position.z = linkTransform.translation().z() + zTarget;
        lookPose.orientation = q_msg;

        visual_tools_->publishAxis(lookPose);
        visual_tools_->trigger();

        geometry_msgs::Pose humanPose;
        humanPose.position.x = targetTransform.transform.translation.x;
        humanPose.position.y = targetTransform.transform.translation.y;
        humanPose.position.z = targetTransform.transform.translation.z;
        humanPose.orientation.w = 1;

        if (!computeLookPose(arm_state, lookPose, humanPose, 10, 2.0, 2.0))
        {
            return false;
        }

        std::vector<double> lookPose_joint_positions;
        arm_state.copyJointGroupPositions(arm_jmg_, lookPose_joint_positions);
        arm_mgi_->setJointValueTarget(lookPose_joint_positions);
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        if (!(arm_mgi_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS))
        {
            ROS_ERROR("Can't plan for pointing movement");
            return false;
        }
        arm_mgi_->execute(plan);
        return true;
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

        for (const auto &planValue : planList)
        {
            gripper_mgi_->execute(planValue);
        }
    }

    bool signalRotate(std::string object_id, Eigen::Vector3d rotationInfo)
    {
        visual_tools_->deleteAllMarkers();

        // Obtain object from scene
        std::map<std::string, moveit_msgs::CollisionObject> objects = planning_scene_interface_->getObjects();

        const double tolerance = 1e-6;
        if (objects.count(object_id) == 0)
        {
            ROS_ERROR("Object does not exist.");
            return false;
        }
        if (!(rotationInfo.isApprox(Eigen::Vector3d(1, 0, 0), tolerance) || rotationInfo.isApprox(Eigen::Vector3d(-1, 0, 0), tolerance) ||
              rotationInfo.isApprox(Eigen::Vector3d(0, 1, 0), tolerance) || rotationInfo.isApprox(Eigen::Vector3d(0, -1, 0), tolerance) ||
              rotationInfo.isApprox(Eigen::Vector3d(0, 0, 1), tolerance) || rotationInfo.isApprox(Eigen::Vector3d(0, 0, -1), tolerance)))
        {
            ROS_ERROR("Rotation information is invalid! Must only provide information about one axis [x, y or z] and direction [1 or -1]");
            return false;
        }

        // Find position of object
        moveit_msgs::CollisionObject object = objects[object_id];
        geometry_msgs::Pose object_pose = object.pose;

        double radius = sqrt(pow(object.primitives[0].dimensions[0], 2) + pow(object.primitives[0].dimensions[1], 2) + pow(object.primitives[0].dimensions[2], 2));
        visual_tools_->publishAxis(object_pose);
        visual_tools_->trigger();

        Eigen::Isometry3d objectEigen;
        visual_tools_->convertPoseSafe(object_pose, objectEigen);

        std::vector<Eigen::Isometry3d> approach_options;
        for (int i = 0; i < 2; i++)
        {
            approach_options.push_back(objectEigen);
            approach_options[i].translate(rotationInfo * pow(-1, i) * (radius + 0.12));
        }

        arm_mgi_->setStartStateToCurrentState();
        moveit::core::RobotState arm_state(*arm_mgi_->getCurrentState());

        Eigen::Isometry3d linkTransform = arm_state.getGlobalLinkTransform(arm_mgi_->getLinkNames().at(0));

        std::vector<Eigen::Isometry3d> closestApproachOption = findClosestApproachOption(approach_options, linkTransform);

        visual_tools_->publishAxis(approach_options[0]);
        visual_tools_->publishAxis(approach_options[1]);
        visual_tools_->publishAxisLabeled(closestApproachOption[0], "Closest");
        visual_tools_->trigger();

        bool sucess_pose = false;
        geometry_msgs::Pose lookPose;

        for (const auto approach : closestApproachOption)
        {

            double xTarget = object_pose.position.x - approach.translation().x();
            double yTarget = object_pose.position.y - approach.translation().y();
            double zTarget = object_pose.position.z - approach.translation().z();

            double sideAngle = atan2(yTarget, xTarget);
            double tiltAngle = M_PI_2 - atan2(zTarget, sqrt(pow(xTarget, 2) + pow(yTarget, 2)));

            tf2::Quaternion q1(tf2::Vector3(0, 0, 1), sideAngle);
            tf2::Quaternion q2(tf2::Vector3(0, 1, 0), tiltAngle);
            tf2::Quaternion qresult = q1 * q2;
            qresult.normalize();

            geometry_msgs::Quaternion q_msg;
            tf2::convert(qresult, q_msg);

            lookPose.position.x = approach.translation().x();
            lookPose.position.y = approach.translation().y();
            lookPose.position.z = approach.translation().z();
            lookPose.orientation = q_msg;

            visual_tools_->publishAxis(lookPose);
            visual_tools_->trigger();

            if (computeLookPose(arm_state, lookPose, object_pose, 10, 1.0, 1.0))
            {
                sucess_pose = true;
                break;
            }
        }

        if (!sucess_pose)
        {
            ROS_ERROR("Can't find suitable pose");
            return false;
        }

        std::vector<double> lookPose_joint_values;
        arm_state.copyJointGroupPositions(arm_jmg_, lookPose_joint_values);

        double maxBound = arm_mgi_->getRobotModel()->getVariableBounds(arm_mgi_->getActiveJoints().at(6)).max_position_;
        double minBound = arm_mgi_->getRobotModel()->getVariableBounds(arm_mgi_->getActiveJoints().at(6)).min_position_;

        if (abs(maxBound - lookPose_joint_values.back()) < M_PI_2)
        {
            lookPose_joint_values.back() -= M_PI_2 - abs(maxBound - lookPose_joint_values.back());
        }
        else if (abs(minBound - lookPose_joint_values.back()) < M_PI_2)
        {
            lookPose_joint_values.back() += M_PI_2 - abs(minBound - lookPose_joint_values.back());
        }

        arm_mgi_->setJointValueTarget(lookPose_joint_values);
        moveit::planning_interface::MoveGroupInterface::Plan first_movement;
        if (!(arm_mgi_->plan(first_movement) == moveit::core::MoveItErrorCode::SUCCESS))
        {
            ROS_ERROR("Can't plan for pointing movement");
            return false;
        }

        // Set Rotated Position as JointValueTarget and Plan
        lookPose_joint_values = first_movement.trajectory_.joint_trajectory.points.back().positions;
        arm_state.setJointGroupPositions(arm_jmg_, lookPose_joint_values);
        arm_mgi_->setStartState(arm_state);
        // Change End-Effector Rotation
        std::vector<double> rotated_joint_values = lookPose_joint_values;

        // Check if the Z axis of the EE is aligned with the object rotation axis
        Eigen::Quaterniond look_orientation(lookPose.orientation.w, lookPose.orientation.x, lookPose.orientation.y, lookPose.orientation.z);
        Eigen::Vector3d look_z_axis = look_orientation * Eigen::Vector3d::UnitZ();

        Eigen::Quaterniond object_orientation(object_pose.orientation.w, object_pose.orientation.x, object_pose.orientation.y, object_pose.orientation.z);
        Eigen::Vector3d object_x_axis = object_orientation * rotationInfo;

        if (look_z_axis.dot(object_x_axis) > 0)
        {
            rotated_joint_values.back() += M_PI_2;
        }
        else
        {
            rotated_joint_values.back() -= M_PI_2;
        }

        arm_mgi_->setJointValueTarget(rotated_joint_values);
        moveit::planning_interface::MoveGroupInterface::Plan first_rotation;
        if (!(arm_mgi_->plan(first_rotation) == moveit::core::MoveItErrorCode::SUCCESS))
        {
            ROS_ERROR("Can't plan for first rotation");
            return false;
        }

        // Set Pre-Rotation Position as JointValueTarget and Plan
        arm_state.setJointGroupPositions(arm_jmg_, rotated_joint_values);
        arm_mgi_->setStartState(arm_state);
        arm_mgi_->setJointValueTarget(lookPose_joint_values);
        moveit::planning_interface::MoveGroupInterface::Plan second_rotation;
        if (!(arm_mgi_->plan(second_rotation) == moveit::core::MoveItErrorCode::SUCCESS))
        {
            ROS_ERROR("Can't plan for second rotation");
            return false;
        }

        arm_mgi_->execute(first_movement);
        arm_mgi_->execute(first_rotation);
        arm_mgi_->execute(second_rotation);

        return true;
    }

private:
    ros::NodeHandle n_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_mgi_;
    const moveit::core::JointModelGroup *arm_jmg_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> gripper_mgi_;
    const moveit::core::JointModelGroup *gripper_jmg_;
    std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;
    std::shared_ptr<moveit_visual_tools::MoveItVisualTools> visual_tools_;
    std::shared_ptr<planning_scene::PlanningScene> planning_scene_;

    ros::ServiceClient planningSceneClient;

    moveit::planning_interface::MoveGroupInterface::Plan openPlan;
    moveit::planning_interface::MoveGroupInterface::Plan closePlan;
    std::vector<double> closedJointValues;
    std::vector<double> openedJointValues;

    bool transformListener(std::string source_frame, std::string target_frame,
                           geometry_msgs::TransformStamped &transform_stamped)
    {
        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tfListener(tfBuffer);
        ros::Rate rate(50.0);

        ros::Time startTime = ros::Time::now();

        while (ros::Time::now() - startTime <= ros::Duration(5.0))
        {
            try
            {
                transform_stamped = tfBuffer.lookupTransform(target_frame, source_frame, ros::Time(0));

                if (transform_stamped.header.stamp.isValid())
                    return true;
            }
            catch (tf2::TransformException &ex)
            {
                ROS_WARN("%s", ex.what());
                ros::Duration(1.0).sleep();
                continue;
            }

            rate.sleep();
        }

        return false;
    }

    void updatePlanningScene(std::shared_ptr<planning_scene::PlanningScene> planning_scene)
    {
        moveit_msgs::GetPlanningScene::Request req;
        moveit_msgs::GetPlanningScene::Response res;

        if (planningSceneClient.call(req, res))
            planning_scene->setPlanningSceneMsg(res.scene); // apply result to actual PlanningScene
    }

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

    std::vector<Eigen::Isometry3d> findClosestApproachOption(const std::vector<Eigen::Isometry3d> &approach_options, const Eigen::Isometry3d &linkTransform)
    {
        std::vector<Eigen::Isometry3d> sorted_approach_options = approach_options;
        std::sort(sorted_approach_options.begin(), sorted_approach_options.end(), [&linkTransform](const Eigen::Isometry3d &p1, const Eigen::Isometry3d &p2)
                  {
        double d1 = (linkTransform.translation() - p1.translation()).norm();
        double d2 = (linkTransform.translation() - p2.translation()).norm();
        return (d1 < d2); });

        return sorted_approach_options;
    }

    std::vector<geometry_msgs::Pose> computePointsOnSphere(int numPoints, geometry_msgs::Point point, geometry_msgs::Point reference_position, double theta_distance, double phi_distance)
    {
        double theta, phi; // Polar and azimuthal angles

        // Convert given point to spherical coordinates

        double x = point.x - reference_position.x;
        double y = point.y - reference_position.y;
        double z = point.z - reference_position.z;

        double r = sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2));
        theta = acos(z / r);
        phi = atan2(y, x);

        std::vector<geometry_msgs::Pose> poses;

        // Compute points around the given point on the sphere
        for (int i = -numPoints / 2; i < numPoints / 2; ++i)
        {
            double newTheta = theta + (i * theta_distance / numPoints); // Incrementing the polar angle
            for (int j = -numPoints / 2; j < numPoints / 2; ++j)
            {
                double newPhi = phi + (j * phi_distance / numPoints); // Incrementing the azimuthal angle
                geometry_msgs::Pose newPoint;
                newPoint.position.x = r * sin(newTheta) * cos(newPhi);
                newPoint.position.y = r * sin(newTheta) * sin(newPhi);
                newPoint.position.z = r * cos(newTheta);

                double sideAngle = atan2(newPoint.position.y, newPoint.position.x);
                double tiltAngle = -M_PI_2 - atan2(newPoint.position.z, sqrt(pow(newPoint.position.x, 2) + pow(newPoint.position.y, 2)));

                tf2::Quaternion q1(tf2::Vector3(0, 0, 1), sideAngle);
                tf2::Quaternion q2(tf2::Vector3(0, 1, 0), tiltAngle);
                tf2::Quaternion qresult = q1 * q2;
                qresult.normalize();

                geometry_msgs::Quaternion q_msg;
                tf2::convert(qresult, q_msg);

                newPoint.position.x += reference_position.x;
                newPoint.position.y += reference_position.y;
                newPoint.position.z += reference_position.z;
                newPoint.orientation = q_msg;

                poses.push_back(newPoint);
            }
        }

        std::sort(poses.begin(), poses.end(), [&point](const geometry_msgs::Pose &p1, const geometry_msgs::Pose &p2)
                  {
            double d1 = sqrt(pow(p1.position.x - point.x, 2) + pow(p1.position.y - point.y, 2) + pow(p1.position.z - point.z, 2));
            double d2 = sqrt(pow(p2.position.x - point.x, 2) + pow(p2.position.y - point.y, 2) + pow(p2.position.z - point.z, 2));
            return (d1 < d2); });

        return poses;
    }

    bool isStateValid(moveit::core::RobotState *arm_state, const moveit::core::JointModelGroup *group, const double *joint_group_variable_values)
    {
        updatePlanningScene(planning_scene_);

        arm_state->setJointGroupPositions(group, joint_group_variable_values);
        arm_state->update();

        visual_tools_->publishRobotState(*arm_state, rviz_visual_tools::GREEN);

        collision_detection::CollisionRequest collision_request;
        collision_request.contacts = false;
        collision_request.distance = false;
        collision_request.cost = false;
        collision_detection::CollisionResult collision_result;
        collision_result.clear();
        planning_scene_->getCurrentStateNonConst() = *arm_state;
        planning_scene_->checkCollision(collision_request, collision_result);

        ROS_INFO_STREAM("COLLISION RESULT is " << ((!collision_result.collision) ? "valid" : "not valid"));

        return !collision_result.collision;
    }

    bool computeLookPose(moveit::core::RobotState &arm_state, geometry_msgs::Pose lookPose, geometry_msgs::Pose focus_position, int numPoints, double theta, double phi)
    {
        if (!arm_state.setFromIK(arm_jmg_, lookPose, 0.5, std::bind(&HRI_Interface::isStateValid, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3)))
        {
            std::vector<geometry_msgs::Pose> pose_vector = computePointsOnSphere(numPoints, lookPose.position, focus_position.position, theta, phi);

            for (const auto &pose : pose_vector)
            {

                visual_tools_->publishAxis(pose);
                visual_tools_->trigger();

                if (arm_state.setFromIK(arm_jmg_, pose, 0.5, std::bind(&HRI_Interface::isStateValid, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3)))
                {
                    return true;
                }
            }

            ROS_ERROR("Pose cannot be achieved.");
            return false;
        }

        return true;
    }
};

void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface &planning_scene_interface)
{

    std::vector<moveit_msgs::CollisionObject> collision_objects;
    std::vector<moveit_msgs::ObjectColor> object_colors;

    collision_objects.resize(4);
    object_colors.resize(4);

    int i = 0;

    collision_objects[i].id = "board_A";
    collision_objects[i].header.frame_id = "yumi_base_link";

    collision_objects[i].primitives.resize(1);
    collision_objects[i].primitives[0].type = collision_objects[0].primitives[0].BOX;
    collision_objects[i].primitives[0].dimensions.resize(3);
    // collision_objects[i].primitives[0].dimensions[0] = 0.05334;
    // collision_objects[i].primitives[0].dimensions[1] = 0.06858;
    // collision_objects[i].primitives[0].dimensions[2] = 0.0016;
    collision_objects[i].primitives[0].dimensions[0] = 0.04;
    collision_objects[i].primitives[0].dimensions[1] = 0.06;
    collision_objects[i].primitives[0].dimensions[2] = 0.0016;

    collision_objects[i].primitive_poses.resize(1);
    collision_objects[i].primitive_poses[0].position.x = 0.36;
    collision_objects[i].primitive_poses[0].position.y = 0.80;
    collision_objects[i].primitive_poses[0].position.z = 0.015 + 0.0016 / 2;
    collision_objects[i].primitive_poses[0].orientation.w = 1.0;

    collision_objects[i].operation = collision_objects[i].ADD;

    object_colors[i].id = collision_objects[i].id;

    std_msgs::ColorRGBA boardA_color;
    boardA_color.r = 1;
    boardA_color.g = 0;
    boardA_color.b = 0;
    boardA_color.a = 1;

    object_colors[i].color = boardA_color;

    i++;

    collision_objects[i].id = "board_B";
    collision_objects[i].header.frame_id = "yumi_base_link";

    collision_objects[i].primitives.resize(1);
    collision_objects[i].primitives[0].type = collision_objects[0].primitives[0].BOX;
    collision_objects[i].primitives[0].dimensions.resize(3);
    // collision_objects[i].primitives[0].dimensions[0] = 0.05334;
    // collision_objects[i].primitives[0].dimensions[1] = 0.06858;
    // collision_objects[i].primitives[0].dimensions[2] = 0.0016;
    collision_objects[i].primitives[0].dimensions[0] = 0.04;
    collision_objects[i].primitives[0].dimensions[1] = 0.06;
    collision_objects[i].primitives[0].dimensions[2] = 0.0016;

    collision_objects[i].primitive_poses.resize(1);
    collision_objects[i].primitive_poses[0].position.x = 0.56;
    collision_objects[i].primitive_poses[0].position.y = 0.80;
    collision_objects[i].primitive_poses[0].position.z = 0.015 + 0.0016 / 2;
    collision_objects[i].primitive_poses[0].orientation.w = 1.0;

    collision_objects[i].operation = collision_objects[i].ADD;

    object_colors[i].id = collision_objects[i].id;

    std_msgs::ColorRGBA boardB_color;
    boardB_color.r = 0;
    boardB_color.g = 1;
    boardB_color.b = 0;
    boardB_color.a = 1;

    object_colors[i].color = boardB_color;

    i++;

    collision_objects[i].id = "table";
    collision_objects[i].header.frame_id = "yumi_base_link";

    collision_objects[i].primitives.resize(1);
    collision_objects[i].primitives[0].type = collision_objects[1].primitives[0].BOX;
    collision_objects[i].primitives[0].dimensions.resize(3);
    collision_objects[i].primitives[0].dimensions[0] = 0.75;
    collision_objects[i].primitives[0].dimensions[1] = 1.8;
    collision_objects[i].primitives[0].dimensions[2] = 0.03;

    collision_objects[i].primitive_poses.resize(1);
    collision_objects[i].primitive_poses[0].position.x = 0.515;
    collision_objects[i].primitive_poses[0].position.y = 0.0;
    collision_objects[i].primitive_poses[0].position.z = 0.0;
    collision_objects[i].primitive_poses[0].orientation.w = 1.0;

    collision_objects[i].operation = collision_objects[i].ADD;

    object_colors[i].id = collision_objects[i].id;

    std_msgs::ColorRGBA table_color;
    table_color.r = 1;
    table_color.g = 1;
    table_color.b = 1;
    table_color.a = 1;

    object_colors[i].color = table_color;

    i++;

    collision_objects[i].id = "test_station";
    collision_objects[i].header.frame_id = "yumi_base_link";

    collision_objects[i].primitives.resize(1);
    collision_objects[i].primitives[0].type = collision_objects[1].primitives[0].BOX;
    collision_objects[i].primitives[0].dimensions.resize(3);
    collision_objects[i].primitives[0].dimensions[0] = 0.02;
    collision_objects[i].primitives[0].dimensions[1] = 0.04;
    collision_objects[i].primitives[0].dimensions[2] = 0.03;

    collision_objects[i].primitive_poses.resize(1);
    collision_objects[i].primitive_poses[0].position.x = 0.38;
    collision_objects[i].primitive_poses[0].position.y = 0.0;
    collision_objects[i].primitive_poses[0].position.z = 0.03;
    collision_objects[i].primitive_poses[0].orientation.w = 1.0;

    collision_objects[i].operation = collision_objects[i].ADD;

    object_colors[i].id = collision_objects[i].id;
    object_colors[i].color = table_color;

    planning_scene_interface.applyCollisionObjects(collision_objects, object_colors);
}

void moveObject(moveit::planning_interface::PlanningSceneInterface &planning_scene_interface, std::string object_id, double x = 0, double y = 0, double z = 0)
{

    std::map<std::string, moveit_msgs::CollisionObject> objects = planning_scene_interface.getObjects();

    moveit_msgs::CollisionObject object = objects[object_id];

    object.pose.position.x = x;
    object.pose.position.y = y;
    object.pose.position.z = z;

    object.operation = object.ADD;

    planning_scene_interface.applyCollisionObject(object);
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

void pick1(moveit::planning_interface::MoveGroupInterface &arm_mgi, moveit::planning_interface::MoveGroupInterface &gripper_mgi)
{
    // Create a vector of grasps to be attempted, currently only creating single grasp.
    // This is essentially useful when using a grasp generator to generate and test multiple grasps.
    std::vector<moveit_msgs::Grasp> grasps;
    grasps.resize(1);

    // Actual Grasping Pose
    grasps[0].grasp_pose.header.frame_id = "yumi_base_link";
    grasps[0].grasp_pose.pose.position.x = 0.389111;
    grasps[0].grasp_pose.pose.position.y = 0.210832;
    grasps[0].grasp_pose.pose.position.z = 0.15828;
    grasps[0].grasp_pose.pose.orientation.x = 0.999895;
    grasps[0].grasp_pose.pose.orientation.y = 0.0143787;
    grasps[0].grasp_pose.pose.orientation.z = 1.23117e-05;
    grasps[0].grasp_pose.pose.orientation.w = 0.00175959;

    // Pre Grasp (vector representing the direction of approach)
    grasps[0].pre_grasp_approach.direction.header.frame_id = "yumi_base_link";
    grasps[0].pre_grasp_approach.direction.vector.z = -1.0;
    grasps[0].pre_grasp_approach.min_distance = 0.01;
    grasps[0].pre_grasp_approach.desired_distance = 0.05;

    // Post Grasp (vector representing the direction of retreat)
    grasps[0].post_grasp_retreat.direction.header.frame_id = "yumi_base_link";
    grasps[0].post_grasp_retreat.direction.vector.z = 1.0;
    grasps[0].post_grasp_retreat.min_distance = 0.01;
    grasps[0].post_grasp_retreat.desired_distance = 0.05;

    // Open Gripper Pose
    for (const std::string joint_name : gripper_mgi.getVariableNames())
    {
        grasps[0].pre_grasp_posture.joint_names.push_back(joint_name);
    };

    grasps[0].pre_grasp_posture.points.resize(1);
    grasps[0].pre_grasp_posture.points[0].positions.resize(2);
    grasps[0].pre_grasp_posture.points[0].positions[0] = 0.025;
    grasps[0].pre_grasp_posture.points[0].positions[1] = 0.025;
    grasps[0].pre_grasp_posture.points[0].time_from_start = ros::Duration(0.5);

    // Close Gripper Pose
    for (const std::string joint_name : gripper_mgi.getVariableNames())
    {
        grasps[0].grasp_posture.joint_names.push_back(joint_name);
    };

    grasps[0].grasp_posture.points.resize(1);
    grasps[0].grasp_posture.points[0].positions.resize(2);
    grasps[0].grasp_posture.points[0].positions[0] = 0.02;
    grasps[0].grasp_posture.points[0].positions[1] = 0.02;
    grasps[0].grasp_posture.points[0].time_from_start = ros::Duration(0.5);

    arm_mgi.setSupportSurfaceName("table");
    arm_mgi.pick("board_A", grasps);
}

void place1(moveit::planning_interface::MoveGroupInterface &arm_mgi, moveit::planning_interface::MoveGroupInterface &gripper_mgi)
{
    std::vector<moveit_msgs::PlaceLocation> place_location;
    place_location.resize(1);

    // Actual Placing Pose (this is the object pose and not the gripper pose)
    place_location[0].place_pose.header.frame_id = "yumi_base_link";
    place_location[0].place_pose.pose.position.x = 0.38;
    place_location[0].place_pose.pose.position.y = 0;
    place_location[0].place_pose.pose.position.z = 0.045 + 0.0016 / 2;
    place_location[0].place_pose.pose.orientation.x = 0;
    place_location[0].place_pose.pose.orientation.y = 0;
    place_location[0].place_pose.pose.orientation.z = 0;
    place_location[0].place_pose.pose.orientation.w = 1;

    // Pre Place Pose (vector representing the direction of approach, simetric to the pick)
    place_location[0].pre_place_approach.direction.header.frame_id = "yumi_base_link";
    place_location[0].pre_place_approach.direction.vector.z = -1.0;
    place_location[0].pre_place_approach.min_distance = 0.01;
    place_location[0].pre_place_approach.desired_distance = 0.05;

    // Pre Place Pose (vector representing the direction of retreat)
    place_location[0].post_place_retreat.direction.header.frame_id = "yumi_base_link";
    place_location[0].post_place_retreat.direction.vector.z = 1.0;
    place_location[0].post_place_retreat.min_distance = 0.01;
    place_location[0].post_place_retreat.desired_distance = 0.05;

    // Open Gripper Pose
    for (const std::string joint_name : gripper_mgi.getVariableNames())
    {
        place_location[0].post_place_posture.joint_names.push_back(joint_name);
    };

    place_location[0].post_place_posture.points.resize(1);
    place_location[0].post_place_posture.points[0].positions.resize(2);
    place_location[0].post_place_posture.points[0].positions[0] = 0.025;
    place_location[0].post_place_posture.points[0].positions[1] = 0.025;
    place_location[0].post_place_posture.points[0].time_from_start = ros::Duration(0.5);

    arm_mgi.setSupportSurfaceName("test_station");
    arm_mgi.place("board_A", place_location);
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
    right_arm_mgi->setPlanningTime(5.0);
    right_arm_mgi->allowReplanning(true);
    right_gripper_mgi->setMaxVelocityScalingFactor(1.0);
    right_gripper_mgi->setMaxAccelerationScalingFactor(1.0);
    right_gripper_mgi->setPlanningTime(5.0);
    right_gripper_mgi->allowReplanning(true);

    left_arm_mgi->setMaxVelocityScalingFactor(1.0);
    left_arm_mgi->setMaxAccelerationScalingFactor(1.0);
    left_arm_mgi->setPlanningTime(5.0);
    left_arm_mgi->allowReplanning(true);
    left_gripper_mgi->setMaxVelocityScalingFactor(1.0);
    left_gripper_mgi->setMaxAccelerationScalingFactor(1.0);
    left_gripper_mgi->setPlanningTime(5.0);
    left_gripper_mgi->allowReplanning(true);

    cout << left_arm_mgi->getPlanningTime() << endl;

    namespace rvt = rviz_visual_tools;
    auto visual_tools = std::make_shared<moveit_visual_tools::MoveItVisualTools>("yumi_base_link");
    visual_tools->deleteAllMarkers();
    visual_tools->loadRemoteControl();
    // visual_tools->setGlobalScale(0.4);

    // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().z() = 0.75;
    visual_tools->publishText(text_pose, "Task 1", rvt::WHITE, rvt::XXLARGE);
    visual_tools->trigger();

    addCollisionObjects(*planning_scene_interface);

    HRI_Interface right_arm_hri(node_handle, right_arm_mgi, right_arm_jmg, right_gripper_mgi, right_gripper_jmg, planning_scene_interface, visual_tools);
    HRI_Interface left_arm_hri(node_handle, left_arm_mgi, left_arm_jmg, left_gripper_mgi, left_gripper_jmg, planning_scene_interface, visual_tools);

    std::map<std::string, moveit_msgs::CollisionObject> objects;

    // Find position of object
    moveit_msgs::CollisionObject object;

    std::string human = "my_marker";

    geometry_msgs::Point point;
    point.x = 0.39;
    point.y = 0.21;
    point.z = 0.015 + 0.0016 / 2;

    int choice;
    string input;
    do
    {
        cout << "MENU:\n"
             << "1. PRINT POSE\n"
             << "2. PRINT OBJECT POSE\n"
             << "3. EXECUTE TASK\n"
             << "0. EXIT\n"
             << "Enter your choice: ";
        cin >> choice;

        switch (choice)
        {
        case 1:
            cout << "PRINTING POSE.\n";
            printPose(*right_arm_mgi);
            printPose(*left_arm_mgi);
            break;
        case 2:
            cout << "Object ID.\n";
            cin >> input;
            objects = planning_scene_interface->getObjects();
            object = objects[input];
            printPose(object);
            break;
        case 3:
            cout << "EXECUTING TASK.\n";
            left_arm_hri.pointToHuman(human);
            ros::WallDuration(1).sleep();

            left_arm_hri.pointToObject("board_A");
            left_arm_hri.signalPick();

            left_arm_hri.pointToPoint(point);

            left_gripper_mgi->setNamedTarget("open");
            left_gripper_mgi->move();

            left_arm_mgi->setNamedTarget("side");
            left_arm_mgi->move();

            visual_tools->prompt("Put Board in required position");

            moveObject(*planning_scene_interface, "board_A", 0.39, 0.21, 0.015 + 0.0016 / 2);
            ros::WallDuration(1).sleep();

            pick1(*left_arm_mgi, *left_gripper_mgi);
            place1(*left_arm_mgi, *left_gripper_mgi);

            left_arm_mgi->setNamedTarget("side");
            left_arm_mgi->move();

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