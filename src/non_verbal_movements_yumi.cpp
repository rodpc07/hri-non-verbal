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

#include <iostream>
#include <string>

#include <memory>

using namespace std;

const double tau = 2 * M_PI;

class HRI_Interface
{
public:
    HRI_Interface(std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface,
                  std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface,
                  const moveit::core::JointModelGroup *joint_model_group,
                  std::shared_ptr<moveit_visual_tools::MoveItVisualTools> visual_tools)
        : move_group_interface_(move_group_interface), planning_scene_interface_(planning_scene_interface), joint_model_group_(joint_model_group), visual_tools_(visual_tools)
    {
        robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
        kinematic_model_ = robot_model_loader.getModel();
        kinematic_state_ = std::make_shared<robot_state::RobotState>(kinematic_model_);
        planning_scene_ = std::make_shared<planning_scene::PlanningScene>(kinematic_model_);
    }

    void yes_no()
    {
        // Set constraints for arm pose
        moveit_msgs::Constraints path_constraints;
        // Add joint constraints
        moveit_msgs::JointConstraint joint_constraint;

        joint_constraint.joint_name = move_group_interface_->getActiveJoints().at(0); // "yumi_joint_1_r"
        joint_constraint.position = 125 * (M_PI / 180);
        joint_constraint.tolerance_above = 0.2;
        joint_constraint.tolerance_below = 0.2;
        path_constraints.joint_constraints.push_back(joint_constraint);

        joint_constraint.joint_name = move_group_interface_->getActiveJoints().at(1); // "yumi_joint_2_r"
        joint_constraint.position = -115 * (M_PI / 180);
        joint_constraint.tolerance_above = 0.1;
        joint_constraint.tolerance_below = 0.1;
        path_constraints.joint_constraints.push_back(joint_constraint);

        joint_constraint.joint_name = move_group_interface_->getActiveJoints().at(3); // "yumi_joint_3_r"
        joint_constraint.position = 45 * (M_PI / 180);
        joint_constraint.tolerance_above = 0.2;
        joint_constraint.tolerance_below = 0.2;
        path_constraints.joint_constraints.push_back(joint_constraint);
        // Set the path constraints for the goal
        move_group_interface_->setPathConstraints(path_constraints);
        move_group_interface_->setGoalPositionTolerance(0.1);

        // The eef's final pose will always be upright and about 0.4 units high.
        geometry_msgs::Pose final_pose;
        final_pose.position.z = 0.40;
        final_pose.orientation.x = 0;
        final_pose.orientation.y = 0;
        final_pose.orientation.z = 0;
        final_pose.orientation.w = 1;

        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener listener(tfBuffer);
        geometry_msgs::TransformStamped transformStamped;
        try
        {
            transformStamped = tfBuffer.lookupTransform("my_marker", "yumi_link_1_r", ros::Time(0), ros::Duration(5));
        }
        catch (tf2::TransformException ex)
        {
            ROS_ERROR("%s", ex.what());
            ros::Duration(1.0).sleep();
        }

        double dist_ratio = 0.4 / sqrt(pow(transformStamped.transform.translation.x, 2) + pow(transformStamped.transform.translation.y, 2));

        // Multiply dist_ratio to x and y values
        double transformed_x = transformStamped.transform.translation.x * dist_ratio;
        double transformed_y = transformStamped.transform.translation.y * dist_ratio;

        geometry_msgs::PoseStamped poseStamped_arm_referenced;
        poseStamped_arm_referenced.header.stamp = ros::Time::now();
        poseStamped_arm_referenced.header.frame_id = "my_marker";
        poseStamped_arm_referenced.pose.position.x = transformed_x;
        poseStamped_arm_referenced.pose.position.y = transformed_y;
        poseStamped_arm_referenced.pose.position.z = 0.0; // Assuming the z-coordinate remains the same

        geometry_msgs::PoseStamped poseStamped_base_referenced = tfBuffer.transform(poseStamped_arm_referenced, "yumi_base_link");

        final_pose.position.x = poseStamped_base_referenced.pose.position.x;
        final_pose.position.y = poseStamped_base_referenced.pose.position.y;

        move_group_interface_->setPoseTarget(final_pose);

        moveit::planning_interface::MoveGroupInterface::Plan plan;

        bool success = (move_group_interface_->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        // visual_tools_->deleteAllMarkers();
        // visual_tools_->publishTrajectoryPath(plan.trajectory_, plan.start_state_, true);
        // visual_tools_->trigger();

        move_group_interface_->move();
        move_group_interface_->clearPathConstraints();
    }

    void screw_unscrew(geometry_msgs::Pose input_pose, std::string mode)
    {
        double distance = 0.05;

        double maxBound = kinematic_model_->getVariableBounds(move_group_interface_->getActiveJoints().at(6)).max_position_;
        double minBound = kinematic_model_->getVariableBounds(move_group_interface_->getActiveJoints().at(6)).min_position_;

        Eigen::Isometry3d unscrew_inital_eigen;
        visual_tools_->convertPoseSafe(input_pose, unscrew_inital_eigen);
        unscrew_inital_eigen.translate(Eigen::Vector3d(0, 0, -distance));
        geometry_msgs::Pose unscrew_inital_pose = visual_tools_->convertPose(unscrew_inital_eigen);

        // Set Initial Position
        geometry_msgs::Pose pose = mode == std::string("screw") ? unscrew_inital_pose : input_pose;

        kinematic_state_->setFromIK(joint_model_group_, pose);

        std::vector<double> joint_group_positions;
        kinematic_state_->copyJointGroupPositions(joint_model_group_, joint_group_positions);

        joint_group_positions.back() = mode == std::string("screw") ? minBound : maxBound;
        move_group_interface_->setJointValueTarget(joint_group_positions);

        moveit::planning_interface::MoveGroupInterface::Plan plan_Set_Initial_Position;

        bool success = (move_group_interface_->plan(plan_Set_Initial_Position) == moveit::core::MoveItErrorCode::SUCCESS);
        move_group_interface_->move();

        visual_tools_->prompt("Waiting for feedback! NEXT STEP -> Translate EndEffector");

        // Translate EndEffector
        geometry_msgs::Pose start_end_effector_pose = move_group_interface_->getCurrentPose().pose;
        Eigen::Isometry3d goal_end_effector_eigen;
        geometry_msgs::Pose goal_end_effector_pose = start_end_effector_pose;

        std::vector<geometry_msgs::Pose> waypoints;

        double turn_angle = mode == std::string("screw") ? M_PI_2 : -M_PI_2;
        int distance_ratio = 6;
        Eigen::Vector3d translation(0, 0, mode == std::string("screw") ? distance / distance_ratio : -distance / distance_ratio);

        for (double angle = 0; angle <= 3 * M_PI; angle += abs(turn_angle))
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

        moveit_msgs::RobotTrajectory trajectory;
        const double jump_threshold = 0.0;
        const double eef_step = 0.01;
        double fraction = move_group_interface_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
        ROS_INFO_NAMED("tutorial", "Visualizing Cartesian path (%.2f%% achieved)", fraction * 100.0);

        for (std::size_t i = 0; i < waypoints.size(); ++i)
            visual_tools_->publishAxis(waypoints[i]);
        visual_tools_->trigger();

        move_group_interface_->execute(trajectory);
    }

private:
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
    std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;
    const moveit::core::JointModelGroup *joint_model_group_;
    std::shared_ptr<moveit_visual_tools::MoveItVisualTools> visual_tools_;

    robot_model::RobotModelPtr kinematic_model_;
    robot_state::RobotStatePtr kinematic_state_;
    std::shared_ptr<planning_scene::PlanningScene> planning_scene_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "exagerate_trajectory");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    static const std::string PLANNING_GROUP = "right_arm";
    auto move_group_interface = std::make_shared<moveit::planning_interface::MoveGroupInterface>(PLANNING_GROUP);
    move_group_interface->setPlanningTime(10.0);

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

    HRI_Interface hri_interface(move_group_interface, planning_scene_interface, joint_model_group, visual_tools);

    move_group_interface->setNamedTarget("home");
    move_group_interface->move();

    geometry_msgs::Pose pose = move_group_interface->getCurrentPose().pose;

    int choice;
    string str1 = "screw";
    do
    {
        cout << "MENU:\n"
             << "1. GO TO TARGET\n"
             << "2. EXECUTE SCREW UNSCREW\n"
             << "2. CHANGE MODE SCREW UNSCREW\n"
             << "4. EXIT\n"
             << "Enter your choice: ";
        cin >> choice;

        switch (choice)
        {
        case 1:
            cout << "You chose to GO TO TARGET.\n";
            hri_interface.yes_no();
            break;
        case 2:
            hri_interface.screw_unscrew(pose, str1);
            break;
        case 3:
            cout << "screw or unscrew:";
            cin >> str1;
            break;
        case 4:
            cout << "Exiting the program...\n";
            break;
        default:
            cout << "Invalid choice. Please try again.\n";
            break;
        }
    } while (choice != 4);

    ros::shutdown();
    return 0;
}