#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_interface/planning_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <iostream>
#include <string>
using namespace std;

// The circle constant tau = 2*pi. One tau is one rotation in radians.
const double tau = 2 * M_PI;

moveit::planning_interface::MoveGroupInterface::Plan plan_and_execute(moveit::planning_interface::MoveGroupInterface &move_group_interface, moveit_visual_tools::MoveItVisualTools &visual_tools, Eigen::Isometry3d &text_pose, std::string planner, moveit::core::JointModelGroup joint_model_group, geometry_msgs::Pose pose)
{
    moveit::core::RobotState start_state(*move_group_interface.getCurrentState());
    start_state.setFromIK(&joint_model_group, pose);

    std::vector<double> joint_positions;
    start_state.copyJointGroupPositions(&joint_model_group, joint_positions);
    move_group_interface.setJointValueTarget(joint_positions);

    move_group_interface.setPlanningTime(45.0);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    bool success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    return my_plan;
}

std::vector<moveit_msgs::CollisionObject> addCollisionObjects(moveit::planning_interface::PlanningSceneInterface &planning_scene_interface, moveit::planning_interface::MoveGroupInterface &move_group_interface)
{
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.resize(2);

    // Add the first table where the cube will originally be kept.
    collision_objects[0].id = "ground";
    collision_objects[0].header.frame_id = move_group_interface.getPlanningFrame();

    /* Define the primitive and its dimensions. */
    collision_objects[0].primitives.resize(1);
    collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
    collision_objects[0].primitives[0].dimensions.resize(3);
    collision_objects[0].primitives[0].dimensions[0] = 5;
    collision_objects[0].primitives[0].dimensions[1] = 5;
    collision_objects[0].primitives[0].dimensions[2] = 0.01;

    /* Define the pose of the table. */
    collision_objects[0].primitive_poses.resize(1);
    collision_objects[0].primitive_poses[0].position.x = 0;
    collision_objects[0].primitive_poses[0].position.y = 0;
    collision_objects[0].primitive_poses[0].position.z = -0.1;
    collision_objects[0].primitive_poses[0].orientation.x = 0;
    collision_objects[0].primitive_poses[0].orientation.y = 0;
    collision_objects[0].primitive_poses[0].orientation.z = 0;
    collision_objects[0].primitive_poses[0].orientation.w = 1.0;

    collision_objects[0].operation = collision_objects[0].ADD;

    collision_objects[1].id = "box";
    collision_objects[1].header.frame_id = move_group_interface.getPlanningFrame();

    /* Define the primitive and its dimensions. */
    collision_objects[1].primitives.resize(1);
    collision_objects[1].primitives[0].type = collision_objects[1].primitives[0].BOX;
    collision_objects[1].primitives[0].dimensions.resize(3);
    collision_objects[1].primitives[0].dimensions[0] = 0.05;
    collision_objects[1].primitives[0].dimensions[1] = 0.05;
    collision_objects[1].primitives[0].dimensions[2] = 0.25;

    /* Define the pose of the table. */
    collision_objects[1].primitive_poses.resize(1);
    collision_objects[1].primitive_poses[0].position.x = 0.35;
    collision_objects[1].primitive_poses[0].position.y = -0.11;
    collision_objects[1].primitive_poses[0].position.z = 0.08;
    collision_objects[1].primitive_poses[0].orientation.x = 0;
    collision_objects[1].primitive_poses[0].orientation.y = 0;
    collision_objects[1].primitive_poses[0].orientation.z = 0;
    collision_objects[1].primitive_poses[0].orientation.w = 1.0;

    collision_objects[1].operation = collision_objects[1].ADD;

    planning_scene_interface.applyCollisionObjects(collision_objects);

    return collision_objects;
}

void removeCollisionObstacles(moveit::planning_interface::PlanningSceneInterface &planning_scene_interface, std::vector<moveit_msgs::CollisionObject> collision_objects)
{

    std::vector<std::string> ids;
    for (const auto &collision_obj : collision_objects)
    {
        ids.push_back(collision_obj.id);
    }
    planning_scene_interface.removeCollisionObjects(ids);
}

moveit::planning_interface::MoveGroupInterface::Plan exagerateTrajectory(moveit_visual_tools::MoveItVisualTools &visual_tools, moveit::planning_interface::MoveGroupInterface::Plan plan, std::string planning_group, Eigen::Vector3d xyz)
{

    // REFERENCE: https://github.com/ros-planning/moveit/issues/1556

    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
    planning_scene::PlanningScene planning_scene(kinematic_model);
    ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());

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

            visual_tools.publishAxisLabeled(target_pose, "Pose");
            visual_tools.trigger();

            kinematic_state->setJointGroupPositions(kinematic_model->getJointModelGroup(planning_group), previous_joint_values);
            success_IK = kinematic_state->setFromIK(kinematic_model->getJointModelGroup(planning_group), target_pose);

            std::stringstream tmp;
            tmp << i;
            char const *iteration = tmp.str().c_str();
            ROS_INFO_NAMED("First Loop", "Iteration %s", success_IK ? iteration : "FAILED");

            visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

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

    visual_tools.deleteAllMarkers();

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

        visual_tools.publishAxisLabeled(target_pose, "Pose");
        visual_tools.trigger();

        kinematic_state->setJointGroupPositions(kinematic_model->getJointModelGroup(planning_group), previous_joint_values);
        success_IK = kinematic_state->setFromIK(kinematic_model->getJointModelGroup(planning_group), target_pose);

        std::stringstream tmp;
        tmp << i;
        char const *iteration = tmp.str().c_str();
        ROS_INFO_NAMED("tutorial", "Iteration %s", success_IK ? iteration : "FAILED");

        visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

        kinematic_state->copyJointGroupPositions(kinematic_model->getJointModelGroup(planning_group), plan.trajectory_.joint_trajectory.points.at(i).positions);

        previous_joint_values = plan.trajectory_.joint_trajectory.points.at(i).positions;
    }

    bool success = planning_scene.isPathValid(plan.start_state_, plan.trajectory_);

    return plan;
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
    visual_tools.publishText(text_pose, "Exagerate Trajectory Testing", rvt::WHITE, rvt::XXLARGE);
    visual_tools.trigger();

    geometry_msgs::Pose start_pose;
    start_pose.position.x = 0.082453;
    start_pose.position.y = -0.24935;
    start_pose.position.z = 0.14972;

    start_pose.orientation.x = 0.571743;
    start_pose.orientation.y = -0.518936;
    start_pose.orientation.z = -0.488325;
    start_pose.orientation.w = -0.406637;

    geometry_msgs::Pose finish_pose;
    finish_pose.position.x = 0.51591;
    finish_pose.position.y = -0.1922;
    finish_pose.position.z = 0.13138;

    finish_pose.orientation.x = 0.571663;
    finish_pose.orientation.y = -0.519008;
    finish_pose.orientation.z = -0.488313;
    finish_pose.orientation.w = -0.406672;

    // // FIRST POSE (START POSE)
    // moveit::core::RobotState start_state(*move_group_interface.getCurrentState());
    // bool success = start_state.setFromIK(joint_model_group, start_pose);

    // std::vector<double> joint_positions;
    // start_state.copyJointGroupPositions(joint_model_group, joint_positions);
    // move_group_interface.setJointValueTarget(joint_positions);

    // move_group_interface.setPlanningTime(45.0);

    // move_group_interface.move();

    // ros::WallDuration(2.0).sleep();

    // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

    // std::vector<moveit_msgs::CollisionObject> collision_objects = addCollisionObjects(planning_scene_interface, move_group_interface);

    move_group_interface.setNamedTarget("home");
    move_group_interface.move();

    ros::WallDuration(2.0).sleep();

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    my_plan = plan_and_execute(move_group_interface, visual_tools, text_pose, "stomp", *joint_model_group, finish_pose);

    ros::WallDuration(2.0).sleep();

    // Modify Trajectory
    Eigen::Vector3d xyz = Eigen::Vector3d(0.0, 0.0, 0.0);

    moveit::planning_interface::MoveGroupInterface::Plan modified_plan = exagerateTrajectory(visual_tools, my_plan, "right_arm", xyz);

    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

    int choice;
    string str1, str2, str3;
    do
    {
        cout << "MENU:\n"
             << "1. Display ORIGINAL plan\n"
             << "2. Display MODIFIED plan\n"
             << "3. Replan\n"
             << "4. Exagerate plan\n"
             << "5. Change values\n"
             << "6. EXIT\n"
             << "Enter your choice: ";
        cin >> choice;

        switch (choice)
        {
        case 1:
            cout << "You chose to display the ORIGINAL plan.\n";
            visual_tools.deleteAllMarkers();
            visual_tools.publishTrajectoryPath(my_plan.trajectory_, my_plan.start_state_, true);
            visual_tools.trigger();
            break;
        case 2:
            cout << "You chose to display the MODIFIED plan.\n";
            visual_tools.deleteAllMarkers();
            visual_tools.publishTrajectoryPath(modified_plan.trajectory_, modified_plan.start_state_, true);
            visual_tools.trigger();
            break;
        case 3:
            cout << "You chose to display the REPLAN.\n";
            my_plan = plan_and_execute(move_group_interface, visual_tools, text_pose, "stomp", *joint_model_group, finish_pose);
            break;
        case 4:
            cout << "You chose to EXAGERATE plan.\n";
            modified_plan = exagerateTrajectory(visual_tools, my_plan, "right_arm", xyz);
            break;
        case 5:
            cout << "X Y Z:";
            cin >> str1 >> str2 >> str3;
            if (str1 != "_")
                xyz[0] = stod(str1);
            if (str2 != "_")
                xyz[1] = stod(str2);
            if (str3 != "_")
                xyz[2] = stod(str3);
            cout << "Entered values: " << xyz.transpose() << endl;
            break;
        case 6:
            cout << "Exiting the program...\n";
            break;
        default:
            cout << "Invalid choice. Please try again.\n";
            break;
        }
    } while (choice != 6);

    // removeCollisionObstacles(planning_scene_interface, collision_objects);

    ros::shutdown();
    return 0;
}