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

#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/robot_state/conversions.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <iostream>
#include <string>

#include <memory>

using namespace std;

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
    ros::init(argc, argv, "pick_and_place");
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

    namespace rvt = rviz_visual_tools;
    auto visual_tools = std::make_shared<moveit_visual_tools::MoveItVisualTools>("yumi_base_link");
    visual_tools->deleteAllMarkers();
    visual_tools->loadRemoteControl();

    // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().z() = 0.75;
    visual_tools->publishText(text_pose, "PICK AND PLACE TESTING", rvt::WHITE, rvt::XXLARGE);
    visual_tools->trigger();

    std::map<std::string, moveit_msgs::CollisionObject> objects = planning_scene_interface->getObjects();

    // Find position of object
    moveit_msgs::CollisionObject object = objects["object"];

    addCollisionObjects(*planning_scene_interface);

    int choice;
    bool mode = true;
    string input;
    do
    {
        cout << "MENU:\n"
             << "1. PRINT POSE\n"
             << "2. PICK\n"
             << "3. PLACE\n"
             << "4. PRINT OBJECT POSE\n"
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
            cout << "PICK.\n";
            pick(*left_arm_mgi, *left_gripper_mgi);
            break;
        case 3:
            cout << "PLACE.\n";
            place(*left_arm_mgi, *left_gripper_mgi);
            break;
        case 4:
            cout << "PRINTING POSE.\n";
            objects = planning_scene_interface->getObjects();
            object = objects["object"];
            printPose(object);
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