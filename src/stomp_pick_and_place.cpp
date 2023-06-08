#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_interface/planning_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

// The circle constant tau = 2*pi. One tau is one rotation in radians.
const double tau = 2 * M_PI;

moveit::planning_interface::MoveGroupInterface::Plan plan_and_execute(moveit::planning_interface::MoveGroupInterface &move_group_interface, moveit_visual_tools::MoveItVisualTools &visual_tools, Eigen::Isometry3d &text_pose, std::string planner, moveit::core::JointModelGroup joint_model_group, geometry_msgs::Pose pose)
{
    moveit::core::RobotState start_state(*move_group_interface.getCurrentState());
    start_state.setFromIK(&joint_model_group, pose);

    std::vector<double> joint_positions;
    start_state.copyJointGroupPositions(&joint_model_group, joint_positions);
    move_group_interface.setJointValueTarget(joint_positions);

    move_group_interface.setPlanningTime(10.0);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    bool success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
    visual_tools.publishAxisLabeled(pose, "end_pose");
    visual_tools.publishText(text_pose, planner, rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
    visual_tools.publishTrajectoryLine(my_plan.trajectory_, &joint_model_group);
    visual_tools.trigger();

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
    collision_objects[1].primitive_poses[0].position.x = 0.4;
    collision_objects[1].primitive_poses[0].position.y = 0;
    collision_objects[1].primitive_poses[0].position.z = 0.25;
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

void exagerateTrajectory(moveit::planning_interface::MoveGroupInterface::Plan my_plan)
{

    // int i = my_plan.trajectory_.joint_trajectory.points.size() / 2;
    // my_plan.trajectory_.joint_trajectory.points[i].positions[0] = 0;
    // my_plan.trajectory_.joint_trajectory.points[i].positions[1] = -3.14;
    // my_plan.trajectory_.joint_trajectory.points[i].positions[2] = 0;
    // my_plan.trajectory_.joint_trajectory.points[i].positions[3] = -1.5707;
    // my_plan.trajectory_.joint_trajectory.points[i].positions[4] = 0;
    // my_plan.trajectory_.joint_trajectory.points[i].positions[5] = 0;

    // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

    // visual_tools.deleteAllMarkers();
    // visual_tools.publishTrajectoryPath(my_plan.trajectory_, my_plan.start_state_, true);
    // visual_tools.trigger();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "planner_testing");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    static const std::string PLANNING_GROUP = "manipulator";
    moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);
    move_group_interface.setPlanningTime(10.0);

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    const moveit::core::JointModelGroup *joint_model_group =
        move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
    visual_tools.deleteAllMarkers();
    visual_tools.loadRemoteControl();

    // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().z() = 0.75;
    visual_tools.publishText(text_pose, "PLANNER TESTING", rvt::WHITE, rvt::XXLARGE);
    visual_tools.trigger();

    geometry_msgs::Pose start_pose;
    start_pose.position.x = 0.374900;
    start_pose.position.y = -0.172585;
    start_pose.position.z = 0.248285;
    start_pose.orientation.w = 1.93647e-5;
    start_pose.orientation.x = 0.000407715;
    start_pose.orientation.y = 1.0;
    start_pose.orientation.z = 0.000809097;

    geometry_msgs::Pose finish_pose;
    finish_pose.position.x = 0.374900;
    finish_pose.position.y = 0.292854;
    finish_pose.position.z = 0.248285;
    finish_pose.orientation.w = 1.93647e-5;
    finish_pose.orientation.x = 0.000407715;
    finish_pose.orientation.y = 1.0;
    finish_pose.orientation.z = 0.000809097;

    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

    // FIRST POSE (START POSE)
    moveit::core::RobotState start_state(*move_group_interface.getCurrentState());
    start_state.setFromIK(joint_model_group, start_pose);

    std::vector<double> joint_positions;
    start_state.copyJointGroupPositions(joint_model_group, joint_positions);
    move_group_interface.setJointValueTarget(joint_positions);

    move_group_interface.setPlanningTime(10.0);

    move_group_interface.move();

    ros::WallDuration(2.0).sleep();

    std::vector<moveit_msgs::CollisionObject> collision_objects = addCollisionObjects(planning_scene_interface, move_group_interface);

    ros::WallDuration(2.0).sleep();

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    my_plan = plan_and_execute(move_group_interface, visual_tools, text_pose, "stomp", *joint_model_group, finish_pose);

    ros::WallDuration(2.0).sleep();

    removeCollisionObstacles(planning_scene_interface, collision_objects);

    ros::WallDuration(2.0).sleep();

    ros::shutdown();
    return 0;
}