#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_interface/planning_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/mesh_operations.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <tf2_eigen/tf2_eigen.h>

#include <iostream>
#include <string>

#include <memory>

#include <hri_interface/hri_interface.h>

#include <hri/hri.h>
#include "planner/boardOrientation.h"

using namespace std;

class TaskManager
{
public:
    // Constructor
    TaskManager(ros::NodeHandle n) : n_(n)
    {
        boardOrientationClient = n_.serviceClient<planner::boardOrientation>("board_orientation_srv");

        planning_scene_interface = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();

        both_arms_mgi = std::make_shared<moveit::planning_interface::MoveGroupInterface>("both_arms");
        both_arms_jmg = both_arms_mgi->getCurrentState()->getJointModelGroup("both_arms");

        right_arm_mgi = std::make_shared<moveit::planning_interface::MoveGroupInterface>("right_arm");
        right_arm_jmg = right_arm_mgi->getCurrentState()->getJointModelGroup("right_arm");

        left_arm_mgi = std::make_shared<moveit::planning_interface::MoveGroupInterface>("left_arm");
        left_arm_jmg = left_arm_mgi->getCurrentState()->getJointModelGroup("left_arm");

        right_gripper_mgi = std::make_shared<moveit::planning_interface::MoveGroupInterface>("right_gripper");
        right_gripper_jmg = right_gripper_mgi->getCurrentState()->getJointModelGroup("right_gripper");

        left_gripper_mgi = std::make_shared<moveit::planning_interface::MoveGroupInterface>("left_gripper");
        left_gripper_jmg = left_gripper_mgi->getCurrentState()->getJointModelGroup("left_gripper");

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

        namespace rvt = rviz_visual_tools;
        visual_tools = std::make_shared<moveit_visual_tools::MoveItVisualTools>("yumi_base_link");
        visual_tools->deleteAllMarkers();
        visual_tools->loadRemoteControl();

        addCollisionObjects(*planning_scene_interface);

        right_arm_hri = std::make_shared<HRI_Interface>(n_, right_arm_mgi, right_arm_jmg, right_gripper_mgi, right_gripper_jmg, planning_scene_interface, visual_tools);
        left_arm_hri = std::make_shared<HRI_Interface>(n_, left_arm_mgi, left_arm_jmg, left_gripper_mgi, left_gripper_jmg, planning_scene_interface, visual_tools);
    }

    void main()
    {

        std::map<std::string, moveit_msgs::CollisionObject> objects;

        // Find position of object
        moveit_msgs::CollisionObject object;

        std::string human = "human";

        geometry_msgs::Point point;
        point.x = 0.5;
        point.y = 0.22;
        point.z = 0.02;

        geometry_msgs::Pose left_screw;
        left_screw.position.x = 0.384331;
        left_screw.position.y = -0.0672509;
        left_screw.position.z = 0.193919;
        left_screw.orientation.x = -0.999977;
        left_screw.orientation.y = 0.00573584;
        left_screw.orientation.z = 0.00334705;
        left_screw.orientation.w = 0.00145001;

        geometry_msgs::Pose right_screw;
        right_screw.position.x = 0.385899;
        right_screw.position.y = 0.0698563;
        right_screw.position.z = 0.194551;
        right_screw.orientation.x = -0.999976;
        right_screw.orientation.y = 0.0057798;
        right_screw.orientation.z = 0.00311527;
        right_screw.orientation.w = 0.00223946;

        int choice;
        string input;
        do
        {
            cout << "MENU:\n"
                 << "1. PRINT POSE\n"
                 << "2. PRINT OBJECT POSE\n"
                 << "3. EXECUTE TASK 1\n"
                 << "4. EXECUTE TASK 2\n"
                 << "5. GET BOARD ORIENTATION\n"
                 << "6. New Screw\n"
                 << "7. Working Box Pick and Place\n"
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
                cout << "Object ID: ";
                cin >> input;
                objects = planning_scene_interface->getObjects();
                object = objects[input];
                printPose(object);
                break;
            case 3:
                cout << "EXECUTING TASK 1.\n";
                left_arm_hri->pointToHuman(human);
                visual_tools->prompt("");

                // MOVEMENT A

                left_arm_hri->pointToObject("board_A");

                left_arm_hri->signalPick();

                left_arm_hri->pointToPoint(point);

                left_gripper_mgi->setNamedTarget("open");
                left_gripper_mgi->move();

                visual_tools->prompt("");

                left_arm_mgi->setNamedTarget("side");
                left_arm_mgi->move();

                visual_tools->prompt("Put Board in required position");

                moveObject(*planning_scene_interface, "board_A", point.x, point.y, point.z);
                ros::Duration(2).sleep();

                left_arm_hri->pointToHuman(human);
                visual_tools->prompt("");

                left_arm_hri->signalRotate("board_A", Eigen::Vector3d(1, 0, 0));

                visual_tools->prompt("");

                left_arm_mgi->setNamedTarget("side");
                left_arm_mgi->move();

                visual_tools->prompt("Rotate Board");

                rotateObject(*planning_scene_interface, "board_A", 0, 0, 0);

                // MOVEMENT C

                pick1(*left_arm_mgi, *left_gripper_mgi);

                place1(*left_arm_mgi, *left_gripper_mgi);
                visual_tools->prompt("");

                left_arm_mgi->setNamedTarget("side");
                left_arm_mgi->move();
                visual_tools->prompt("");
                // MOVEMENT D

                right_arm_hri->pointToHuman(human);
                visual_tools->prompt("");

                right_arm_hri->screw_unscrew(true, left_screw);
                visual_tools->prompt("");

                right_arm_hri->screw_unscrew(true, right_screw);
                visual_tools->prompt("");

                right_arm_mgi->setNamedTarget("side");
                right_arm_mgi->move();
                visual_tools->prompt("");

                // MOVEMENT F

                right_arm_hri->pointToHuman(human);
                visual_tools->prompt("");

                right_arm_hri->screw_unscrew(false, left_screw);
                visual_tools->prompt("");

                right_arm_hri->screw_unscrew(false, right_screw);
                visual_tools->prompt("");

                right_arm_mgi->setNamedTarget("side");
                right_arm_mgi->move();
                visual_tools->prompt("");

                // MOVEMENT H

                pickLast(*right_arm_mgi, *right_gripper_mgi);

                placeFaulty(*right_arm_mgi, *right_gripper_mgi);

                right_arm_mgi->setNamedTarget("side");
                right_arm_mgi->move();

                break;

            case 4:
                cout << "EXECUTING TASK 2.\n";
                left_arm_hri->pointToHuman(human);
                ros::Duration(2).sleep();

                visual_tools->prompt("");

                // MOVEMENT A

                left_arm_hri->pointToObject("board_B");
                visual_tools->prompt("");

                left_arm_hri->signalPick();
                visual_tools->prompt("");

                left_arm_hri->pointToObject("test_station");

                left_gripper_mgi->setNamedTarget("open");
                left_gripper_mgi->move();

                visual_tools->prompt("");

                left_arm_mgi->setNamedTarget("side");
                left_arm_mgi->move();

                visual_tools->prompt("Put Board in required position");

                moveObject(*planning_scene_interface, "board_B", 0.38, 0.0, 0.045 + 0.0016 / 2);
                ros::Duration(2).sleep();

                // MOVEMENT C

                right_arm_hri->signalRotate("board_B", Eigen::Vector3d(0, 0, -1));
                ros::Duration(1).sleep();

                right_arm_mgi->setNamedTarget("side");
                right_arm_mgi->move();

                rotateObject(*planning_scene_interface, "board_B", 0, 0, 90 * (M_PI / 180));

                break;

            case 5:
            {
                planner::boardOrientation boardOrientation;
                boardOrientationClient.call(boardOrientation);

                // right_arm_hri->signalRotate("board_B", Eigen::Vector3d(boardOrientation.response.x, boardOrientation.response.y, boardOrientation.response.z));

                break;
            }

            case 6:

                right_arm_hri->pointToHuman(human);
                visual_tools->prompt("");

                right_arm_hri->screw_unscrew(true, left_screw);
                visual_tools->prompt("");

                right_arm_hri->screw_unscrew(true, right_screw);
                visual_tools->prompt("");

                right_arm_mgi->setNamedTarget("side");
                right_arm_mgi->move();
                visual_tools->prompt("");

                // MOVEMENT F

                right_arm_hri->pointToHuman(human);
                visual_tools->prompt("");

                right_arm_hri->screw_unscrew(false, left_screw);
                visual_tools->prompt("");

                right_arm_hri->screw_unscrew(false, right_screw);
                visual_tools->prompt("");

                right_arm_mgi->setNamedTarget("side");
                right_arm_mgi->move();
                visual_tools->prompt("");
                break;

            case 7:
                rotateObject(*planning_scene_interface, "board_A", 0, 0, 0);
                moveObject(*planning_scene_interface, "board_A", 0.378252, 0.00381671, 0.0379046);
                pickLast(*left_arm_mgi, *left_gripper_mgi);
                placeWorking(*left_arm_mgi, *left_gripper_mgi);

            case 0:
                cout << "Exiting the program...\n";
                break;
            default:
                cout << "Invalid choice. Please try again.\n";
                break;
            }
        } while (choice != 0);
    }

private:
    ros::NodeHandle n_;

    ros::ServiceClient boardOrientationClient;

    std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface;

    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> both_arms_mgi;
    const moveit::core::JointModelGroup *both_arms_jmg;

    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> right_arm_mgi;
    const moveit::core::JointModelGroup *right_arm_jmg;

    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> left_arm_mgi;
    const moveit::core::JointModelGroup *left_arm_jmg;

    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> right_gripper_mgi;
    const moveit::core::JointModelGroup *right_gripper_jmg;

    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> left_gripper_mgi;
    const moveit::core::JointModelGroup *left_gripper_jmg;

    std::shared_ptr<moveit_visual_tools::MoveItVisualTools> visual_tools;
    Eigen::Isometry3d text_pose;

    std::shared_ptr<HRI_Interface> right_arm_hri;

    std::shared_ptr<HRI_Interface> left_arm_hri;

    void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface &planning_scene_interface)
    {

        std::vector<moveit_msgs::CollisionObject> collision_objects;
        std::vector<moveit_msgs::ObjectColor> object_colors;

        collision_objects.resize(6);
        object_colors.resize(6);

        int i = 0;

        // collision_objects[i].id = "board_A";
        // collision_objects[i].header.frame_id = "yumi_base_link";

        // collision_objects[i].primitives.resize(1);
        // collision_objects[i].primitives[0].type = collision_objects[0].primitives[0].BOX;
        // collision_objects[i].primitives[0].dimensions.resize(3);
        // // collision_objects[i].primitives[0].dimensions[0] = 0.05334;
        // // collision_objects[i].primitives[0].dimensions[1] = 0.06858;
        // // collision_objects[i].primitives[0].dimensions[2] = 0.0016;
        // collision_objects[i].primitives[0].dimensions[0] = 0.04;
        // collision_objects[i].primitives[0].dimensions[1] = 0.06;
        // collision_objects[i].primitives[0].dimensions[2] = 0.0016;

        // collision_objects[i].primitive_poses.resize(1);
        // collision_objects[i].primitive_poses[0].position.x = 0.36;
        // collision_objects[i].primitive_poses[0].position.y = 0.80;
        // collision_objects[i].primitive_poses[0].position.z = 0.015 + 0.0016 / 2;
        // collision_objects[i].primitive_poses[0].orientation.w = 1.0;

        // collision_objects[i].operation = collision_objects[i].ADD;

        // object_colors[i].id = collision_objects[i].id;

        // std_msgs::ColorRGBA boardA_color;
        // boardA_color.r = 195 / 255.0;
        // boardA_color.g = 247 / 255.0;
        // boardA_color.b = 200 / 255.0;
        // boardA_color.a = 1.0;

        // object_colors[i].color = boardA_color;

        // i++

        // collision_objects[i].id = "board_B";
        //  collision_objects[i].header.frame_id = "yumi_base_link";

        // collision_objects[i].primitives.resize(1);
        // collision_objects[i].primitives[0].type = collision_objects[i].primitives[0].BOX;
        // collision_objects[i].primitives[0].dimensions.resize(3);
        // collision_objects[i].primitives[0].dimensions[0] = 0.05334;
        // collision_objects[i].primitives[0].dimensions[1] = 0.06858;
        // collision_objects[i].primitives[0].dimensions[2] = 0.0016;
        // // collision_objects[i].primitives[0].dimensions[0] = 0.04;
        // // collision_objects[i].primitives[0].dimensions[1] = 0.06;
        // // collision_objects[i].primitives[0].dimensions[2] = 0.0016;

        // collision_objects[i].primitive_poses.resize(1);
        // collision_objects[i].primitive_poses[0].position.x = 0.56;
        // collision_objects[i].primitive_poses[0].position.y = 0.80;
        // collision_objects[i].primitive_poses[0].position.z = 0.015 + 0.0016 / 2;
        // collision_objects[i].primitive_poses[0].orientation.w = 1.0;

        // collision_objects[i].operation = collision_objects[i].ADD;

        // object_colors[i].id = collision_objects[i].id;

        // std_msgs::ColorRGBA boardB_color;
        // boardB_color.r = 255 / 255.0;
        // boardB_color.g = 227 / 255.0;
        // boardB_color.b = 66 / 255.0;
        // boardB_color.a = 1.0;

        // object_colors[i].color = boardB_color;

        // i++;

        shapes::Mesh *board_m = shapes::createMeshFromResource("package://planner/objects/board.stl");
        board_m->scale(1);

        shape_msgs::Mesh board_object_mesh;
        shapes::ShapeMsg board_object_mesh_msg;
        shapes::constructMsgFromShape(board_m, board_object_mesh_msg);
        board_object_mesh = boost::get<shape_msgs::Mesh>(board_object_mesh_msg);

        collision_objects[i].id = "board_A";
        collision_objects[i].header.frame_id = "yumi_base_link";

        collision_objects[i].pose.position.x = 0.36;
        collision_objects[i].pose.position.y = 0.80;
        collision_objects[i].pose.position.z = 0.015 + 0.0016 / 2;
        collision_objects[i].pose.orientation.x = 1;
        collision_objects[i].pose.orientation.y = 0;
        collision_objects[i].pose.orientation.z = 0;
        collision_objects[i].pose.orientation.w = 0.000796327;

        collision_objects[i].meshes.resize(1);
        collision_objects[i].meshes[0] = board_object_mesh;

        collision_objects[i].operation = collision_objects[i].ADD;

        std_msgs::ColorRGBA boardA_color;
        boardA_color.r = 195 / 255.0;
        boardA_color.g = 247 / 255.0;
        boardA_color.b = 200 / 255.0;
        boardA_color.a = 1.0;

        object_colors[i].id = collision_objects[i].id;
        object_colors[i].color = boardA_color;

        i++;

        collision_objects[i].id = "board_B";
        collision_objects[i].header.frame_id = "yumi_base_link";

        collision_objects[i].pose.position.x = 0.56;
        collision_objects[i].pose.position.y = 0.80;
        collision_objects[i].pose.position.z = 0.015 + 0.0016 / 2;
        collision_objects[i].pose.orientation.x = 1;
        collision_objects[i].pose.orientation.y = 0;
        collision_objects[i].pose.orientation.z = 0;
        collision_objects[i].pose.orientation.w = 0.000796327;

        collision_objects[i].meshes.resize(1);
        collision_objects[i].meshes[0] = board_object_mesh;

        collision_objects[i].operation = collision_objects[i].ADD;

        std_msgs::ColorRGBA boardB_color;
        boardB_color.r = 255 / 255.0;
        boardB_color.g = 227 / 255.0;
        boardB_color.b = 66 / 255.0;
        boardB_color.a = 1.0;

        object_colors[i].id = collision_objects[i].id;
        object_colors[i].color = boardB_color;

        i++;

        collision_objects[i].id = "table";
        collision_objects[i].header.frame_id = "yumi_base_link";

        collision_objects[i].primitives.resize(1);
        collision_objects[i].primitives[0].type = collision_objects[i].primitives[0].BOX;
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
        table_color.r = 207 / 255.0;
        table_color.g = 168 / 255.0;
        table_color.b = 114 / 255.0;
        table_color.a = 1.0;

        object_colors[i].color = table_color;

        i++;

        // collision_objects[i].id = "test_station";
        // collision_objects[i].header.frame_id = "yumi_base_link";

        // collision_objects[i].pose.position.x = 0.38;
        // collision_objects[i].pose.position.y = 0.00;
        // collision_objects[i].pose.position.z = 0.03;
        // collision_objects[i].pose.orientation.w = 1.0;

        // collision_objects[i].primitives.resize(4);
        // collision_objects[i].primitive_poses.resize(4);

        // collision_objects[i].primitives[0].type = collision_objects[i].primitives[0].BOX;
        // collision_objects[i].primitives[0].dimensions.resize(3);
        // collision_objects[i].primitives[0].dimensions[0] = 0.02;
        // collision_objects[i].primitives[0].dimensions[1] = 0.04;
        // collision_objects[i].primitives[0].dimensions[2] = 0.03;

        // collision_objects[i].primitive_poses[0].position.x = 0.0;
        // collision_objects[i].primitive_poses[0].position.y = 0.0;
        // collision_objects[i].primitive_poses[0].position.z = 0.0;
        // collision_objects[i].primitive_poses[0].orientation.w = 1.0;

        // collision_objects[i].primitives[1].type = collision_objects[i].primitives[1].BOX;
        // collision_objects[i].primitives[1].dimensions.resize(3);
        // collision_objects[i].primitives[1].dimensions[0] = 0.02;
        // collision_objects[i].primitives[1].dimensions[1] = 0.10;
        // collision_objects[i].primitives[1].dimensions[2] = 0.01;

        // collision_objects[i].primitive_poses[1].position.x = 0.0;
        // collision_objects[i].primitive_poses[1].position.y = 0.0;
        // collision_objects[i].primitive_poses[1].position.z = -0.01;
        // collision_objects[i].primitive_poses[1].orientation.w = 1.0;

        // collision_objects[i].primitives[2].type = collision_objects[i].primitives[2].CYLINDER;
        // collision_objects[i].primitives[2].dimensions.resize(2);
        // collision_objects[i].primitives[2].dimensions[0] = 0.04;
        // collision_objects[i].primitives[2].dimensions[1] = 0.005;

        // collision_objects[i].primitive_poses[2].position.x = 0.00;
        // collision_objects[i].primitive_poses[2].position.y = -0.04;
        // collision_objects[i].primitive_poses[2].position.z = 0.00;
        // collision_objects[i].primitive_poses[2].orientation.w = 1.0;

        // collision_objects[i].primitives[3].type = collision_objects[i].primitives[3].CYLINDER;
        // collision_objects[i].primitives[3].dimensions.resize(2);
        // collision_objects[i].primitives[3].dimensions[0] = 0.04;
        // collision_objects[i].primitives[3].dimensions[1] = 0.005;

        // collision_objects[i].primitive_poses[3].position.x = 0.00;
        // collision_objects[i].primitive_poses[3].position.y = 0.04;
        // collision_objects[i].primitive_poses[3].position.z = 0.00;
        // collision_objects[i].primitive_poses[3].orientation.w = 1.0;

        // collision_objects[i].operation = collision_objects[i].ADD;

        // std_msgs::ColorRGBA test_station_color;
        // test_station_color.r = 255 / 255;
        // test_station_color.g = 255 / 255;
        // test_station_color.b = 255 / 255;
        // test_station_color.a = 1.0;

        // object_colors[i].id = collision_objects[i].id;
        // object_colors[i].color = test_station_color;

        collision_objects[i].id = "test_station";
        collision_objects[i].header.frame_id = "yumi_base_link";

        collision_objects[i].pose.position.x = 0.38;
        collision_objects[i].pose.position.y = 0.00;
        collision_objects[i].pose.position.z = 0.015;
        collision_objects[i].pose.orientation.w = 1.0;

        shapes::Mesh *m = shapes::createMeshFromResource("package://planner/objects/test_station_fix.stl");
        m->scale(1);

        shape_msgs::Mesh object_mesh;
        shapes::ShapeMsg object_mesh_msg;
        shapes::constructMsgFromShape(m, object_mesh_msg);
        object_mesh = boost::get<shape_msgs::Mesh>(object_mesh_msg);

        collision_objects[i].meshes.resize(1);
        collision_objects[i].meshes[0] = object_mesh;

        collision_objects[i].operation = collision_objects[i].ADD;

        std_msgs::ColorRGBA test_station_color;
        test_station_color.r = 201 / 255.0;
        test_station_color.g = 81 / 255.0;
        test_station_color.b = 81 / 255.0;
        test_station_color.a = 1.0;

        object_colors[i].id = collision_objects[i].id;
        object_colors[i].color = test_station_color;

        i++;

        collision_objects[i].id = "faulty_box";
        collision_objects[i].header.frame_id = "yumi_base_link";

        collision_objects[i].pose.position.x = 0.26;
        collision_objects[i].pose.position.y = -0.42;
        collision_objects[i].pose.position.z = 0.02;
        collision_objects[i].pose.orientation.w = 1.0;

        collision_objects[i].primitives.resize(5);
        collision_objects[i].primitive_poses.resize(5);

        // Floor
        collision_objects[i].primitives[0].type = collision_objects[i].primitives[0].BOX;
        collision_objects[i].primitives[0].dimensions.resize(3);
        collision_objects[i].primitives[0].dimensions[0] = 0.20;
        collision_objects[i].primitives[0].dimensions[1] = 0.20;
        collision_objects[i].primitives[0].dimensions[2] = 0.005;

        collision_objects[i].primitive_poses[0].position.x = 0.0;
        collision_objects[i].primitive_poses[0].position.y = 0.0;
        collision_objects[i].primitive_poses[0].position.z = 0.0;
        collision_objects[i].primitive_poses[0].orientation.w = 1.0;

        // wall 1
        collision_objects[i].primitives[1].type = collision_objects[i].primitives[1].BOX;
        collision_objects[i].primitives[1].dimensions.resize(3);
        collision_objects[i].primitives[1].dimensions[0] = 0.20;
        collision_objects[i].primitives[1].dimensions[1] = 0.01;
        collision_objects[i].primitives[1].dimensions[2] = 0.10;

        collision_objects[i].primitive_poses[1].position.x = 0.0;
        collision_objects[i].primitive_poses[1].position.y = 0.10;
        collision_objects[i].primitive_poses[1].position.z = 0.05;
        collision_objects[i].primitive_poses[1].orientation.w = 1.0;

        // wall 2
        collision_objects[i].primitives[2].type = collision_objects[i].primitives[2].BOX;
        collision_objects[i].primitives[2].dimensions.resize(3);
        collision_objects[i].primitives[2].dimensions[0] = 0.20;
        collision_objects[i].primitives[2].dimensions[1] = 0.01;
        collision_objects[i].primitives[2].dimensions[2] = 0.10;

        collision_objects[i].primitive_poses[2].position.x = 0.0;
        collision_objects[i].primitive_poses[2].position.y = -0.10;
        collision_objects[i].primitive_poses[2].position.z = 0.05;
        collision_objects[i].primitive_poses[2].orientation.w = 1.0;

        // wall 3
        collision_objects[i].primitives[3].type = collision_objects[i].primitives[3].BOX;
        collision_objects[i].primitives[3].dimensions.resize(3);
        collision_objects[i].primitives[3].dimensions[0] = 0.01;
        collision_objects[i].primitives[3].dimensions[1] = 0.20;
        collision_objects[i].primitives[3].dimensions[2] = 0.10;

        collision_objects[i].primitive_poses[3].position.x = 0.10;
        collision_objects[i].primitive_poses[3].position.y = 0.0;
        collision_objects[i].primitive_poses[3].position.z = 0.05;
        collision_objects[i].primitive_poses[3].orientation.w = 1.0;

        // wall 4
        collision_objects[i].primitives[4].type = collision_objects[i].primitives[4].BOX;
        collision_objects[i].primitives[4].dimensions.resize(3);
        collision_objects[i].primitives[4].dimensions[0] = 0.01;
        collision_objects[i].primitives[4].dimensions[1] = 0.20;
        collision_objects[i].primitives[4].dimensions[2] = 0.10;

        collision_objects[i].primitive_poses[4].position.x = -0.10;
        collision_objects[i].primitive_poses[4].position.y = 0.0;
        collision_objects[i].primitive_poses[4].position.z = 0.05;
        collision_objects[i].primitive_poses[4].orientation.w = 1.0;

        collision_objects[i].operation = collision_objects[i].ADD;

        std_msgs::ColorRGBA box_color;
        box_color.r = 255 / 255.0;
        box_color.g = 255 / 255.0;
        box_color.b = 255 / 255.0;
        box_color.a = 1.0;

        object_colors[i].id = collision_objects[i].id;
        object_colors[i].color = box_color;

        i++;

        collision_objects[i].id = "working_box";
        collision_objects[i].header.frame_id = "yumi_base_link";

        collision_objects[i].pose.position.x = 0.26;
        collision_objects[i].pose.position.y = 0.42;
        collision_objects[i].pose.position.z = 0.02;
        collision_objects[i].pose.orientation.w = 1.0;

        collision_objects[i].primitives.resize(5);
        collision_objects[i].primitive_poses.resize(5);

        // Floor
        collision_objects[i].primitives[0].type = collision_objects[i].primitives[0].BOX;
        collision_objects[i].primitives[0].dimensions.resize(3);
        collision_objects[i].primitives[0].dimensions[0] = 0.20;
        collision_objects[i].primitives[0].dimensions[1] = 0.20;
        collision_objects[i].primitives[0].dimensions[2] = 0.005;

        collision_objects[i].primitive_poses[0].position.x = 0.0;
        collision_objects[i].primitive_poses[0].position.y = 0.0;
        collision_objects[i].primitive_poses[0].position.z = 0.0;
        collision_objects[i].primitive_poses[0].orientation.w = 1.0;

        // wall 1
        collision_objects[i].primitives[1].type = collision_objects[i].primitives[1].BOX;
        collision_objects[i].primitives[1].dimensions.resize(3);
        collision_objects[i].primitives[1].dimensions[0] = 0.20;
        collision_objects[i].primitives[1].dimensions[1] = 0.01;
        collision_objects[i].primitives[1].dimensions[2] = 0.10;

        collision_objects[i].primitive_poses[1].position.x = 0.0;
        collision_objects[i].primitive_poses[1].position.y = 0.10;
        collision_objects[i].primitive_poses[1].position.z = 0.05;
        collision_objects[i].primitive_poses[1].orientation.w = 1.0;

        // wall 2
        collision_objects[i].primitives[2].type = collision_objects[i].primitives[2].BOX;
        collision_objects[i].primitives[2].dimensions.resize(3);
        collision_objects[i].primitives[2].dimensions[0] = 0.20;
        collision_objects[i].primitives[2].dimensions[1] = 0.01;
        collision_objects[i].primitives[2].dimensions[2] = 0.10;

        collision_objects[i].primitive_poses[2].position.x = 0.0;
        collision_objects[i].primitive_poses[2].position.y = -0.10;
        collision_objects[i].primitive_poses[2].position.z = 0.05;
        collision_objects[i].primitive_poses[2].orientation.w = 1.0;

        // wall 3
        collision_objects[i].primitives[3].type = collision_objects[i].primitives[3].BOX;
        collision_objects[i].primitives[3].dimensions.resize(3);
        collision_objects[i].primitives[3].dimensions[0] = 0.01;
        collision_objects[i].primitives[3].dimensions[1] = 0.20;
        collision_objects[i].primitives[3].dimensions[2] = 0.10;

        collision_objects[i].primitive_poses[3].position.x = 0.10;
        collision_objects[i].primitive_poses[3].position.y = 0.0;
        collision_objects[i].primitive_poses[3].position.z = 0.05;
        collision_objects[i].primitive_poses[3].orientation.w = 1.0;

        // wall 4
        collision_objects[i].primitives[4].type = collision_objects[i].primitives[4].BOX;
        collision_objects[i].primitives[4].dimensions.resize(3);
        collision_objects[i].primitives[4].dimensions[0] = 0.01;
        collision_objects[i].primitives[4].dimensions[1] = 0.20;
        collision_objects[i].primitives[4].dimensions[2] = 0.10;

        collision_objects[i].primitive_poses[4].position.x = -0.10;
        collision_objects[i].primitive_poses[4].position.y = 0.0;
        collision_objects[i].primitive_poses[4].position.z = 0.05;
        collision_objects[i].primitive_poses[4].orientation.w = 1.0;

        collision_objects[i].operation = collision_objects[i].ADD;

        object_colors[i].id = collision_objects[i].id;
        object_colors[i].color = box_color;

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

    void rotateObject(moveit::planning_interface::PlanningSceneInterface &planning_scene_interface, std::string object_id, double roll = 0, double pitch = 0, double yaw = 0)
    {

        std::map<std::string, moveit_msgs::CollisionObject> objects = planning_scene_interface.getObjects();

        moveit_msgs::CollisionObject object = objects[object_id];

        tf2::Quaternion q;
        q.setRPY(roll, pitch, yaw);
        geometry_msgs::Quaternion q_msg;
        tf2::convert(q, q_msg);

        object.pose.orientation = q_msg;

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
        grasps[0].grasp_pose.pose.position.x = 0.51;
        grasps[0].grasp_pose.pose.position.y = 0.221018;
        grasps[0].grasp_pose.pose.position.z = 0.161538;
        grasps[0].grasp_pose.pose.orientation.x = -0.71394;
        grasps[0].grasp_pose.pose.orientation.y = -0.70018;
        grasps[0].grasp_pose.pose.orientation.z = 0.00444485;
        grasps[0].grasp_pose.pose.orientation.w = 0.00420897;

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
        place_location[0].place_pose.pose.position.x = 0.378252;
        place_location[0].place_pose.pose.position.y = 0.00381671;
        place_location[0].place_pose.pose.position.z = 0.0379046;
        place_location[0].place_pose.pose.orientation.x = -7.79512e-05;
        place_location[0].place_pose.pose.orientation.y = -5.27803e-05;
        place_location[0].place_pose.pose.orientation.z = 4.15536e-05;
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

    void pickLast(moveit::planning_interface::MoveGroupInterface &arm_mgi, moveit::planning_interface::MoveGroupInterface &gripper_mgi)
    {
        // Create a vector of grasps to be attempted, currently only creating single grasp.
        // This is essentially useful when using a grasp generator to generate and test multiple grasps.
        std::vector<moveit_msgs::Grasp> grasps;
        grasps.resize(1);

        // Actual Grasping Pose
        grasps[0].grasp_pose.header.frame_id = "yumi_base_link";
        grasps[0].grasp_pose.pose.position.x = 0.399909;
        grasps[0].grasp_pose.pose.position.y = 0.00346781;
        grasps[0].grasp_pose.pose.position.z = 0.180667;
        grasps[0].grasp_pose.pose.orientation.x = -0.728023;
        grasps[0].grasp_pose.pose.orientation.y = -0.685522;
        grasps[0].grasp_pose.pose.orientation.z = 0.00438482;
        grasps[0].grasp_pose.pose.orientation.w = 0.00476908;

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

        arm_mgi.setSupportSurfaceName("test_station");
        arm_mgi.pick("board_A", grasps);
    }

    void placeFaulty(moveit::planning_interface::MoveGroupInterface &arm_mgi, moveit::planning_interface::MoveGroupInterface &gripper_mgi)
    {
        std::vector<moveit_msgs::PlaceLocation> place_location;
        place_location.resize(1);

        // Actual Placing Pose (this is the object pose and not the gripper pose)
        place_location[0].place_pose.header.frame_id = "yumi_base_link";
        place_location[0].place_pose.pose.position.x = 0.26;
        place_location[0].place_pose.pose.position.y = -0.42;
        place_location[0].place_pose.pose.position.z = 0.05;
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

        arm_mgi.setSupportSurfaceName("faulty_box");
        arm_mgi.place("board_A", place_location);
    }

    void placeWorking(moveit::planning_interface::MoveGroupInterface &arm_mgi, moveit::planning_interface::MoveGroupInterface &gripper_mgi)
    {
        std::vector<moveit_msgs::PlaceLocation> place_location;
        place_location.resize(1);

        // Actual Placing Pose (this is the object pose and not the gripper pose)
        place_location[0].place_pose.header.frame_id = "yumi_base_link";
        place_location[0].place_pose.pose.position.x = 0.26;
        place_location[0].place_pose.pose.position.y = 0.42;
        place_location[0].place_pose.pose.position.z = 0.05;
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

        arm_mgi.setSupportSurfaceName("working_box");
        arm_mgi.place("board_A", place_location);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hri_interface");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    TaskManager taskmanager = TaskManager(node_handle);

    taskmanager.main();

    ros::shutdown();
    return 0;
}