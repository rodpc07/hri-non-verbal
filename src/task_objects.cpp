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
        planning_scene_interface = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();

        addCollisionObjects(*planning_scene_interface);
    }

private:
    ros::NodeHandle n_;

    std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface;

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
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hri_interface");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    TaskManager taskmanager = TaskManager(node_handle);

    ros::shutdown();
    return 0;
}