#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include "planner/TransformListener.h"
#include <iostream>
#include <vector>
#include <string>

bool transformServiceCallback(planner::TransformListener::Request &req,
                              planner::TransformListener::Response &res)
{
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    ros::Rate rate(15.0);

    ROS_INFO("Target Frame: %s\n", req.target_frame.c_str());
    ROS_INFO("Source Frame: %s\n", req.source_frame.c_str());

    ros::Time startTime = ros::Time::now();

    while (ros::Time::now() - startTime <= ros::Duration(5.0))
    {
        try
        {
            res.transformStamped = tfBuffer.lookupTransform(req.target_frame, req.source_frame, ros::Time(0));

            if (res.transformStamped.header.stamp.isValid())
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

    // ROS_INFO("TransformStamped:\n");
    // ROS_INFO("Header:\n");
    // ROS_INFO("  frame_id: %s", res.transformStamped.header.frame_id.c_str());
    // ROS_INFO("  stamp: %f", res.transformStamped.header.stamp.toSec());
    // ROS_INFO("Child Frame ID: %s", res.transformStamped.child_frame_id.c_str());
    // ROS_INFO("Transform:\n");
    // ROS_INFO("  Translation:\n");
    // ROS_INFO("    x: %f", res.transformStamped.transform.translation.x);
    // ROS_INFO("    y: %f", res.transformStamped.transform.translation.y);
    // ROS_INFO("    z: %f", res.transformStamped.transform.translation.z);
    // ROS_INFO("  Rotation:\n");
    // ROS_INFO("    x: %f", res.transformStamped.transform.rotation.x);
    // ROS_INFO("    y: %f", res.transformStamped.transform.rotation.y);
    // ROS_INFO("    z: %f", res.transformStamped.transform.rotation.z);
    // ROS_INFO("    w: %f", res.transformStamped.transform.rotation.w);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "my_tf2_listener");
    ros::NodeHandle node;
    ros::ServiceServer service = node.advertiseService("transform_listener_srv", transformServiceCallback);
    ROS_INFO("Service is up!");
    ros::spin();

    return 0;
}