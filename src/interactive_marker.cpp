#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <interactive_markers/interactive_marker_server.h>

void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
    ROS_INFO_STREAM("Marker '" << feedback->marker_name << "' is now at position ("
                               << feedback->pose.position.x << ", " << feedback->pose.position.y << ", "
                               << feedback->pose.position.z << ")");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "interactive_marker_example");
    ros::NodeHandle nh;

    // Create an interactive marker server
    interactive_markers::InteractiveMarkerServer server("interactive_marker");

    // Create an interactive marker
    visualization_msgs::InteractiveMarker int_marker;
    int_marker.header.frame_id = "yumi_base_link";
    int_marker.name = "my_marker";
    int_marker.description = "Interactive Marker";

    // Create a marker control
    visualization_msgs::Marker marker;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    visualization_msgs::InteractiveMarkerControl control;
    control.always_visible = true;
    control.markers.push_back(marker);

    // Create a control with a rotation interaction mode
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);

    // Create a control with a translation interaction mode
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_PLANE;
    control.markers.clear();
    control.markers.push_back(marker);
    int_marker.controls.push_back(control);

    // Create a control for translation gizmo
    visualization_msgs::InteractiveMarkerControl gizmo_control;
    gizmo_control.orientation_mode = visualization_msgs::InteractiveMarkerControl::FIXED;
    gizmo_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_3D;
    gizmo_control.orientation.w = 1.0;
    gizmo_control.name = "move_gizmo";
    gizmo_control.markers.push_back(marker);
    int_marker.controls.push_back(gizmo_control);

    // Add the interactive marker to the server
    server.insert(int_marker, &processFeedback);
    server.applyChanges();

    ros::spin();

    return 0;
}
