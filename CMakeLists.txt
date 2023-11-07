cmake_minimum_required(VERSION 3.0.2)
project(planner)

find_package(catkin REQUIRED
  COMPONENTS
    interactive_markers
    visualization_msgs
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    moveit_ros_perception
    rviz_visual_tools
    moveit_visual_tools
    pluginlib
    geometric_shapes
    pcl_ros
    pcl_conversions
    rosbag
    tf2
    tf2_ros
    tf2_eigen
    tf2_geometry_msgs
    actionlib_msgs
    control_msgs
    tf
    tf_conversions
    eigen_conversions
    std_msgs
    genmsg
    message_generation
    moveit_grasps
    hri_interface
    hri
)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)

# generate_messages(
#   DEPENDENCIES actionlib_msgs std_msgs trajectory_msgs control_msgs  # Or other packages containing msgs
# )

add_service_files(
  FILES
  boardOrientation.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  LIBRARIES
  INCLUDE_DIRS
  CATKIN_DEPENDS
    moveit_core
    moveit_visual_tools
    moveit_ros_planning_interface
    interactive_markers
    visualization_msgs
    tf2_geometry_msgs
    actionlib_msgs
    control_msgs
    moveit_grasps
    hri
  DEPENDS
    EIGEN3
)



include_directories(SYSTEM ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS})

add_executable(non_verbal_movements_yumi src/non_verbal_movements_yumi.cpp)
target_link_libraries(non_verbal_movements_yumi ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(interactive_marker src/interactive_marker.cpp)
target_link_libraries(interactive_marker ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(task src/task.cpp)
target_link_libraries(task ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(pick_and_place src/pick_and_place.cpp)
target_link_libraries(pick_and_place ${catkin_LIBRARIES} ${Boost_LIBRARIES})

