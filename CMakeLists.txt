cmake_minimum_required(VERSION 3.0.2)
project(planner)

find_package(catkin REQUIRED
  COMPONENTS
    interactive_markers
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
    tf2_ros
    tf2_eigen
    tf2_geometry_msgs
    actionlib_msgs
    control_msgs
    tf
    tf_conversions
    eigen_conversions
)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)

generate_messages(
  DEPENDENCIES actionlib_msgs std_msgs trajectory_msgs control_msgs  # Or other packages containing msgs
)

catkin_package(
  LIBRARIES
  INCLUDE_DIRS
  CATKIN_DEPENDS
    moveit_core
    moveit_visual_tools
    moveit_ros_planning_interface
    interactive_markers
    tf2_geometry_msgs
    actionlib_msgs
    control_msgs
  DEPENDS
    EIGEN3
)

include_directories(SYSTEM ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS})

add_executable(pick_and_place src/pick_and_place.cpp)
target_link_libraries(pick_and_place ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(stomp_pick_and_place src/stomp_pick_and_place.cpp)
target_link_libraries(stomp_pick_and_place ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(exagerate_trajectory src/exagerate_trajectory.cpp)
target_link_libraries(exagerate_trajectory ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(exagerate_trajectory_constraints src/exagerate_trajectory_constraints.cpp)
target_link_libraries(exagerate_trajectory_constraints ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(exagerate_trajectory_pilz src/exagerate_trajectory_pilz.cpp)
target_link_libraries(exagerate_trajectory_pilz ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(exagerate_trajectory_yumi src/exagerate_trajectory_yumi.cpp)
target_link_libraries(exagerate_trajectory_yumi ${catkin_LIBRARIES} ${Boost_LIBRARIES})
