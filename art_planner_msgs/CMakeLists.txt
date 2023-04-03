cmake_minimum_required(VERSION 3.0.2)
project(art_planner_msgs)

find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs 
  genmsg
  geometry_msgs
)

add_action_files(
  DIRECTORY action
  FILES PlanToGoal.action
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS actionlib_msgs geometry_msgs
)
