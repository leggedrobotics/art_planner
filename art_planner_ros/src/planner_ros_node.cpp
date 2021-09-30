#include "art_planner_ros/planner_ros.h"



int main(int argc, char** argv) {
  ros::init(argc, argv, "art_planner");

  ros::NodeHandle nh("~");
  art_planner::PlannerRos planner(nh);

  ros::AsyncSpinner spinner(nh.param<int>("planner/n_threads",1));

  spinner.start();
  ros::waitForShutdown();
}
