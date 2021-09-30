#include "art_planner_ros/converter.h"



using namespace art_planner;



Converter::Converter(std::shared_ptr<Planner::StateSpace> space) : space_(space) {

}



ob::ScopedState<> Converter::poseRosToOmpl(const geometry_msgs::PoseStamped& pose_ros) const {
  ob::ScopedState<> pose(space_);

  auto pose_se3 = pose->as<Planner::StateType>();
  pose_se3->setX(pose_ros.pose.position.x);
  pose_se3->setY(pose_ros.pose.position.y);
  pose_se3->setZ(pose_ros.pose.position.z);
  pose_se3->rotation().w = pose_ros.pose.orientation.w;
  pose_se3->rotation().x = pose_ros.pose.orientation.x;
  pose_se3->rotation().y = pose_ros.pose.orientation.y;
  pose_se3->rotation().z = pose_ros.pose.orientation.z;

  return pose;
}



nav_msgs::Path Converter::pathOmplToRos(const og::PathGeometric& path_ompl) const {
  nav_msgs::Path path;
  path.poses.resize(path_ompl.getStateCount());

  for (size_t i = 0; i < path_ompl.getStateCount(); ++i) {
    const auto state = path_ompl.getState(i);
    const auto state_ompl = state->as<Planner::StateType>();
    path.poses[i].pose.position.x = state_ompl->getX();
    path.poses[i].pose.position.y = state_ompl->getY();
    path.poses[i].pose.position.z = state_ompl->getZ();
    path.poses[i].pose.orientation.x = state_ompl->rotation().x;
    path.poses[i].pose.orientation.y = state_ompl->rotation().y;
    path.poses[i].pose.orientation.z = state_ompl->rotation().z;
    path.poses[i].pose.orientation.w = state_ompl->rotation().w;
  }

  return path;
}
