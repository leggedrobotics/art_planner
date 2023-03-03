#include "art_planner/validity_checker/validity_checker_feet.h"

#include <grid_map_core/iterators/PolygonIterator.hpp>

#include "art_planner/utils.h"



using namespace art_planner;



ValidityCheckerFeet::ValidityCheckerFeet(const ParamsConstPtr& params) : params_(params) {
  box_length_ = params_->robot.feet.reach.x;
  box_width_ = params_->robot.feet.reach.y;

  checker_.reset(new HeightMapBoxChecker(box_length_, box_width_, params_->robot.feet.reach.z));
}



void ValidityCheckerFeet::setMap(const MapPtr &map) {
  traversability_map_ = map;
}



static HeightMapBoxChecker::dPose d_pose;



bool ValidityCheckerFeet::boxIsValidAtPose(const Pose3 &pose) const {
  // Return if outside of map bounds.
  if (!traversability_map_->isInside(grid_map::Position(pose.translation().x(),
                                                        pose.translation().y()))) {
    return !params_->planner.unknown_space_untraversable;
  }

  std::lock_guard<std::mutex> lock(mutex_);

  d_pose.origin[0] = pose.translation().x();
  d_pose.origin[1] = pose.translation().y();
  d_pose.origin[2] = pose.translation().z();
  Eigen::Map<Eigen::Matrix<dReal, 3, 4,Eigen::RowMajor> > rot(d_pose.rotation.data());
  rot.topLeftCorner(3,3) = pose.matrix().topLeftCorner(3,3);
  return static_cast<bool>(checker_->checkCollision({d_pose}));
}



bool ValidityCheckerFeet::boxesAreValidAtPoses(const std::vector<Pose3> &poses) const {
  bool valid = true;
  for (const auto& pose: poses) {
    valid &= boxIsValidAtPose(pose);
    // Need all four feet to be valid, so we can return, when one is not.
    if (!valid) break;
  }
  return valid;
}



bool ValidityCheckerFeet::isValid(const Pose3& pose) const {
  std::vector<Pose3> foot_poses({
                     pose*Pose3FromXYZ( params_->robot.feet.offset.x,  params_->robot.feet.offset.y, 0.0f),
                     pose*Pose3FromXYZ( params_->robot.feet.offset.x, -params_->robot.feet.offset.y, 0.0f),
                     pose*Pose3FromXYZ(-params_->robot.feet.offset.x,  params_->robot.feet.offset.y, 0.0f),
                     pose*Pose3FromXYZ(-params_->robot.feet.offset.x, -params_->robot.feet.offset.y, 0.0f)});
  return boxesAreValidAtPoses(foot_poses);
}



bool ValidityCheckerFeet::hasMap() const {
  return static_cast<bool>(traversability_map_);
}



void ValidityCheckerFeet::updateHeightField() {
  std::lock_guard<std::mutex> lock(mutex_);
  checker_->setHeightField(traversability_map_, "elevation_masked");
}
