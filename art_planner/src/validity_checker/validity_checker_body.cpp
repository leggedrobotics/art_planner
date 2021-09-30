#include "art_planner/validity_checker/validity_checker_body.h"



using namespace art_planner;



ValidityCheckerBody::ValidityCheckerBody(const ParamsConstPtr& params) : params_(params) {
  checker_.reset(new HeightMapBoxChecker(params_->robot.torso.length,
                                         params_->robot.torso.width,
                                         params_->robot.torso.height));
}



void ValidityCheckerBody::setMap(const MapPtr& map) {
  elevation_map_ = map;
}



static HeightMapBoxChecker::dPose d_pose;



bool ValidityCheckerBody::isValid(const Pose3& pose) const {
  // Return no collision if outside of map bounds.
  if (!elevation_map_->isInside(grid_map::Position(pose.translation().x(),
                                                   pose.translation().y()))) {
    return true;
  }

  std::lock_guard<std::mutex> lock(mutex_);

  d_pose.origin[0] = pose.translation().x();
  d_pose.origin[1] = pose.translation().y();
  d_pose.origin[2] = pose.translation().z();
  Eigen::Map<Eigen::Matrix<Scalar, 3, 4,Eigen::RowMajor> > rot(d_pose.rotation.data());
  rot.topLeftCorner(3,3) = pose.matrix().topLeftCorner(3,3);
  return !static_cast<bool>(checker_->checkCollision({d_pose}));
}



bool ValidityCheckerBody::hasMap() const {
  return static_cast<bool>(elevation_map_);
}



void ValidityCheckerBody::updateHeightField() {
  std::lock_guard<std::mutex> lock(mutex_);
  checker_->setHeightField(elevation_map_, params_->planner.elevation_layer);
}
