#include "art_planner/validity_checker/validity_checker.h"



using namespace art_planner;



StateValidityChecker::StateValidityChecker(const ob::SpaceInformationPtr &si,
                                           const ParamsConstPtr& params) :
     ob::StateValidityChecker(si),
     checker_feet_(params),
     checker_body_(params),
     params_(params) {
}



void StateValidityChecker::setMap(const std::shared_ptr<Map> &map) {
  checker_feet_.setMap(map);
  checker_body_.setMap(map);
}



void StateValidityChecker::updateHeightField() {
  checker_feet_.updateHeightField();
  checker_body_.updateHeightField();
}



bool StateValidityChecker::hasMap() const {
  return checker_feet_.hasMap() && checker_body_.hasMap();
}



bool StateValidityChecker::isValid(const ob::State *state) const {
  const auto state_pose = Pose3FromSE3(state);
  const auto pose_base = state_pose*Pose3FromXYZ(params_->robot.torso.offset.x,
                                                 params_->robot.torso.offset.y,
                                                 params_->robot.torso.offset.z - params_->robot.feet.offset.z);
  return checker_body_.isValid(pose_base) && checker_feet_.isValid(state_pose);
}



double StateValidityChecker::clearance(const ob::State *state) const {
  return checker_feet_.getClearance(Pose3FromSE3(state));
}
