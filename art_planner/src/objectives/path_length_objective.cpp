#include "art_planner/objectives/path_length_objective.h"



using namespace art_planner;



inline double getAngleDiff(double x, double y){
  const double d = std::fabs(y - x);
  return (d > M_PI) ? 2.0 * M_PI - d : d;
}



PathLengthObjective::PathLengthObjective(const ob::SpaceInformationPtr &si,
                                         const ParamsConstPtr &params)
  : Base::PathLengthOptimizationObjective(si),
    params_(params),
    checker_(si->getStateValidityChecker()) {

}



ob::Cost PathLengthObjective::motionCost(const ob::State* s1,
                                         const ob::State* s2) const {
  if (!params_->objectives.custom_path_length.use_directional_cost) {
    return motionCostHeuristic(s1, s2);
  } else {
    const double x1 = s1->as<ob::SE3StateSpace::StateType>()->getX();
    const double x2 = s2->as<ob::SE3StateSpace::StateType>()->getX();
    const double y1 = s1->as<ob::SE3StateSpace::StateType>()->getY();
    const double y2 = s2->as<ob::SE3StateSpace::StateType>()->getY();
    const double z1 = s1->as<ob::SE3StateSpace::StateType>()->getZ();
    const double z2 = s2->as<ob::SE3StateSpace::StateType>()->getZ();
    const double yaw1 = getYawFromSO3(s1->as<ob::SE3StateSpace::StateType>()->rotation());
    const double yaw2 = getYawFromSO3(s2->as<ob::SE3StateSpace::StateType>()->rotation());
    const double x_dif = x2 - x1;
    const double y_dif = y2 - y1;
    // TODO: Also use z difference.
    const double z_dif = z2 - z1;
    const double yaw_dif = getAngleDiff(yaw2, yaw1);

    const double lon_dif = cos(yaw1)*x_dif + sin(yaw1)*y_dif;
    const double lat_dif = -sin(yaw1)*x_dif + cos(yaw1)*y_dif;

    const double t_yaw = std::fabs(yaw_dif) / params_->objectives.custom_path_length.max_ang_vel;
    const double t_lon = std::fabs(lon_dif) / params_->objectives.custom_path_length.max_lon_vel;
    const double t_lat = std::fabs(lat_dif) / params_->objectives.custom_path_length.max_lat_vel;

    return ob::Cost(std::max(std::max(t_lon, t_lat), t_yaw));
  }
}



ob::Cost PathLengthObjective::motionCostHeuristic(const ob::State* s1,
                                                  const ob::State* s2) const {
  const double x1 = s1->as<ob::SE3StateSpace::StateType>()->getX();
  const double x2 = s2->as<ob::SE3StateSpace::StateType>()->getX();
  const double y1 = s1->as<ob::SE3StateSpace::StateType>()->getY();
  const double y2 = s2->as<ob::SE3StateSpace::StateType>()->getY();
  const double z1 = s1->as<ob::SE3StateSpace::StateType>()->getZ();
  const double z2 = s2->as<ob::SE3StateSpace::StateType>()->getZ();
  const double x_dif = x2 - x1;
  const double y_dif = y2 - y1;
  const double z_dif = z2 - z1;
  return ob::Cost(std::sqrt(x_dif*x_dif + y_dif*y_dif + z_dif*z_dif)/params_->objectives.custom_path_length.max_lon_vel);
}


