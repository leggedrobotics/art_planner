#include "art_planner/objectives/motion_cost_objective.h"



using namespace art_planner;



inline double getAngleDiff(double x, double y){
  const double d = std::fabs(y - x);
  return (d > M_PI) ? 2.0 * M_PI - d : d;
}



MotionCostObjective::MotionCostObjective(const ob::SpaceInformationPtr &si,
                                         const ParamsConstPtr &params,
                                         std::unique_ptr<MotionCostFunc>&& motion_cost_func)
  : Base::PathLengthOptimizationObjective(si),
    params_(params),
    motion_cost_func_(std::move(motion_cost_func)),
    checker_(si->getStateValidityChecker()) {

}



bool MotionCostObjective::costQuery(const EdgeMatrix& edge_matrix,
                                     EdgeMatrix* edge_cost) const {
  edge_cost->resize(edge_matrix.rows(), 3);
  return (*motion_cost_func_)(edge_matrix, edge_cost);
}



ob::Cost MotionCostObjective::motionCost(const ob::State* s1,
                                         const ob::State* s2) const {
  const auto s1_se3 = s1->as<ob::SE3StateSpace::StateType>();
  const auto s2_se3 = s2->as<ob::SE3StateSpace::StateType>();

  const auto dist = lateralDistance(s1, s2);
  const unsigned int n_interp = dist / params_->planner.prm_motion_cost.max_query_edge_length;

  EdgeMatrix edge_matrix;
  EdgeMatrix edge_cost;

  // Interpolate connection.
  auto& ss = si_->getStateSpace();
  const double n_interp_div = 1.0/(n_interp+1);

  edge_matrix.resize(n_interp+1, 6);
  edge_cost.resize(n_interp+1, 3);

  // Add source state as first start.
  edge_matrix(0, 3) = s1_se3->getX();
  edge_matrix(0, 4) = s1_se3->getY();
  edge_matrix(0, 5) = getYawFromSO3(s1_se3->rotation());

  // Add target state as last goal.
  edge_matrix(n_interp, 0) = s2_se3->getX();
  edge_matrix(n_interp, 1) = s2_se3->getY();
  edge_matrix(n_interp, 2) = getYawFromSO3(s2_se3->rotation());

  // Fill matrix with interpolated states.
  ob::ScopedState<> cur_state(si_);
  for (unsigned int step = 1; step < n_interp+1; ++step) {
    ss->interpolate(s1, s2, step * n_interp_div, cur_state.get());

    const auto cur_state_se3 = cur_state.get()->as<ob::SE3StateSpace::StateType>();
    edge_matrix(step-1, 0) = cur_state_se3->getX();
    edge_matrix(step-1, 1) = cur_state_se3->getY();
    edge_matrix(step-1, 2) = getYawFromSO3(cur_state_se3->rotation());
    edge_matrix(step, 3) = cur_state_se3->getX();
    edge_matrix(step, 4) = cur_state_se3->getY();
    edge_matrix(step, 5) = getYawFromSO3(cur_state_se3->rotation());
  }

  if (!costQuery(edge_matrix, &edge_cost)) {
    // Service call failed, return infinity for safety.
    // TODO: Maybe throw error here. Needs to be handled well, though.
    throw std::runtime_error("Motion cost call failed");
    return ob::Cost(std::numeric_limits<double>::infinity());
  }

  double cost = 0;
  for (unsigned int i = 0; i < n_interp+1; ++i) {
    if (getRisk(edge_cost.row(i)) > params_->planner.prm_motion_cost.risk_threshold) {
      // Return infinite cost if any part is not feasible.
      return ob::Cost(std::numeric_limits<double>::infinity());
    }
    cost += getCost(edge_cost.row(i));
  }
  return ob::Cost(cost);

}



ob::Cost MotionCostObjective::motionCostHeuristic(const ob::State* s1,
                                                  const ob::State* s2) const {
  // TODO: Figure out better heuristic here, if it is used anywhere.
  return ob::Cost(0.0);
}


