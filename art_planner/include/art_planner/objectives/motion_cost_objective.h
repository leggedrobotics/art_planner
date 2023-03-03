#pragma once

#include "art_planner/utils.h"

#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/spaces/SE3StateSpace.h>

#include "art_planner/params.h"
#include "art_planner/validity_checker/validity_checker.h"



namespace ob = ompl::base;

namespace art_planner {



class MotionCostObjective : public ob::PathLengthOptimizationObjective {

public:
  using EdgeMatrix = Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
  using MotionCostFunc = std::function<bool (const EdgeMatrix&, EdgeMatrix*)>;

private:
  // Parameters.
  ParamsConstPtr params_;

  std::unique_ptr<MotionCostFunc> motion_cost_func_;

protected:

  using Base = ob::PathLengthOptimizationObjective;

  ob::StateValidityCheckerPtr checker_;

  inline double getEnergy(const Eigen::Vector3f& cost_elements) const {
    return cost_elements(0);
  }

  inline double getTime(const Eigen::Vector3f& cost_elements) const {
    return cost_elements(1);
  }

  inline double getRisk(const Eigen::Vector3f& cost_elements) const {
    return cost_elements(2);
  }

public:
  MotionCostObjective(const ob::SpaceInformationPtr &si,
                      const ParamsConstPtr& params,
                      std::unique_ptr<MotionCostFunc>&& motion_cost_func);

  inline double getCost(const Eigen::Vector3f& cost_elements) const {
    const auto w_e = params_->planner.prm_motion_cost.cost_weights.energy;
    const auto w_t = params_->planner.prm_motion_cost.cost_weights.time;
    const auto w_r = params_->planner.prm_motion_cost.cost_weights.risk;
    const auto c_e = getEnergy(cost_elements);
    const auto c_t = getTime(cost_elements);
    const auto c_r = getRisk(cost_elements);
    return c_e*w_e + c_t*w_t + c_r*w_r;
  }

  inline bool isFeasible(const Eigen::Vector3f& cost_elements) const {
    return getRisk(cost_elements) <= params_->planner.prm_motion_cost.risk_threshold;
  }

  bool costQuery(const EdgeMatrix& edge_matrix, EdgeMatrix* edge_cost) const;

  virtual ob::Cost motionCost(const ob::State* s1, const ob::State* s2) const override;

  virtual ob::Cost motionCostHeuristic(const ob::State* s1, const ob::State* s2) const override;

};



}
