#pragma once

#include "art_planner/utils.h"

#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/spaces/SE3StateSpace.h>

#include "art_planner/params.h"
#include "art_planner/validity_checker/validity_checker.h"



namespace ob = ompl::base;

namespace art_planner {



class PathLengthObjective : public ob::PathLengthOptimizationObjective {

  // Parameters.
  ParamsConstPtr params_;

protected:

  using Base = ob::PathLengthOptimizationObjective;

  ob::StateValidityCheckerPtr checker_;

public:
  PathLengthObjective(const ob::SpaceInformationPtr &si,
                      const ParamsConstPtr& params);

  virtual ob::Cost motionCost(const ob::State* s1, const ob::State* s2) const override;

  virtual ob::Cost motionCostHeuristic(const ob::State* s1, const ob::State* s2) const override;

};



}
