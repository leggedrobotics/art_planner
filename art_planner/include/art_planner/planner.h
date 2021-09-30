#pragma once

#include <mutex>

#include <grid_map_core/GridMap.hpp>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/spaces/SE3StateSpace.h>

#include "art_planner/map/map.h"
#include "art_planner/params.h"
#include "art_planner/planner_status.h"
#include "art_planner/sampler.h"
#include "art_planner/validity_checker/validity_checker.h"



namespace ob = ompl::base;
namespace og = ompl::geometric;



namespace art_planner {



ob::OptimizationObjectivePtr getObjective(const ob::SpaceInformationPtr& si,
                                          const ParamsConstPtr &params);



class Planner {

public:

  using StateSpace = ob::SE3StateSpace;
  using StateType = typename StateSpace::StateType;

protected:

  // Parameters.
  ParamsConstPtr params_;

  std::shared_ptr<og::SimpleSetup> ss_;
  bool solved_{false};
  std::shared_ptr<StateSpace> space_;
  std::shared_ptr<StateValidityChecker> checker_;

  std::shared_ptr<Map> map_;
  mutable std::mutex map_mutex_;

  SE3FromSE2SamplerAllocator sampler_allocator_;



  void setStartAndGoal(const ob::ScopedState<>& start,
                       const ob::ScopedState<>& goal);

  void setUpMapProcessors();



public:
  Planner(const ParamsConstPtr& params = std::make_shared<const Params>());

  void setMap(std::unique_ptr<grid_map::GridMap>&& map);

  PlannerStatus plan(const ob::ScopedState<>& start,
                     const ob::ScopedState<>& goal);

  og::PathGeometric getSolutionPath(const bool& simplify = false) const;
};



}
