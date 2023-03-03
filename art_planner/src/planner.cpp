#include "art_planner/planner.h"

#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/planners/rrt/RRTsharp.h>
#include <ompl/geometric/planners/prm/LazyPRMstar.h>
#include <ompl/util/Console.h>

#include "art_planner/goal.h"
#include "art_planner/map/processors/basic.h"
#include "art_planner/map/processors/change.h"
#include "art_planner/map/processors/probability_distribution.h"
#include "art_planner/map/processors/sample_density.h"
#include "art_planner/objectives/path_length_objective.h"
#include "art_planner/planners/lazy_prm_star_min_update.h"
#include "art_planner/planners/prm_motion_cost.h"
#include "art_planner/sampler.h"
#include "art_planner/start.h"



using namespace art_planner;
using namespace std::placeholders;



ob::OptimizationObjectivePtr art_planner::getObjective(const ob::SpaceInformationPtr& si,
                                                       const ParamsConstPtr& params) {
  auto opt = std::make_shared<ob::MultiOptimizationObjective>(si);

  ob::OptimizationObjectivePtr length_obj(new PathLengthObjective(si, params));
  opt->addObjective(length_obj, 1.0);

  return std::dynamic_pointer_cast<ob::OptimizationObjective>(opt);
}



void Planner::setUpMapProcessors() {
  // New map processors.
  art_planner::processors::ChainNewMap chain_new;
  chain_new.appendFunction(art_planner::processors::Basic(params_));
  if (params_->sampler.sample_from_distribution) {
    if (params_->sampler.use_inverse_vertex_density) {
      chain_new.appendFunction(std::bind(art_planner::processors::computeInverseSampleDensity,
                                         _1,
                                         ss_->getPlanner(),
                                         (params_->robot.torso.length + params_->robot.torso.width)*0.25));
    }
    chain_new.appendFunction(art_planner::processors::applyBaseSampleDistribution);
    if (params_->sampler.use_max_prob_unknown_samples) {
      chain_new.appendFunction(std::bind(art_planner::processors::applyMaxUnknownProbability,
                                         _1,
                                         params_->sampler.max_prob_unknown_samples));
    }
    chain_new.appendFunction(art_planner::processors::computeCumulativeProbabilityDistribution);
  }
  map_->setNewMapProcessorChain(chain_new);

  // Old map post processors.
  art_planner::processors::ChainOldMap chain_old;
  if (params_->planner.lazy_prm_star_min_update.invalidate_updated_graph_components) {
    chain_old.appendFunction(std::bind(art_planner::processors::computeChange,
                                       _1,
                                       _2,
                                       params_->planner.lazy_prm_star_min_update.height_change_for_update,
                                       params_->planner.elevation_layer));
  }
  map_->setOldMapProcessorChain(chain_old);

}



Planner::Planner(const ParamsConstPtr& params)
    : params_(params),
      map_(std::make_shared<Map>(params)),
      sampler_allocator_{params_} {
  space_ = std::make_shared<StateSpace>();
  ss_ = std::make_shared<og::SimpleSetup>(space_);

  if (params_->verbose) {
    ompl::msg::setLogLevel(ompl::msg::LogLevel::LOG_DEBUG);
  } else {
    ompl::msg::setLogLevel(ompl::msg::LogLevel::LOG_ERROR);
  }

  auto si = ss_->getSpaceInformation();

  // Set algorithm.
  ob::PlannerPtr planner;
  if (params_->planner.name == "rrt_star") {
    planner.reset(new og::RRTstar(si));
  } else if (params_->planner.name == "inf_rrt_star") {
    planner.reset(new og::InformedRRTstar(si));
  } else if (params_->planner.name == "rrt_sharp") {
    planner.reset(new og::RRTsharp(si));
  } else if (params_->planner.name == "lazy_prm_star") {
    planner.reset(new og::LazyPRMstar(si));
  } else if (params_->planner.name == "lazy_prm_star_min_update") {
    planner.reset(new LazyPRMStarMinUpdate(si));
  } else if (params_->planner.name == "prm_motion_cost") {
    planner.reset(new PRMMotionCost(si));
  } else {
    throw std::runtime_error("Unknown planner requested: " + params_->planner.name);
  }
  ss_->setPlanner(planner);

  setUpMapProcessors();

  // Set maintainer for planner.
  if (params_->planner.name == "lazy_prm_star_min_update") {
    planner->as<LazyPRMStarMinUpdate>()->setMaintainer(
          std::unique_ptr<LazyPRMStarMinUpdateMaintainer>(
              new LazyPRMStarMinUpdateMaintainer(map_, params_)));
  }

  // Set sampler.
  sampler_allocator_.setMap(map_);
  space_->setStateSamplerAllocator(std::bind(&SE3FromSE2SamplerAllocator::getSampler,
                                             sampler_allocator_,
                                             std::placeholders::_1));

  // Set checker.
  checker_ = std::make_shared<StateValidityChecker>(si, params_);
  checker_->setMap(map_);
  ss_->setStateValidityChecker(checker_);

  // Set objective.
  ss_->setOptimizationObjective(getObjective(si, params_));
}



void Planner::setMap(std::unique_ptr<grid_map::GridMap>&& map) {

  if (!map->exists(params_->planner.elevation_layer)) {
    if (params_->verbose) {
      std::cout << "Grid map does not have \""
                << params_->planner.elevation_layer
                << "\" layer." << std::endl;
    }
    return;
  }

  const auto& pos = map->getPosition();
  const auto& length = map->getLength();
  ob::RealVectorBounds bounds(3);
  bounds.setLow(0, pos.x() - length.x());
  bounds.setLow(1, pos.y() - length.y());
  bounds.setLow(2, map->get(params_->planner.elevation_layer).minCoeffOfFinites()
                     - params_->robot.feet.reach.z/2);
  bounds.setHigh(0, pos.x() + length.x());
  bounds.setHigh(1, pos.y() + length.y());
  bounds.setHigh(2, map->get(params_->planner.elevation_layer).maxCoeffOfFinites()
                      + params_->robot.feet.reach.z/2);

  std::lock_guard<std::mutex> lock(map_mutex_);

  space_->setBounds(bounds);
  map_->setMap(std::move(map));
  checker_->updateHeightField();
}



void Planner::setStartAndGoal(const ob::ScopedState<>& start,
                              const ob::ScopedState<>& goal) {
  // Get valid start state.
  ob::ScopedState<> start_valid = start;
  StartState start_state(ss_->getSpaceInformation());
  start_state.setState(start);
  start_state.setVerbosity(params_->verbose);
  start_state.setThreshold(params_->planner.start_goal_search.start_radius);
  start_state.setMaxNumSamples(params_->planner.start_goal_search.n_iter);
  start_state.sampleGoal(start_valid.get());

  ss_->setStartState(start_valid);

  // Get valid goal.
  auto goal_state = std::make_shared<GoalStateRegion>(ss_->getSpaceInformation());
  goal_state->setState(goal);
  goal_state->setThreshold(params_->planner.start_goal_search.goal_radius);
  goal_state->setMaxNumSamples(params_->planner.start_goal_search.n_iter);
  goal_state->setVerbosity(params_->verbose);
  ss_->setGoal(goal_state);

  solved_ = false;
}



PlannerStatus Planner::plan(const ob::ScopedState<>& start,
                            const ob::ScopedState<>& goal) {
  std::lock_guard<std::mutex> lock(map_mutex_);

  if (!checker_->hasMap()) {
    std::cout << "Planner does not have the elevation map set, yet." << std::endl;
    return PlannerStatus::NO_MAP;
  }

  // Enforce goal pose inside bounds.
  auto space = ss_->getStateSpace();
  ob::ScopedState<> goal_clipped(goal);
  auto goal_clipped_ptr = goal_clipped->as<StateType>();

  if (!space_->satisfiesBounds(goal_clipped.get())) {
    if(params_->verbose) {
      printf("Original goal [%f\t%f\t%f], ",
             goal_clipped_ptr->getX(),
             goal_clipped_ptr->getY(),
             goal_clipped_ptr->getZ());
    }
    ss_->getStateSpace()->enforceBounds(goal_clipped.get());
    if (params_->verbose) {
      printf("clipped to [%f\t%f\t%f].\n",
             goal_clipped_ptr->getX(),
             goal_clipped_ptr->getY(),
             goal_clipped_ptr->getZ());
    }
  }

  // Get goal height from elevation map.
  if (map_->isInside(grid_map::Position(goal_clipped_ptr->getX(),
                                        goal_clipped_ptr->getY()))) {
    // Get height, roll, pitch from height map.
    double xyzrpy[6];
    xyzrpy[0] = goal_clipped_ptr->getX();
    xyzrpy[1] = goal_clipped_ptr->getY();
    xyzrpy[5] = getYawFromSO3(goal_clipped_ptr->rotation());

    map_->get3DPoseFrom2D(xyzrpy);

    goal_clipped_ptr->setZ(xyzrpy[2]);
    setSO3FromRPY(goal_clipped_ptr->rotation(), xyzrpy+3);

  }

  // Reset planner and start planning.
  ss_->getPlanner()->clearQuery();
  setStartAndGoal(start, goal_clipped);

  ob::PlannerStatus solved;
  try {
    solved = ss_->solve(params_->planner.plan_time);
    solved_ = static_cast<bool>(solved);
  } catch (ompl::Exception& e) {
    // All graph edges to goal where actually invalid.
    std::cout << e.what() << std::endl;
    solved_ = false;
    return PlannerStatus::NOT_SOLVED;
  }

  switch (ob::PlannerStatus::StatusType(solved)) {
    case ob::PlannerStatus::INVALID_START: return PlannerStatus::INVALID_START;
    case ob::PlannerStatus::INVALID_GOAL: return PlannerStatus::INVALID_GOAL;
    case ob::PlannerStatus::UNRECOGNIZED_GOAL_TYPE: return PlannerStatus::INVALID_GOAL;
    case ob::PlannerStatus::TIMEOUT: return PlannerStatus::NOT_SOLVED;
    case ob::PlannerStatus::EXACT_SOLUTION: return PlannerStatus::SOLVED;
  }
  return PlannerStatus::UNKNOWN;
}



og::PathGeometric Planner::getSolutionPath(const bool& simplify) const {
  auto path = ss_->getSolutionPath();
  if (!solved_) {
    throw ompl::Exception("Requested failed solution path.");
  }
  if (simplify) {
    ss_->simplifySolution();
    const auto path_simple = ss_->getSolutionPath();

    // checkAndRepair() inside of simplifySolution might fail, but the result of that
    // operation is not accessible through SimpleSetup. That's why we need to do the
    // following checks to catch this case.
    if (!path_simple.check()) {
      std::cout << "Simplified path is invalid. Returning original." << std::endl;
    } else {
      const auto obj = ss_->getOptimizationObjective();
      const auto cost_simple = path_simple.cost(obj);
      const auto cost_orig = path.cost(obj);
      if (params_->verbose) {
        std::cout << "cost_simple " << cost_simple << std::endl;
        std::cout << "cost_orig " << cost_orig << std::endl;
      }
      if (obj->isCostBetterThan(cost_orig, cost_simple)) {
        if (params_->verbose) {
          std::cout << "Original path cost is lower than simplified. Returning original." << std::endl;
        }
      } else {
        path = path_simple;
      }
    }
  }
  return path;
}
