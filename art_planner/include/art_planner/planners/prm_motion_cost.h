#pragma once

#include <atomic>
#include <functional>
#include <mutex>
#include <thread>

#include <ompl/geometric/planners/prm/LazyPRMstar.h>

#include "art_planner/map/map.h"
#include "art_planner/params.h"
#include <art_planner/objectives/motion_cost_objective.h>



namespace og = ompl::geometric;
namespace ob = ompl::base;



namespace art_planner {



class PRMMotionCostMaintainer;



class PRMMotionCost : public og::LazyPRMstar {


  friend PRMMotionCostMaintainer;
  static const unsigned int VALIDITY_UNKNOWN = og::LazyPRM::VALIDITY_UNKNOWN;
  static const unsigned int VALIDITY_TRUE = og::LazyPRM::VALIDITY_TRUE;

//  void removeVertices(const std::set<Vertex>& milestones_to_remove);

//  void updateVertices();

//  std::vector<Vertex> getGraphVertices() const;

//  std::vector<Vertex> getGraphVerticesInRandomOrder() const;

//  bool checkIfValid(const Vertex& v);

  Vertex addValidMilestone(ob::State *state);



  mutable std::mutex mutex_;

  std::unique_ptr<PRMMotionCostMaintainer> maintainer_;

  bool computeCostForVertexEdges(const Vertex& v);



public:

  using LazyPRMstar::LazyPRMstar;

  void setMaintainer(std::unique_ptr<PRMMotionCostMaintainer>&& maintainer);

  virtual void clear() override;

  virtual void setup() override;

  // Contrary to the default version, this only returns validated edges.
  void getPlannerData(ob::PlannerData &data, bool get_invalid) const;

  virtual ob::PlannerStatus solve(const ob::PlannerTerminationCondition &ptc) override;

  ob::PlannerStatus baseSolve(const ob::PlannerTerminationCondition &ptc);

  ob::PathPtr constructSolution(const Vertex &start, const Vertex &goal);

  void sampleGraph();


};



class PRMMotionCostMaintainer {

public:
  using Vertex = typename PRMMotionCost::Vertex;
  using Edge = typename PRMMotionCost::Edge;

private:
  ParamsConstPtr params_;

  std::atomic<bool> can_clean_{false};
  std::atomic<bool> shutdown_requested_{false};
  std::thread connection_checker_thread_;

  PRMMotionCost* p_{nullptr};

  std::shared_ptr<Map> map_;

  std::unique_ptr<MotionCostObjective::MotionCostFunc> motion_cost_func_;

  grid_map::Time last_map_update_time_{0};


  void invalidateUpdatedGraphComponents();

  void removeOutdatedVertices();

  void removeStartGoalVertices();

  bool wasUpdated(const PRMMotionCost::Vertex& v) const;

  bool wasUpdated(const PRMMotionCost::Edge& e) const;

  bool isOutOfBounds(const Vertex& v) const;

  bool updateEdges();

  void setMotionCostFunction(std::unique_ptr<MotionCostObjective::MotionCostFunc>&& motion_cost_func);

public:

  PRMMotionCostMaintainer(const std::shared_ptr<Map> &map,
                          const ParamsConstPtr& params,
                          std::unique_ptr<MotionCostObjective::MotionCostFunc>&& motion_cost_func);

  ~PRMMotionCostMaintainer();

  inline void setPlanner(PRMMotionCost* planner) {
    p_ = planner;
  }

  void update();

  void sampleGraph();

};



}
