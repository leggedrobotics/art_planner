#pragma once

#include <atomic>
#include <mutex>
#include <thread>

#include <ompl/geometric/planners/prm/LazyPRMstar.h>

#include "art_planner/map/map.h"
#include "art_planner/params.h"



namespace og = ompl::geometric;
namespace ob = ompl::base;



namespace art_planner {



class LazyPRMStarMinUpdateMaintainer;



class LazyPRMStarMinUpdate : public og::LazyPRMstar {

  friend LazyPRMStarMinUpdateMaintainer;
  static const unsigned int VALIDITY_UNKNOWN = og::LazyPRM::VALIDITY_UNKNOWN;
  static const unsigned int VALIDITY_TRUE = og::LazyPRM::VALIDITY_TRUE;

  void removeVertices(const std::set<Vertex>& milestones_to_remove);

  void updateVertices();

  std::vector<Vertex> getGraphVertices() const;

  std::vector<Vertex> getGraphVerticesInRandomOrder() const;

  bool checkIfValid(const Vertex& v);

  Vertex addValidMilestone(ob::State *state);



  mutable std::mutex mutex_;

  std::unique_ptr<LazyPRMStarMinUpdateMaintainer> maintainer_;



public:

  using LazyPRMstar::LazyPRMstar;

  void setMaintainer(std::unique_ptr<LazyPRMStarMinUpdateMaintainer>&& maintainer);

  virtual void clear() override;

  virtual void setup() override;

  // Contrary to the default version, this only returns validated edges.
  virtual void getPlannerData(ob::PlannerData &data) const override;

  virtual ob::PlannerStatus solve(const ob::PlannerTerminationCondition &ptc) override;

  ob::PlannerStatus baseSolve(const ob::PlannerTerminationCondition &ptc);

  ob::PathPtr constructSolution(const Vertex &start, const Vertex &goal);


};



class LazyPRMStarMinUpdateMaintainer {

  using Vertex = typename LazyPRMStarMinUpdate::Vertex;
  using Edge = typename LazyPRMStarMinUpdate::Edge;

  ParamsConstPtr params_;

  std::atomic<bool> can_clean_{false};
  std::atomic<bool> shutdown_requested_{false};
  std::thread connection_checker_thread_;

  LazyPRMStarMinUpdate* p_{nullptr};

  std::shared_ptr<Map> map_;



  void connectionCheckerThread();

  void invalidateUpdatedGraphComponents();

  void removeOutdatedVertices();

  void removeStartGoalVertices();

  bool wasUpdated(const LazyPRMStarMinUpdate::Vertex& v) const;

  bool wasUpdated(const LazyPRMStarMinUpdate::Edge& e) const;

  bool isOutOfBounds(const Vertex& v) const;

public:

  LazyPRMStarMinUpdateMaintainer(const std::shared_ptr<Map> &map,
                                 const ParamsConstPtr& params);

  ~LazyPRMStarMinUpdateMaintainer();

  inline void setPlanner(LazyPRMStarMinUpdate* planner) {
    p_ = planner;
  }

  void update();

  inline void requestPauseCleaning() {
    can_clean_ = false;
  }

  inline void continueCleaning() {
    can_clean_ = true;
  }

};



}
