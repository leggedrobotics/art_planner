#include "art_planner/planners/prm_motion_cost.h"

#include <algorithm>
#include <chrono>

#include <art_planner/utils.h>
#include <boost/foreach.hpp>
#include <boost/graph/lookup_edge.hpp>
#include <ompl/base/Planner.h>
#include <ompl/base/spaces/SE3StateSpace.h>



using namespace art_planner;
namespace og = ompl::geometric;



bool PRMMotionCostMaintainer::isOutOfBounds(const Vertex& v) const {
  const auto vertex_state = p_->stateProperty_[v]->as<ob::SE3StateSpace::StateType>();
  return !map_->isInside(grid_map::Position(vertex_state->getX(),
                                            vertex_state->getY()));
}



bool PRMMotionCostMaintainer::updateEdges() {
  MotionCostObjective::EdgeMatrix edge_matrix(boost::num_edges(p_->g_), 6);
  MotionCostObjective::EdgeMatrix edge_cost(edge_matrix.rows(), 3);

  // Fill edge matrix for target and start vertices.
  size_t counter = 0;
  BOOST_FOREACH(const Edge e, boost::edges(p_->g_)) {
    const Vertex v1 = boost::source(e, p_->g_);
    const Vertex v2 = boost::target(e, p_->g_);

    edge_matrix(counter, 0) = p_->stateProperty_[v2]->as<ob::SE3StateSpace::StateType>()->getX();
    edge_matrix(counter, 1) = p_->stateProperty_[v2]->as<ob::SE3StateSpace::StateType>()->getY();
    edge_matrix(counter, 2) = getYawFromSO3(p_->stateProperty_[v2]->as<ob::SE3StateSpace::StateType>()->rotation());
    edge_matrix(counter, 3) = p_->stateProperty_[v1]->as<ob::SE3StateSpace::StateType>()->getX();
    edge_matrix(counter, 4) = p_->stateProperty_[v1]->as<ob::SE3StateSpace::StateType>()->getY();
    edge_matrix(counter, 5) = getYawFromSO3(p_->stateProperty_[v1]->as<ob::SE3StateSpace::StateType>()->rotation());
    ++counter;
  }

  // Query cost.
  const auto n_edges_before = boost::num_edges(p_->g_);
  counter = 0;
  MotionCostObjective::MotionCostFunc& func = *motion_cost_func_;
  const auto obj = std::static_pointer_cast<MotionCostObjective>(p_->opt_);
  if (func(edge_matrix, &edge_cost)) {
    // Apply motion cost to graph.
    BOOST_FOREACH(const Edge e, boost::edges(p_->g_)) {
      if (obj->isFeasible(edge_cost.row(counter))) {
        boost::put(boost::edge_weight_t(), p_->g_, e, ob::Cost(obj->getCost(edge_cost.row(counter))));
        p_->edgeValidityProperty_[e] = PRMMotionCost::VALIDITY_TRUE;
      } else {
        // Too risky. Make cost infinite so it isn't used.
        boost::put(boost::edge_weight_t(), p_->g_, e, ob::Cost(std::numeric_limits<double>::infinity()));
      }
      ++counter;
    }

    const auto n_edges_after = boost::num_edges(p_->g_);
    if (params_->verbose) {
      std::cout << "Removed " << n_edges_before - n_edges_after << " risky edges." << std::endl;
    }

    return true;
  } else {
    return false;
  }
}



bool PRMMotionCost::computeCostForVertexEdges(const Vertex& v) {
  std::list<Edge> edges;
  BOOST_FOREACH(const Edge e, boost::in_edges(v, g_)) {
    edges.push_back(e);
  }
  BOOST_FOREACH(const Edge e, boost::out_edges(v, g_)) {
    edges.push_back(e);
  }

  const auto n_edges = edges.size();
  if (n_edges > 0) {
    MotionCostObjective::EdgeMatrix edge_matrix(n_edges, 6);
    MotionCostObjective::EdgeMatrix edge_cost(n_edges, 3);

    size_t counter = 0;
    for(const Edge e: edges) {
      const Vertex v1 = boost::source(e, g_);
      const Vertex v2 = boost::target(e, g_);

      edge_matrix(counter, 0) = stateProperty_[v2]->as<ob::SE3StateSpace::StateType>()->getX();
      edge_matrix(counter, 1) = stateProperty_[v2]->as<ob::SE3StateSpace::StateType>()->getY();
      edge_matrix(counter, 2) = getYawFromSO3(stateProperty_[v2]->as<ob::SE3StateSpace::StateType>()->rotation());
      edge_matrix(counter, 3) = stateProperty_[v1]->as<ob::SE3StateSpace::StateType>()->getX();
      edge_matrix(counter, 4) = stateProperty_[v1]->as<ob::SE3StateSpace::StateType>()->getY();
      edge_matrix(counter, 5) = getYawFromSO3(stateProperty_[v1]->as<ob::SE3StateSpace::StateType>()->rotation());
      ++counter;
    }

    counter = 0;
    const auto obj = std::static_pointer_cast<MotionCostObjective>(opt_);
    if (obj->costQuery(edge_matrix, &edge_cost)) {
      // Apply motion cost to graph.
      for(const Edge e: edges) {
        if (obj->isFeasible(edge_cost.row(counter))) {
          boost::put(boost::edge_weight_t(), g_, e, ob::Cost(obj->getCost(edge_cost.row(counter))));
        } else {
          // Too risky. Make cost infinite so it isn't used.
          boost::put(boost::edge_weight_t(), g_, e, ob::Cost(std::numeric_limits<double>::infinity()));
        }
        ++counter;
      }

      return true;
    } else {
      return false;
    }
  } else {
    // No connections, we do not need to query.
    return true;
  }

}



PRMMotionCostMaintainer::PRMMotionCostMaintainer(const std::shared_ptr<Map> &map,
                                                 const ParamsConstPtr& params,
                                                 std::unique_ptr<MotionCostObjective::MotionCostFunc>&& motion_cost_func)
  : params_(params), map_(map), motion_cost_func_(std::move(motion_cost_func)) {
}



PRMMotionCostMaintainer::~PRMMotionCostMaintainer() {
}



void PRMMotionCostMaintainer::sampleGraph() {
  const auto map_time = map_->getMap().getTimestamp();
  if (map_time == last_map_update_time_) {
    // Don't need to sample graph again if we query with same map.
    // TODO: Remove this spammy stuff, once we know it works.
    return;
  } else {
    last_map_update_time_ = map_time;
  }

  std::random_device rd;  // Will be used to obtain a seed for the random number engine.
  std::mt19937 gen(rd()); // Standard mersenne_twister_engine seeded with rd().


  if (p_) {
    ob::ScopedState<ob::SE3StateSpace> work_state(p_->si_);

    auto start = std::chrono::high_resolution_clock::now();
//    std::lock_guard<std::mutex> lock(p_->mutex_);


    size_t n_samples = 0;
    size_t n_valid_samples = 0;
    unsigned int n_proc = 0;
    bool timer_exceeded = false;

    while (boost::num_vertices(p_->g_) < params_->planner.prm_motion_cost.max_n_vertices
           && boost::num_edges(p_->g_) < params_->planner.prm_motion_cost.max_n_edges) {
      // Sample new vertex.
      do {
        p_->sampler_->sampleUniform(work_state.get());
        ++n_samples;
        if (n_samples % 100 == 0) {
          const auto end = std::chrono::high_resolution_clock::now();
          std::chrono::duration<double> elapsed_seconds = end-start;
          if (elapsed_seconds.count() > params_->planner.prm_motion_cost.max_sample_time) {
            std::cout << "Reached sample timer limit." << std::endl;
            timer_exceeded = true;
            break;
          }
        }
      } while (!p_->si_->isValid(work_state.get()));
      if (timer_exceeded) break;
      ++n_valid_samples;
      const auto vert1 = p_->addValidMilestone(p_->si_->cloneState(work_state.get()));
      if ((boost::num_vertices(p_->g_) / params_->planner.prm_motion_cost.recompute_density_after_n_samples) > n_proc) {
        map_->reApplyPreprocessing();
        ++n_proc;
      }
    }

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed_seconds = end-start;

    if (params_->verbose) {
      std::cout << "Sampled for " << std::setprecision(3) <<  elapsed_seconds.count()*1000.0 << "ms" << std::endl;
      std::cout << "Added " << n_valid_samples << " new milestones from " << n_samples << " attempts." << std::endl;
      std::cout << "for a total of " << boost::num_vertices(p_->g_) << " milestones" << std::endl;
      std::cout << "and " << boost::num_edges(p_->g_) << " edges." << std::endl;
    }

    start = std::chrono::high_resolution_clock::now();

    // TODO: Catch error here when updatEdges() fails.
    updateEdges();

    end = std::chrono::high_resolution_clock::now();
    elapsed_seconds = end-start;

    if (params_->verbose) {
      std::cout << "Queried cost in " << std::setprecision(3) <<  elapsed_seconds.count()*1000.0 << "ms" << std::endl;
    }
  }

}



void PRMMotionCostMaintainer::setMotionCostFunction(std::unique_ptr<MotionCostObjective::MotionCostFunc>&& motion_cost_func) {
  motion_cost_func_ = std::move(motion_cost_func);
}



void PRMMotionCost::setMaintainer(std::unique_ptr<PRMMotionCostMaintainer>&& maintainer) {
  maintainer_ = std::move(maintainer);
  maintainer_->setPlanner(this);
}



void PRMMotionCost::clear() {
  ob::Planner::clear();
  freeMemory();
  if (nn_)
      nn_->clear();

  clearQuery();

  componentCount_ = 0;
  iterations_ = 0;
  bestCost_ = ob::Cost(std::numeric_limits<double>::quiet_NaN());
}



void PRMMotionCost::setup() {
  og::LazyPRM::setup();

  sampler_ = si_->allocStateSampler();
}



void PRMMotionCost::getPlannerData(ob::PlannerData &data, bool get_invalid) const {
//  std::lock_guard<std::mutex> lock(mutex_);

  Planner::getPlannerData(data);

  // Explicitly add start and goal states. Tag all states known to be valid as 1.
  // Unchecked states are tagged as 0.
  for (auto i : startM_)
      data.addStartVertex(ob::PlannerDataVertex(stateProperty_[i], 1));

  for (auto i : goalM_)
      data.addGoalVertex(ob::PlannerDataVertex(stateProperty_[i], 1));

  // Adding edges and all other vertices simultaneously
  BOOST_FOREACH(const Edge e, boost::edges(g_)) {
    const Vertex v1 = boost::source(e, g_);
    const Vertex v2 = boost::target(e, g_);

    data.addVertex(stateProperty_[v1]);
    data.addVertex(stateProperty_[v2]);

    // Add tags for the newly added vertices
    data.tagState(stateProperty_[v1], (vertexValidityProperty_[v1] & VALIDITY_TRUE) == 0 ? 0 : 1);
    data.tagState(stateProperty_[v2], (vertexValidityProperty_[v2] & VALIDITY_TRUE) == 0 ? 0 : 1);

    if ((edgeValidityProperty_[e] || get_invalid) & VALIDITY_TRUE) {
      data.addEdge(ob::PlannerDataVertex(stateProperty_[v1]), ob::PlannerDataVertex(stateProperty_[v2]));
      // Add the reverse edge, since we're constructing an undirected roadmap
      data.addEdge(ob::PlannerDataVertex(stateProperty_[v2]), ob::PlannerDataVertex(stateProperty_[v1]));
    }

  }
}



ob::PlannerStatus PRMMotionCost::solve(const ob::PlannerTerminationCondition &ptc) {
  std::lock_guard<std::mutex> lock(mutex_);

  maintainer_->sampleGraph();

  ob::PlannerStatus status = baseSolve(ptc);

  return status;
}



#include "ompl/geometric/planners/prm/LazyPRM.h"
#include "ompl/base/objectives/PathLengthOptimizationObjective.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/geometric/planners/prm/ConnectionStrategy.h"
#include "ompl/tools/config/SelfConfig.h"
#include <boost/graph/astar_search.hpp>
#include <boost/graph/incremental_components.hpp>
#include <boost/graph/lookup_edge.hpp>
#include <boost/foreach.hpp>
#include <ompl/util/Exception.h>

#ifndef OMPL_GEOMETRIC_PLANNERS_PRM_A_STAR_GOAL_VISITOR_
#define OMPL_GEOMETRIC_PLANNERS_PRM_A_STAR_GOAL_VISITOR_

#include <boost/graph/astar_search.hpp>



og::LazyPRM::Vertex PRMMotionCost::addValidMilestone(ob::State *state) {
  Vertex m = boost::add_vertex(g_);
  stateProperty_[m] = state;
  vertexValidityProperty_[m] = VALIDITY_TRUE;
  unsigned long int newComponent = componentCount_++;
  vertexComponentProperty_[m] = newComponent;
  componentSize_[newComponent] = 1;

  // Which milestones will we attempt to connect to?
  const std::vector<Vertex> &neighbors = connectionStrategy_(m);
  BOOST_FOREACH (Vertex n, neighbors)
    if (connectionFilter_(m, n)) {
      const ob::Cost weight = ob::Cost(); // Proper cost is computed batched later on.
      const Graph::edge_property_type properties(weight);
      // Figure out if we need to interpolate between states.
      const auto dist = lateralDistance(stateProperty_[m], stateProperty_[n]);
      // TODO: Add minimum distance parameter.
      static constexpr double kMaxDist = 0.5;  // TODO: Pipe in parameter for distance here.
      const unsigned int n_interp = dist / kMaxDist;
      if (n_interp > 0) {
        bool connection_valid = true;
        // We interpolate.
        auto prev_milestone = m;
        const double n_interp_div = 1.0/(n_interp+1);
        auto& ss = si_->getStateSpace();
        for (unsigned int step = 1; step < n_interp+1; ++step) {
          // Interpolate state.
          auto new_state = ss->allocState();
          ss->interpolate(stateProperty_[m], stateProperty_[n], step * n_interp_div, new_state);
          // Check if valid.
          connection_valid &= si_->isValid(new_state);
          if (connection_valid) {
            // Add edge and continue.
            Vertex new_milestone = boost::add_vertex(g_);
            const Edge &e = boost::add_edge(prev_milestone, new_milestone, properties, g_).first;
            edgeValidityProperty_[e] = VALIDITY_UNKNOWN;
            stateProperty_[new_milestone] = new_state;
            vertexComponentProperty_[new_milestone] = vertexComponentProperty_[prev_milestone];
            vertexValidityProperty_[new_milestone] = VALIDITY_TRUE;
            nn_->add(new_milestone);
            prev_milestone = new_milestone;
          } else {
            // Edge invalid. Free state. Stop interpolate.
            ss->freeState(new_state);
            break;
          }
        }
        if (connection_valid) {
          // If full connection is valid. Add final edge.
          const Edge &e = boost::add_edge(prev_milestone, n, properties, g_).first;
          edgeValidityProperty_[e] = VALIDITY_UNKNOWN;
          uniteComponents(m, n);
        }
      } else {
        // We connect directly.
        const Edge &e = boost::add_edge(m, n, properties, g_).first;
        edgeValidityProperty_[e] = VALIDITY_UNKNOWN;
        uniteComponents(m, n);
      }

    }

  nn_->add(m);

  return m;
}



namespace
{
    struct AStarFoundGoal
    {
    };  // exception for termination

    // visitor that terminates when we find the goal
    // V is the vertex type
    template <typename V>
    class AStarGoalVisitor : public boost::default_astar_visitor
    {
    public:
        AStarGoalVisitor(const V &goal) : goal_(goal)
        {
        }

        // G is the graph type
        template <typename G>
        void examine_vertex(const V &u, const G & /*unused*/)
        {
            if (u == goal_)
                throw AStarFoundGoal();
        }

    private:
        V goal_;
    };
}

#endif

namespace ompl
{
    namespace magic
    {
        /** \brief The number of nearest neighbors to consider by
            default in the construction of the PRM roadmap */
        static const unsigned int DEFAULT_NEAREST_NEIGHBORS_LAZY = 5;

        /** \brief When optimizing solutions with lazy planners, this is the minimum
            number of path segments to add before attempting a new optimized solution
            extraction */
        static const unsigned int MIN_ADDED_SEGMENTS_FOR_LAZY_OPTIMIZATION = 5;
    }
}

ob::PlannerStatus PRMMotionCost::baseSolve(const ob::PlannerTerminationCondition &ptc)
{
    checkValidity();
    auto *goal = dynamic_cast<ob::GoalSampleableRegion *>(pdef_->getGoal().get());

    if (goal == nullptr)
    {
        OMPL_ERROR("%s: Unknown type of goal", getName().c_str());
        return ob::PlannerStatus::UNRECOGNIZED_GOAL_TYPE;
    }

    // Add the valid start states as milestones
    while (const ob::State *st = pis_.nextStart())
        startM_.push_back(addValidMilestone(si_->cloneState(st)));

    if (startM_.empty())
    {
        OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
        return ob::PlannerStatus::INVALID_START;
    }

    if (!goal->couldSample())
    {
        OMPL_ERROR("%s: Insufficient states in sampleable goal region", getName().c_str());
        return ob::PlannerStatus::INVALID_GOAL;
    }

    // Ensure there is at least one valid goal state
    if (goal->maxSampleCount() > goalM_.size() || goalM_.empty())
    {
        const ob::State *st = goalM_.empty() ? pis_.nextGoal(ptc) : pis_.nextGoal();
        if (st != nullptr)
            goalM_.push_back(addValidMilestone(si_->cloneState(st)));

        if (goalM_.empty())
        {
            OMPL_ERROR("%s: Unable to find any valid goal states", getName().c_str());
            return ob::PlannerStatus::INVALID_GOAL;
        }
    }

    // Compute cost for newly added edges.
    for (const Vertex v: startM_) {
      computeCostForVertexEdges(v);
    }
    for (const Vertex v: goalM_) {
      computeCostForVertexEdges(v);
    }

    unsigned long int nrStartStates = boost::num_vertices(g_);
    OMPL_INFORM("%s: Starting planning with %lu states already in datastructure", getName().c_str(), nrStartStates);

    bestCost_ = opt_->infiniteCost();
    ob::State *workState = si_->allocState();
    std::pair<std::size_t, std::size_t> startGoalPair;
    ob::PathPtr solution;

    const long int solComponent = solutionComponent(&startGoalPair);
    // If the start & goal are connected and we either did not find any solution
    // so far or the one we found still needs optimizing and we just added an edge
    // to the connected component that is used for the solution, we attempt to
    // construct a new solution.
    if (solComponent != -1)
    {
        Vertex startV = startM_[startGoalPair.first];
        Vertex goalV = goalM_[startGoalPair.second];
        do
        {
            solution = constructSolution(startV, goalV);
        } while (!solution && vertexComponentProperty_[startV] == vertexComponentProperty_[goalV]);
        if (solution)
        {
            // TODO: Find way to get actual cost from graph.
            //       Not functionally relevant, but would be nice to know.
            bestCost_ = opt_->identityCost();
        }
    }

    si_->freeState(workState);

    if (solution)
    {
        ob::PlannerSolution psol(solution);
        psol.setPlannerName(getName());
        // if the solution was optimized, we mark it as such
        psol.setOptimized(opt_, bestCost_, true);
        pdef_->addSolutionPath(psol);
    }

    OMPL_INFORM("%s: Created %u states", getName().c_str(), boost::num_vertices(g_) - nrStartStates);

    return solution ? ob::PlannerStatus::EXACT_SOLUTION : ob::PlannerStatus::TIMEOUT;
}



ompl::base::PathPtr PRMMotionCost::constructSolution(const Vertex &start, const Vertex &goal)
{

    // Need to update the index map here, becuse nodes may have been removed and
    // the numbering will not be 0 .. N-1 otherwise.
    unsigned long int index = 0;
    boost::graph_traits<Graph>::vertex_iterator vi, vend;
    for (boost::tie(vi, vend) = boost::vertices(g_); vi != vend; ++vi, ++index)
        indexProperty_[*vi] = index;

    boost::property_map<Graph, boost::vertex_predecessor_t>::type prev;
    try
    {
        // Consider using a persistent distance_map if it's slow
        boost::astar_search(g_, start,
                            [this, goal](Vertex v)
                            {
                                // TODO: Figure out why this returns identity cost.
                                //       But for the moment, identity cost is good.
                                return costHeuristic(v, goal);
                            },
                            boost::predecessor_map(prev)
                                .distance_compare([this](ob::Cost c1, ob::Cost c2)
                                                  {
                                                      return opt_->isCostBetterThan(c1, c2);
                                                  })
                                .distance_combine([this](ob::Cost c1, ob::Cost c2)
                                                  {
                                                      return opt_->combineCosts(c1, c2);
                                                  })
                                .distance_inf(opt_->infiniteCost())
                                .distance_zero(opt_->identityCost())
                                .visitor(AStarGoalVisitor<Vertex>(goal)));
    }
    catch (AStarFoundGoal &)
    {
    }

    if (prev[goal] == goal)
        throw ompl::Exception(name_, "Could not find solution path");

    // First, get the solution states without copying them, and check them for validity.
    // We do all the node validity checks for the vertices, as this may remove a larger
    // part of the graph (compared to removing an edge).
    std::vector<const ob::State *> states(1, stateProperty_[goal]);
    std::set<Vertex> milestonesToRemove;
    for (Vertex pos = prev[goal]; prev[pos] != pos; pos = prev[pos])
    {
        const ob::State *st = stateProperty_[pos];
        unsigned int &vd = vertexValidityProperty_[pos];
        if ((vd & VALIDITY_TRUE) == 0)
            if (si_->isValid(st))
                vd |= VALIDITY_TRUE;
        if ((vd & VALIDITY_TRUE) == 0)
            milestonesToRemove.insert(pos);
        if (milestonesToRemove.empty())
            states.push_back(st);
    }

    // We remove *all* invalid vertices. This is not entirely as described in the original LazyPRM
    // paper, as the paper suggest removing the first vertex only, and then recomputing the
    // shortest path. Howeve, the paper says the focus is on efficient vertex & edge removal,
    // rather than collision checking, so this modification is in the spirit of the paper.
    if (!milestonesToRemove.empty())
    {
        unsigned long int comp = vertexComponentProperty_[start];
        // Remember the current neighbors.
        std::set<Vertex> neighbors;
        for (auto it = milestonesToRemove.begin(); it != milestonesToRemove.end(); ++it)
        {
            boost::graph_traits<Graph>::adjacency_iterator nbh, last;
            for (boost::tie(nbh, last) = boost::adjacent_vertices(*it, g_); nbh != last; ++nbh)
                if (milestonesToRemove.find(*nbh) == milestonesToRemove.end())
                    neighbors.insert(*nbh);
            // Remove vertex from nearest neighbors data structure.
            nn_->remove(*it);
            // Free vertex state.
            si_->freeState(stateProperty_[*it]);
            // Remove all edges.
            boost::clear_vertex(*it, g_);
            // Remove the vertex.
            boost::remove_vertex(*it, g_);
        }
        // Update the connected component ID for neighbors.
        for (auto neighbor : neighbors)
        {
            if (comp == vertexComponentProperty_[neighbor])
            {
                unsigned long int newComponent = componentCount_++;
                componentSize_[newComponent] = 0;
                markComponent(neighbor, newComponent);
            }
        }
        return ob::PathPtr();
    }

    // start is checked for validity already
    states.push_back(stateProperty_[start]);

    // We intentionally still check the edges with discrete validity checking to make sure
    // the motion cost doesn't miss any geometry. This should be cheap since it's only for
    // the optimal path and the edges should almost always be valid.
    // Check the edges too, if the vertices were valid. Remove the first invalid edge only.
    std::vector<const ob::State *>::const_iterator prevState = states.begin(), state = prevState + 1;
    Vertex prevVertex = goal, pos = prev[goal];
    do
    {
        Edge e = boost::lookup_edge(pos, prevVertex, g_).first;
        const auto edge_cost = boost::get(boost::edge_weight_t(), g_, e);
        if (!opt_->isFinite(edge_cost)) {
          // Infinite cost is same as invalid edge. Path is not feasible.
          return ob::PathPtr();
        }
        unsigned int &evd = edgeValidityProperty_[e];
        if ((evd & VALIDITY_TRUE) == 0)
        {
            if (si_->checkMotion(*state, *prevState))
                evd |= VALIDITY_TRUE;
        }
        if ((evd & VALIDITY_TRUE) == 0)
        {
            boost::remove_edge(e, g_);
            unsigned long int newComponent = componentCount_++;
            componentSize_[newComponent] = 0;
            markComponent(pos, newComponent);
            return ob::PathPtr();
        }
        prevState = state;
        ++state;
        prevVertex = pos;
        pos = prev[pos];
    } while (prevVertex != pos);

    auto p(std::make_shared<og::PathGeometric>(si_));
    for (std::vector<const ob::State *>::const_reverse_iterator st = states.rbegin(); st != states.rend(); ++st)
        p->append(*st);
    return p;
}



void PRMMotionCost::sampleGraph() {
  maintainer_->sampleGraph();
}
