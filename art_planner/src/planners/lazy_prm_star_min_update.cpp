#include "art_planner/planners/lazy_prm_star_min_update.h"

#include <algorithm>
#include <chrono>

#include <boost/foreach.hpp>
#include <boost/graph/lookup_edge.hpp>
#include <ompl/base/Planner.h>
#include <ompl/base/spaces/SE3StateSpace.h>



using namespace art_planner;
namespace og = ompl::geometric;



void LazyPRMStarMinUpdateMaintainer::invalidateUpdatedGraphComponents() {
  size_t counter = 0;
  BOOST_FOREACH (Vertex v, boost::vertices(p_->g_)) {
    unsigned int &vd = p_->vertexValidityProperty_[v];
    // If it is valid...
    if (vd & LazyPRMStarMinUpdate::VALIDITY_TRUE) {
      // ... and was updated ...
      if (wasUpdated(v)) {
        // ... mark vertex validity as unknown.
        vd = LazyPRMStarMinUpdate::VALIDITY_UNKNOWN;
        ++counter;
      }
    }
  }
  if (params_->verbose) {
    std::cout << "Invalidated " << counter << " vertices" << std::endl;
  }

  counter = 0;
  size_t counter_valid = 0;
  // TODO? Sort by length?
  // Invalidate edges passing through updated space.
  BOOST_FOREACH (const Edge e, boost::edges(p_->g_)) {
    unsigned int& ed = p_->edgeValidityProperty_[e];
    if (ed & LazyPRMStarMinUpdate::VALIDITY_TRUE) {
      ++counter_valid;
      if (wasUpdated(e)) {
        ed = LazyPRMStarMinUpdate::VALIDITY_UNKNOWN;
        ++counter;
      }
    }
  }

  if (params_->verbose) {
    std::cout << "Invalidated " << counter << "/" << counter_valid << " edges" << std::endl;
  }
}



void LazyPRMStarMinUpdateMaintainer::removeOutdatedVertices() {
  // Compute vertices which are out-of-bounds.
  std::set<Vertex> milestones_to_remove;
  BOOST_FOREACH (Vertex v, boost::vertices(p_->g_)) {
    if (isOutOfBounds(v)) {
      milestones_to_remove.insert(v);
    }
  }

  // Remove start and goal vertices since they will be re-added.
  milestones_to_remove.insert(std::begin(p_->startM_), std::end(p_->startM_));
  milestones_to_remove.insert(std::begin(p_->goalM_), std::end(p_->goalM_));

  p_->removeVertices(milestones_to_remove);
}



bool LazyPRMStarMinUpdateMaintainer::wasUpdated(const Vertex& v) const {
  const auto vertex_state = p_->stateProperty_[v]->as<ob::SE3StateSpace::StateType>();
  return map_->getUpdatedAtPosition(grid_map::Position(vertex_state->getX(),
                                                       vertex_state->getY()));
}



bool LazyPRMStarMinUpdateMaintainer::wasUpdated(const Edge& e) const {
  const auto source = boost::source(e, p_->g_);
  const auto target = boost::target(e, p_->g_);
  const auto source_state = p_->stateProperty_[source]->as<ob::SE3StateSpace::StateType>();
  const auto target_state = p_->stateProperty_[target]->as<ob::SE3StateSpace::StateType>();
  return map_->getUpdatedOnLine(grid_map::Position(source_state->getX(), source_state->getY()),
                                grid_map::Position(target_state->getX(), target_state->getY()));
}



bool LazyPRMStarMinUpdateMaintainer::isOutOfBounds(const Vertex& v) const {
  const auto vertex_state = p_->stateProperty_[v]->as<ob::SE3StateSpace::StateType>();
  return !map_->isInside(grid_map::Position(vertex_state->getX(),
                                            vertex_state->getY()));
}



LazyPRMStarMinUpdateMaintainer::LazyPRMStarMinUpdateMaintainer(const std::shared_ptr<Map> &map, const ParamsConstPtr& params)
  : params_(params), map_(map) {
  if (params_->planner.lazy_prm_star_min_update.cleanup_when_not_planning) {
    connection_checker_thread_ = std::thread(&LazyPRMStarMinUpdateMaintainer::connectionCheckerThread,
                                             this);
  }
}



LazyPRMStarMinUpdateMaintainer::~LazyPRMStarMinUpdateMaintainer() {
  can_clean_ = false;
  shutdown_requested_ = true;
  if (connection_checker_thread_.joinable()) {
    connection_checker_thread_.join();
  }
}



void LazyPRMStarMinUpdateMaintainer::update() {
  if (p_) {
    requestPauseCleaning();
    std::lock_guard<std::mutex> lock(p_->mutex_);

    removeOutdatedVertices();
    if (params_->planner.lazy_prm_star_min_update.invalidate_updated_graph_components) {
      invalidateUpdatedGraphComponents();
    }

    continueCleaning();
  }
}



void LazyPRMStarMinUpdateMaintainer::connectionCheckerThread() {
  std::random_device rd;  // Will be used to obtain a seed for the random number engine.
  std::mt19937 gen(rd()); // Standard mersenne_twister_engine seeded with rd().


  while (!shutdown_requested_) {
    if (p_ && can_clean_ && boost::num_vertices(p_->g_) > 0) {
      ob::ScopedState<ob::SE3StateSpace> work_state(p_->si_);

      auto start = std::chrono::high_resolution_clock::now();
      std::lock_guard<std::mutex> lock(p_->mutex_);


      // Plan random paths to find invalid shortcuts.
      size_t n_paths = 0;
      size_t n_pairs = 0;
      size_t n_samples = 0;
      size_t n_edges_removed = 0;
      Vertex vert2;
      bool valid_pair, found_solution;

      while (can_clean_) {
        // Sample new vertex.
        do {
          p_->sampler_->sampleUniform(work_state.get());
        } while (!p_->si_->isValid(work_state.get()));
        const auto vert1 = p_->addValidMilestone(p_->si_->cloneState(work_state.get()));
        ++n_samples;

        // Plan path between random vertices.
        const auto vertices = p_->getGraphVertices();
        if (vertices.size() > 1) {
          std::uniform_int_distribution<size_t> dis(0, vertices.size() - 1);
          valid_pair = false;
          found_solution = false;

          // Use the start vertex if we have one.
          if (!p_->startM_.empty()) {
            vert2 = p_->startM_[0];
            valid_pair = true;
          }
          // Sample a random vertices if we couldn't find start vertex.
          while (!valid_pair && can_clean_) {
            vert2 = vertices[dis(gen)];
            // Vertices shouldn't be the same.
            valid_pair = vert1 != vert2;
          }
          // Vertices should belong to same component.
          if (valid_pair && can_clean_) {
            valid_pair &= p_->vertexComponentProperty_[vert1] == p_->vertexComponentProperty_[vert2];
          }
          ++n_pairs;

          // Plan path between two vertices until we have found solution or vertices are not same component.
          while (!found_solution && valid_pair && can_clean_) {
            const auto n_edges_before = boost::num_edges(p_->g_);
            found_solution = static_cast<bool>(p_->constructSolution(vert1, vert2));
            n_edges_removed += n_edges_before - boost::num_edges(p_->g_);
            ++n_paths;
            // Check if vertices are still same component after possibly altering graph.
            valid_pair &= p_->vertexComponentProperty_[vert1] == p_->vertexComponentProperty_[vert2];
          }
        }

      }

      auto end = std::chrono::high_resolution_clock::now();
      std::chrono::duration<double> elapsed_seconds = end-start;

      if (params_->verbose) {
        std::cout << "Cleaned up for " << std::setprecision(3) <<  elapsed_seconds.count()*1000.0 << "ms" << std::endl;
        std::cout << "Added " << n_samples << " new milestones." << std::endl;
        std::cout << "Planned " << n_paths << " paths between " << n_pairs << " pairs and removed " << n_edges_removed << " edges" << std::endl;
      }
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
}



void LazyPRMStarMinUpdate::setMaintainer(std::unique_ptr<LazyPRMStarMinUpdateMaintainer>&& maintainer) {
  maintainer_ = std::move(maintainer);
  maintainer_->setPlanner(this);
}



void LazyPRMStarMinUpdate::clear() {
  ob::Planner::clear();
//  freeMemory();
//  if (nn_)
//      nn_->clear();
  updateVertices();

  clearQuery();

//  componentCount_ = 0;
  iterations_ = 0;
  bestCost_ = ob::Cost(std::numeric_limits<double>::quiet_NaN());
}



void LazyPRMStarMinUpdate::setup() {
  og::LazyPRM::setup();

  sampler_ = si_->allocStateSampler();
}



void LazyPRMStarMinUpdate::removeVertices(const std::set<Vertex>& milestones_to_remove) {
  if (milestones_to_remove.empty()) return;

  // Remove vertices from graph.
  std::set<Vertex> neighbors;
  std::set<unsigned long int> comps;

  for (auto it = milestones_to_remove.begin(); it != milestones_to_remove.end(); ++it) {
    // Get component of removed vertex.
    comps.insert(vertexComponentProperty_[*it]);
    // Search for neighbors, for which we need to update the component.
    boost::graph_traits<Graph>::adjacency_iterator nbh, last;
    for (boost::tie(nbh, last) = boost::adjacent_vertices(*it, g_); nbh != last; ++nbh)
        if (milestones_to_remove.find(*nbh) == milestones_to_remove.end())
            neighbors.insert(*nbh);

    // Free vertex state.
    si_->freeState(stateProperty_[*it]);
    // Remove all edges.
    boost::clear_vertex(*it, g_);
    // Remove the vertex.
    boost::remove_vertex(*it, g_);
  }

  // Clear nn structure and re-add all vertices.
  // This is faster than removing vertices individually.
  if (nn_) {
    nn_->clear();
    std::vector<Vertex> vertices;
    BOOST_FOREACH (Vertex v, boost::vertices(g_)) {
      vertices.push_back(v);
    }
    nn_->add(vertices);
  }

  // Update the connected component ID for neighbors which were part of the same component.
  for (auto neighbor : neighbors) {
    // We only need to update component if neighbor was part of the same component
    // as the removed vertex.
    if (comps.find(vertexComponentProperty_[neighbor]) != comps.end()) {
      const unsigned long int newComponent = componentCount_++;
      componentSize_[newComponent] = 0;
      markComponent(neighbor, newComponent);
    }
  }
}



void LazyPRMStarMinUpdate::updateVertices() {
  maintainer_->update();
}




bool LazyPRMStarMinUpdate::checkIfValid(const Vertex& v) {
  const ob::State *st = stateProperty_[v];
  unsigned int &vd = vertexValidityProperty_[v];

  if ((vd & VALIDITY_TRUE) == 0) {
    const auto result = si_->isValid(st);
    if (result) {
      vd |= VALIDITY_TRUE;
      return true;
    } else {
      return false;
    }
  } else {
    return false;
  }
}



std::vector<LazyPRMStarMinUpdate::Vertex> LazyPRMStarMinUpdate::getGraphVertices() const {
  std::vector<Vertex> vertices(boost::num_vertices(g_));
  size_t i = 0;
  BOOST_FOREACH (Vertex v, boost::vertices(g_)) {
    vertices[i++] = v;
  }
  return vertices;
}



std::vector<LazyPRMStarMinUpdate::Vertex> LazyPRMStarMinUpdate::getGraphVerticesInRandomOrder() const {
  std::vector<Vertex> vertices = getGraphVertices();
  std::random_shuffle(vertices.begin(), vertices.end());
  return vertices;
}



void LazyPRMStarMinUpdate::getPlannerData(ob::PlannerData &data) const {
  if (maintainer_) {
    maintainer_->requestPauseCleaning();
  }
  std::lock_guard<std::mutex> lock(mutex_);

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

    if (edgeValidityProperty_[e] & VALIDITY_TRUE) {
      data.addEdge(ob::PlannerDataVertex(stateProperty_[v1]), ob::PlannerDataVertex(stateProperty_[v2]));
      // Add the reverse edge, since we're constructing an undirected roadmap
      data.addEdge(ob::PlannerDataVertex(stateProperty_[v2]), ob::PlannerDataVertex(stateProperty_[v1]));
    }

  }

  if (maintainer_) {
    maintainer_->continueCleaning();
  }
}



ob::PlannerStatus LazyPRMStarMinUpdate::solve(const ob::PlannerTerminationCondition &ptc) {
  if (maintainer_) {
    maintainer_->requestPauseCleaning();
  }
  std::lock_guard<std::mutex> lock(mutex_);

  ob::PlannerStatus status = baseSolve(ptc);

  if (maintainer_) {
    maintainer_->continueCleaning();
  }

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



og::LazyPRM::Vertex LazyPRMStarMinUpdate::addValidMilestone(ob::State *state) {
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
      const ob::Cost weight = opt_->motionCost(stateProperty_[m], stateProperty_[n]);
      const Graph::edge_property_type properties(weight);
      const Edge &e = boost::add_edge(m, n, properties, g_).first;
      edgeValidityProperty_[e] = VALIDITY_UNKNOWN;
      uniteComponents(m, n);
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

ob::PlannerStatus LazyPRMStarMinUpdate::baseSolve(const ob::PlannerTerminationCondition &ptc)
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

    unsigned long int nrStartStates = boost::num_vertices(g_);
    OMPL_INFORM("%s: Starting planning with %lu states already in datastructure", getName().c_str(), nrStartStates);

    bestCost_ = opt_->infiniteCost();
    ob::State *workState = si_->allocState();
    std::pair<std::size_t, std::size_t> startGoalPair;
    ob::PathPtr bestSolution;
    bool fullyOptimized = false;
    bool someSolutionFound = false;
    unsigned int optimizingComponentSegments = 0;

    // Grow roadmap in lazy fashion -- add vertices and edges without checking validity
    while (!ptc)
    {
        ++iterations_;
        do {
          sampler_->sampleUniform(workState);
        } while (!si_->isValid(workState));
        Vertex addedVertex = addValidMilestone(si_->cloneState(workState));

        const long int solComponent = solutionComponent(&startGoalPair);
        // If the start & goal are connected and we either did not find any solution
        // so far or the one we found still needs optimizing and we just added an edge
        // to the connected component that is used for the solution, we attempt to
        // construct a new solution.
        if (solComponent != -1 &&
            (!someSolutionFound || (long int)vertexComponentProperty_[addedVertex] == solComponent))
        {
            // If we already have a solution, we are optimizing. We check that we added at least
            // a few segments to the connected component that includes the previously found
            // solution before attempting to construct a new solution.
            if (someSolutionFound)
            {
                if (++optimizingComponentSegments < ompl::magic::MIN_ADDED_SEGMENTS_FOR_LAZY_OPTIMIZATION)
                    continue;
                optimizingComponentSegments = 0;
            }
            Vertex startV = startM_[startGoalPair.first];
            Vertex goalV = goalM_[startGoalPair.second];
            ob::PathPtr solution;
            do
            {
                solution = constructSolution(startV, goalV);
            } while (!solution && vertexComponentProperty_[startV] == vertexComponentProperty_[goalV]);
            if (solution)
            {
                someSolutionFound = true;
                ob::Cost c = solution->cost(opt_);
                if (opt_->isSatisfied(c))
                {
                    fullyOptimized = true;
                    bestSolution = solution;
                    bestCost_ = c;
                    break;
                }
                if (opt_->isCostBetterThan(c, bestCost_))
                {
                    bestSolution = solution;
                    bestCost_ = c;
                }
            }
        }
    }

    si_->freeState(workState);

    if (bestSolution)
    {
        ob::PlannerSolution psol(bestSolution);
        psol.setPlannerName(getName());
        // if the solution was optimized, we mark it as such
        psol.setOptimized(opt_, bestCost_, fullyOptimized);
        pdef_->addSolutionPath(psol);
    }

    OMPL_INFORM("%s: Created %u states", getName().c_str(), boost::num_vertices(g_) - nrStartStates);

    return bestSolution ? ob::PlannerStatus::EXACT_SOLUTION : ob::PlannerStatus::TIMEOUT;
}



ompl::base::PathPtr LazyPRMStarMinUpdate::constructSolution(const Vertex &start, const Vertex &goal)
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


    // Check the edges too, if the vertices were valid. Remove the first invalid edge only.
    std::vector<const ob::State *>::const_iterator prevState = states.begin(), state = prevState + 1;
    Vertex prevVertex = goal, pos = prev[goal];
    do
    {
        Edge e = boost::lookup_edge(pos, prevVertex, g_).first;
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
