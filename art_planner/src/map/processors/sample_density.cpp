#include "art_planner/map/processors/sample_density.h"

#include "art_planner/utils.h"



using namespace art_planner::processors;




void art_planner::processors::computeInverseSampleDensity(const art_planner::GridMapPtr& map,
                                                          ompl::base::PlannerPtr planner,
                                                          double blur_radius) {
  ob::PlannerData dat(planner->getSpaceInformation());
  planner->getPlannerData(dat);

  const auto n_vertices = dat.numVertices();
  grid_map::Index ind;

  map->add("n_samples", 0.0);
  auto& n_samples = map->get("n_samples");

  for (size_t i = 0; i < n_vertices; ++i) {
    const auto vert = dat.getVertex(i);
    const auto state = vert.getState()->as<ob::SE3StateSpace::StateType>();

    if (map->getIndex(grid_map::Position(state->getX(), state->getY()), ind)) {
      map->at("n_samples", ind) += 1;
    }
  }

  int filter_size_cells = 6*blur_radius / map->getResolution();
  const double std_dev_cells = blur_radius / map->getResolution();
  if (filter_size_cells % 2 == 0) filter_size_cells += 1;
  n_samples = art_planner::gaussianBlurMatrix(n_samples,
                                              filter_size_cells,
                                              std_dev_cells);
  if (!n_samples.isZero()) {
    // applyBaseSampleDistribution will add a uniform distribution if this is not executed.
    map->add("sample_probability", n_samples.maxCoeff() - n_samples.array());
  }
}
