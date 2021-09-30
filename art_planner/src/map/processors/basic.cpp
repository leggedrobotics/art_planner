#include "art_planner/map/processors/basic.h"

#include <iostream>

#include "art_planner/utils.h"



using namespace art_planner::processors;



void Basic::checkTraversability(const GridMapPtr& map) {
  if (!map->exists(params_->planner.traversability_layer)) {
    // Add traversability layers.
    map->add(params_->planner.traversability_layer, 1.0);
    if (params_->verbose) {
      std::cout << "Grid map does not have \"traversability\" layer. Assuming traversable." << std::endl;
    }
  }
}



void Basic::addKnownCells(const GridMapPtr& map) {
  // Remember which cells are known.
  const auto size = map->getSize();
  grid_map::Matrix known_cells(size.x(), size.y());
  known_cells.setZero();
  for (Eigen::Index i = 0; i < size.x(); ++i) {
    for (Eigen::Index j = 0; j < size.y(); ++j) {
      if (map->isValid(grid_map::Index(i, j))) {
        known_cells(i, j) = 1.0f;
      }
    }
  }
  map->add("observed", known_cells);
}



void Basic::setMaskedElevationAndTraversability(const GridMapPtr& map) {
  // Inpaint traversability and elevation.
  map->get(params_->planner.traversability_layer) = inpaintMatrix(map->get(params_->planner.traversability_layer));
  map->get(params_->planner.elevation_layer) = inpaintMatrix(map->get(params_->planner.elevation_layer));

  estimateNormals(*map, (params_->robot.torso.length + params_->robot.torso.width)*0.25, params_->planner.elevation_layer);

  if (params_->planner.unknown_space_untraversable) {
    const auto size = map->getSize();
    map->get(params_->planner.traversability_layer) =
        (map->get("observed").array() > 0.5f).select(map->get(params_->planner.traversability_layer),
                                                     grid_map::Matrix::Zero(size.x(), size.y()));
  }

  // Set untraversable region very low, so that feet are never in collision.
  map->add("elevation_masked", -std::numeric_limits<float>::infinity());
  map->get("elevation_masked") =
      (map->get(params_->planner.traversability_layer).array()
       > params_->planner.traversability_thres).select(map->get(params_->planner.elevation_layer),
                                                       map->get("elevation_masked"));
}



void Basic::setTraversabilityFilter(const GridMapPtr& map) {
  // Compute traversability sampling mask.
  const auto mapsize = map->getSize();
  grid_map::Matrix trav_filter =
      (map->get(params_->planner.traversability_layer).array() > params_->planner.traversability_thres).select(Eigen::MatrixXf::Ones(mapsize.x(), mapsize.y()),
                                                                                                                     Eigen::MatrixXf::Zero(mapsize.x(), mapsize.y()));

  map->add("traversability_thresholded", trav_filter);

  // We can step over small untraversable obstacles, so remove them.
  const double total_reach = std::sqrt(params_->robot.feet.reach.x*params_->robot.feet.reach.x +
                                       params_->robot.feet.reach.y*params_->robot.feet.reach.y);
  trav_filter = dilateAndErodeMatrix(trav_filter, total_reach / map->getResolution());
  // We can not step too close to the wall.
  const double min_wall_dist = std::min((params_->robot.torso.length - params_->robot.feet.reach.x)*0.5,
                                        (params_->robot.torso.width - params_->robot.feet.reach.y)*0.5);
  trav_filter = erodeMatrix(trav_filter, min_wall_dist / map->getResolution());

  map->add("traversability_sample_filter", trav_filter);
}



Basic::Basic(const ParamsConstPtr& params) : params_(params) {

}



void Basic::operator()(const GridMapPtr& map) {
  map->convertToDefaultStartIndex();

  checkTraversability(map);
  addKnownCells(map);
  setMaskedElevationAndTraversability(map);
  setTraversabilityFilter(map);

}


