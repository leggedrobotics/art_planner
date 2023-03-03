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

  // Compute safety features.
  const auto mapsize = map->getSize();
  grid_map::Matrix trav_filter =
      (map->get(params_->planner.traversability_layer).array() > params_->planner.traversability_thres).select(Eigen::MatrixXf::Ones(mapsize.x(), mapsize.y()),
                                                                                                               Eigen::MatrixXf::Zero(mapsize.x(), mapsize.y()));

  map->add("traversability_thresholded_no_safety", trav_filter);

  // Removes "safe" patches which are smaller in diameter than foothold_size.
  const auto foothold_size = std::ceil(params_->planner.safety.foothold_size / map->getResolution());
//  grid_map::Matrix trav_filter_saftey = erodeAndDilateMatrix(trav_filter, foothold_size);
//  trav_filter = (trav_filter.array() < 0.5f).select(trav_filter, trav_filter_saftey);  // Make sure we don't validate unsafe things.

  // Erodes traversable edges, unless they are on a small hole, without large elevation change.
  const auto safety_margin = std::ceil(2 * params_->planner.safety.foothold_margin / map->getResolution());
  const auto hole_size = std::floor(params_->planner.safety.foothold_margin_max_hole_size / map->getResolution());
  grid_map::Matrix trav_filter_saftey = dilateAndErodeMatrix(trav_filter, hole_size);  // Close holes.

  // Compute height difference.
  const grid_map::Matrix elevation = map->get(params_->planner.elevation_layer);
  const auto safety_search_radius = std::ceil(2 * params_->planner.safety.foothold_margin_max_drop_search_radius / map->getResolution());
  const grid_map::Matrix diff_low = elevation - erodeMatrix(elevation, safety_search_radius);
  const auto hole_mask = diff_low.array() > params_->planner.safety.foothold_margin_max_drop;
  trav_filter_saftey = (hole_mask).select(trav_filter, trav_filter_saftey);  // Make sure we don't validate unsafe things.
  map->add("diff_low_mask", (hole_mask).select(Eigen::MatrixXf::Ones(mapsize.x(), mapsize.y()),
                                               Eigen::MatrixXf::Zero(mapsize.x(), mapsize.y())));

  // Don't erode if we have walls, which means stepping close to them is safe.
  const grid_map::Matrix diff_high = dilateMatrix(elevation, safety_margin) - elevation;
  const auto wall_mask = (diff_high.array() > params_->planner.safety.foothold_margin_min_step /*&& !hole_mask*/);
  trav_filter_saftey = (wall_mask).select(Eigen::MatrixXf::Ones(mapsize.x(), mapsize.y()), trav_filter_saftey);  // Make sure we don't validate unsafe things.
  map->add("diff_high_mask", (wall_mask).select(Eigen::MatrixXf::Ones(mapsize.x(), mapsize.y()),
                                                Eigen::MatrixXf::Zero(mapsize.x(), mapsize.y())));


  trav_filter_saftey = erodeMatrix(trav_filter_saftey, safety_margin);  // Erode.
  // Make sure we don't validate unsafe things and remove erosion along walls.
  trav_filter_saftey = (trav_filter.array() < 0.5f || wall_mask).select(trav_filter, trav_filter_saftey);

  // Finally, remove any new small valid patches.
  trav_filter_saftey = erodeAndDilateMatrix(trav_filter_saftey, foothold_size);
  trav_filter_saftey = (trav_filter.array() < 0.5f).select(trav_filter, trav_filter_saftey);  // Make sure we don't validate unsafe things.

  map->add("traversability_thresholded", trav_filter_saftey);

  // Set untraversable region very low, so that feet are never in collision.
  map->add("elevation_masked", -std::numeric_limits<float>::infinity());
  map->get("elevation_masked") =
      (trav_filter_saftey.array() > 0.5).select(map->get(params_->planner.elevation_layer),
                                                         map->get("elevation_masked"));
}



void Basic::setTraversabilityFilter(const GridMapPtr& map) {
  // Compute traversability sampling mask.

  auto trav_filter = map->get("traversability_thresholded");

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


