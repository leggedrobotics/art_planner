#include "art_planner/map/processors/change.h"



using namespace art_planner::processors;



void art_planner::processors::computeChange(const art_planner::GridMapPtr& map_new,
                                            const art_planner::GridMapPtr& map_old,
                                            float height_change_for_update,
                                            const std::string& elevation_layer) {
  grid_map::Matrix updated = grid_map::Matrix::Ones(map_new->getSize().x(), map_new->getSize().y());

  bool success_new, success_old;
  grid_map::SubmapGeometry geom_new(*map_new,
                                    map_old->getPosition(),
                                    map_old->getLength(),
                                    success_new);
  grid_map::SubmapGeometry geom_old(*map_old,
                                    map_new->getPosition(),
                                    map_new->getLength(),
                                    success_old);
  grid_map::Size size;
  size.x() = geom_new.getSize().x() > geom_old.getSize().x() ? geom_old.getSize().x() : geom_new.getSize().x();
  size.y() = geom_new.getSize().y() > geom_old.getSize().y() ? geom_old.getSize().y() : geom_new.getSize().y();

  if (success_new && success_old) {
    for (Eigen::Index i = 0; i < size.x(); ++i) {
      for (Eigen::Index j = 0; j < size.y(); ++j) {
        const grid_map::Index ind_new = geom_new.getStartIndex() + grid_map::Index(i, j);
        const grid_map::Index ind_old = geom_old.getStartIndex() + grid_map::Index(i, j);
        // Check significant height change.
        const auto height_diff = map_new->at(elevation_layer, ind_new) - map_old->at(elevation_layer, ind_old);
        const bool height_changed = std::fabs(height_diff) > height_change_for_update;
        // Check if traversability has gone untraversable.
        const bool trav_changed = map_old->at("traversability_thresholded", ind_old)
            - map_new->at("traversability_thresholded", ind_new) > 0.5f;
        if (!height_changed && !trav_changed) {
          updated(ind_new.x(), ind_new.y()) = 0;
        }

      }
    }
  }

  // TODO: Fix noise issue at edges.
  // TODO: Dilate with appropriate radius when noise is fixed.

  map_new->add("updated", updated);
}
