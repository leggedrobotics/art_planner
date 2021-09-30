#include "art_planner/map/map.h"
#include "art_planner/utils.h"

#include <random>

#include <grid_map_core/iterators/LineIterator.hpp>
#include <grid_map_core/SubmapGeometry.hpp>



using namespace art_planner;



void Map::setMap(GridMapPtr&& map_new) {
  map_new->setBasicLayers({params_->planner.elevation_layer, params_->planner.traversability_layer});
  map_pre_processor_.process(map_new);
  if (map_) {
    // Not locking mutex here should be fine because we only read from map_.
    old_map_post_processor_.process(map_new, map_);
  }
  std::lock_guard<std::mutex> lock(mutex_);
  const GridMapPtr map_old = std::move(map_);
  map_ = std::move(map_new);
}



void Map::copy(const Map& map) {
  std::lock_guard<std::mutex> lock(mutex_);
  *map_ = *map.map_;
  params_ = map.params_;
}



Map::Map(const ParamsConstPtr& params)
    : params_(params) {

}



bool Map::getUpdatedOnLine(const grid_map::Position& start, const grid_map::Position& end) {
  std::lock_guard<std::mutex> lock(mutex_);
  const auto updated = map_->get("updated");
  grid_map::LineIterator iter(*map_, start, end);
  for (; !iter.isPastEnd(); ++iter) {
    const auto ind = *iter;
    if (updated(ind.x(), ind.y()) > std::numeric_limits<grid_map::DataType>::epsilon()) return true;
  }
  return false;
}



void Map::blurMapLayer(const std::string& layer, const double& filter_size) {
  std::lock_guard<std::mutex> lock(mutex_);
  const int filter_size_cells = filter_size/map_->getResolution();
  map_->get(layer) = blurMatrix(map_->get(layer), filter_size_cells);
}



void Map::gaussianBlurMapLayer(const std::string& layer,
                               const double& filter_size,
                               const double& std_dev) {
  int filter_size_cells = lengthToCellCount(filter_size);
  const double std_dev_cells = lengthToCellCount(std_dev);
  if (filter_size_cells % 2 == 0) filter_size_cells += 1;
  std::lock_guard<std::mutex> lock(mutex_);
  map_->get(layer) = gaussianBlurMatrix(map_->get(layer), filter_size_cells, std_dev_cells);
}



void Map::get3DPoseFrom2D(double* xyzrpy) const {
  const auto ind = getIndexOfPosition(grid_map::Position(xyzrpy[0], xyzrpy[1]));

  // Get height.
  xyzrpy[2] = getHeightAtIndex(ind);

  // Get normal and use it to get roll, pitch.
  const Eigen::Vector3d normal_w = getNormal(ind);
  const Eigen::Quaterniond R_yaw(Eigen::AngleAxisd(xyzrpy[5],
                                                   Eigen::Vector3d::UnitZ()));
  const auto normal_b = R_yaw.inverse() * normal_w;
  xyzrpy[3] = -atan2(normal_b.y(), normal_b.z()); // Roll.
  xyzrpy[4] = atan2(normal_b.x(), normal_b.z()); // Pitch.
}
