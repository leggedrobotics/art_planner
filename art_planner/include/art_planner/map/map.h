#pragma once

#include <functional>
#include <memory>
#include <mutex>

#include <grid_map_core/GridMap.hpp>

#include "art_planner/map/processors/chain.h"
#include "art_planner/params.h"



namespace art_planner {



using GridMapPtr = std::unique_ptr<grid_map::GridMap>;




class Map {

  GridMapPtr map_{new grid_map::GridMap()};

  processors::ChainNewMap map_pre_processor_;

  processors::ChainOldMap old_map_post_processor_;

  ParamsConstPtr params_;

  mutable std::mutex mutex_;

  // Returns a multiplication mask which enforces the maximum desired sample probability for unknown space.
  grid_map::Matrix computeUnknownProbabilityMask(const grid_map::Matrix& prob,
                                                 const grid_map::Matrix& valid_mask);

  // Compute the probability distribution for the sampler.
  void computeSamplerProbabilityDistribution();




public:

  // Constructs an empty map, without underlying grid map.
  Map(const ParamsConstPtr& params);

  // Sets the underlying grid map by taking ownership of it.
  void setMap(GridMapPtr&& map);

  // Copies the underlying map and parameters of another map object.
  void copy(const Map& map);



  // Checks whether position is inside of map bounds.
  inline bool isInside(const grid_map::Position& pos) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return map_->isInside(pos);
  }

  grid_map::Position getPositionOfIndex(const grid_map::Index& ind) const {
    std::lock_guard<std::mutex> lock(mutex_);
    grid_map::Position pos;
    map_->getPosition(ind, pos);
    return pos;
  }

  // Gets the height value at a certain map index.
  inline float getHeightAtIndex(const grid_map::Index& ind) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return map_->at(params_->planner.elevation_layer, ind);
  }

  // Gets the height value at a certain map position.
  // Might throw exceptions if position is outside map boundaries.
  inline float getHeightAtPosition(const grid_map::Position& pos) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return map_->atPosition(params_->planner.elevation_layer, pos);
  }

  // Return whether this cell was updated in the last map update.
  // Might throw exceptions if position is outside map boundaries.
  inline bool getUpdatedAtPosition(const grid_map::Position& pos) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return map_->atPosition("updated", pos) > std::numeric_limits<grid_map::DataType>::epsilon();
  }

  bool getUpdatedOnLine(const grid_map::Position& start, const grid_map::Position& end);

  // Gets the terrain normal at a certain map index.
  inline Eigen::Vector3d getNormal(const grid_map::Index& ind) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return Eigen::Vector3d(map_->at("normal_x", ind),
                           map_->at("normal_y", ind),
                           map_->at("normal_z", ind));
  }

  // Gets the map index of a certain position.
  // Might throw exception if position is out of bounds.
  inline grid_map::Index getIndexOfPosition(const grid_map::Position& pos) const {
    std::lock_guard<std::mutex> lock(mutex_);
    grid_map::Index ind;
    if (!map_->getIndex(pos, ind)) {
      throw std::out_of_range("Requested position is outside of map bounds.");
    }
    return ind;
  }

  // Gets the standard deviation of a plane fit error at the given map index.
  inline float getPlaneFitStdDev(const grid_map::Index& ind) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return map_->at("plane_fit_std_dev", ind);
  }

  inline const grid_map::Matrix& getLayer(const std::string& layer) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return map_->get(layer);
  }

  inline void addLayer(const std::string& layer_name, const grid_map::Matrix& data) {
    std::lock_guard<std::mutex> lock(mutex_);
    map_->add(layer_name, data);
  }

  inline void addLayer(const std::string& layer_name, const double val) {
    std::lock_guard<std::mutex> lock(mutex_);
    map_->add(layer_name, val);
  }

  inline Eigen::Index lengthToCellCount(const double& length) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return length / map_->getResolution();
  }

  void blurMapLayer(const std::string& layer, const double& filter_size);

  void gaussianBlurMapLayer(const std::string& layer,
                            const double& filter_size,
                            const double& std_dev);

  // Gets Z and roll, pitch values from the elevation map, given X, Y and yaw coordinates.
  void get3DPoseFrom2D(double* xyzrpy) const;

  void setNewMapProcessorChain(const processors::ChainNewMap& chain) {
    std::lock_guard<std::mutex> lock(mutex_);
    map_pre_processor_ = chain;
  }

  void setOldMapProcessorChain(const processors::ChainOldMap& chain) {
    std::lock_guard<std::mutex> lock(mutex_);
    old_map_post_processor_ = chain;
  }



  inline const grid_map::GridMap& getMap() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return *map_;
  }

};



}
