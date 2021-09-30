#pragma once

#include <memory>
#include <mutex>

#include <grid_map_core/GridMap.hpp>

#include "height_map_box_checker.h"
#include "art_planner/map/map.h"
#include "art_planner/params.h"
#include "art_planner/utils.h"



namespace art_planner {



class ValidityCheckerBody {
  using MapPtr = std::shared_ptr<Map>;

  ParamsConstPtr params_;

  MapPtr elevation_map_;
  std::unique_ptr<HeightMapBoxChecker> checker_;
  mutable std::mutex mutex_;

public:

  ValidityCheckerBody(const ParamsConstPtr& params);

  void setMap(const MapPtr& map);

  bool isValid(const Pose3& pose) const;

  bool hasMap() const;

  void updateHeightField();

};



}
