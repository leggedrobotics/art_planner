#pragma once

#include <memory>
#include <mutex>

#include <grid_map_core/GridMap.hpp>

#include "height_map_box_checker.h"
#include "art_planner/map/map.h"
#include "art_planner/params.h"
#include "art_planner/utils.h"



namespace art_planner {



class ValidityCheckerFeet {

  ParamsConstPtr params_;

  using MapPtr = std::shared_ptr<Map>;

  MapPtr traversability_map_;
  std::unique_ptr<HeightMapBoxChecker> checker_;

  mutable std::mutex mutex_;

  float box_length_;
  float box_width_;

  bool boxIsValidAtPose(const Pose3& pose) const;

  bool boxesAreValidAtPoses(const std::vector<Pose3> &poses) const;

public:

  ValidityCheckerFeet(const ParamsConstPtr& params);

  void setMap(const MapPtr& map);

  bool isValid(const Pose3& pose) const;

  bool hasMap() const;

  void updateHeightField();

};



}
