#pragma once

#include "art_planner/map/map.h"



namespace art_planner {

namespace processors {



class Basic {

  ParamsConstPtr params_;

  void checkTraversability(const GridMapPtr& map);

  void addKnownCells(const GridMapPtr& map);

  void setMaskedElevationAndTraversability(const GridMapPtr& map);

  void setTraversabilityFilter(const GridMapPtr& map);

public:

  Basic(const ParamsConstPtr& params);

  void operator()(const GridMapPtr& map);

};



} // namespace processors

} // namespace art_planner
