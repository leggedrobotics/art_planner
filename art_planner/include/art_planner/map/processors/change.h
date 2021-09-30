#pragma once

#include "art_planner/map/map.h"



namespace art_planner {

namespace processors {



void computeChange(const GridMapPtr& map_new,
                   const GridMapPtr& map_old,
                   float height_change_for_update, const std::string &elevation_layer);



} // namespace processors

} // namespace art_planner
