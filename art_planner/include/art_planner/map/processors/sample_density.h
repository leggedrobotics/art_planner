#pragma once

#include <ompl/base/Planner.h>

#include "art_planner/map/map.h"



namespace art_planner {

namespace processors {



void computeInverseSampleDensity(const art_planner::GridMapPtr& map,
                                 ompl::base::PlannerPtr planner,
                                 double blur_radius);



}

}
