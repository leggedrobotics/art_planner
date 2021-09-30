#pragma once

#include "art_planner/map/map.h"



namespace art_planner {

namespace processors {



void applyBaseSampleDistribution(const art_planner::GridMapPtr& map);

void computeCumulativeProbabilityDistribution(const art_planner::GridMapPtr& map);

void applyMaxUnknownProbability(const art_planner::GridMapPtr& map,
                                double max_prob_unknown_samples);



}

}
