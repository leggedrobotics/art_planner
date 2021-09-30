#include "art_planner/objectives/min_clearance_objective.h"



using namespace art_planner;



ob::Cost MinClearanceObjective::motionCost(const ob::State* s1, const ob::State* s2) const {
  return this->combineCosts(this->stateCost(s1), this->stateCost(s2));
}
