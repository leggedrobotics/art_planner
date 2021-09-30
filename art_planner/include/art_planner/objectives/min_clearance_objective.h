#pragma once

#include "art_planner/utils.h"

#include <ompl/base/objectives/MaximizeMinClearanceObjective.h>



namespace ob = ompl::base;

namespace art_planner {



class MinClearanceObjective : public ob::MaximizeMinClearanceObjective {

  using Base = ob::MaximizeMinClearanceObjective;

public:
  using Base::Base;

  virtual ob::Cost motionCost(const ob::State* s1, const ob::State* s2) const override;

};



}
