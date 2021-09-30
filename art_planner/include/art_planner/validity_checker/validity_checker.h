#pragma once

#include "validity_checker_feet.h"
#include "validity_checker_body.h"

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/State.h>

#include "art_planner/map/map.h"



namespace ob = ompl::base;

namespace art_planner {



class StateValidityChecker : public ob::StateValidityChecker {

  ValidityCheckerFeet checker_feet_;
  ValidityCheckerBody checker_body_;

  ParamsConstPtr params_;

public:
   StateValidityChecker(const ob::SpaceInformationPtr &si,
                        const ParamsConstPtr& params);

   void setMap(const std::shared_ptr<Map>& map);

   void updateHeightField();

   bool hasMap() const;

   virtual bool isValid(const ob::State *state) const override;

   virtual double clearance(const ob::State *state) const override;
};



}
