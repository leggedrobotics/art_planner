#pragma once

#include <ompl/base/ScopedState.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/base/spaces/SO3StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

#include "art_planner/map/map.h"
#include "art_planner/params.h"



namespace ob = ompl::base;

namespace art_planner {



class SE3FromSE2Sampler : public ob::StateSampler {

  // Parameters.
  ParamsConstPtr params_;

  ob::StateSpacePtr space_real_vec_;
  ob::StateSpacePtr space_rot_;

  ob::ScopedState<ob::RealVectorStateSpace> state_pos_;
  ob::ScopedState<ob::SO2StateSpace> state_rot_;

  ob::ScopedState<ob::RealVectorStateSpace> near_pos_;
  ob::ScopedState<ob::SO2StateSpace> near_rot_;

  ob::RealVectorStateSampler base_real_vec_;
  ob::SO2StateSampler base_rot_;

  std::shared_ptr<Map> map_;

  grid_map::Position samplePositionInMap();

  grid_map::Position samplePositionInMapFromDist();

public:

  SE3FromSE2Sampler(const ob::StateSpace* si,
                    const std::shared_ptr<Map>& map,
                    const ParamsConstPtr& params);

  virtual void sampleUniform(ob::State* state) override;

  virtual void sampleUniformNear(ob::State* state,
                                 const ob::State* near,
                                 double distance) override;

  virtual void sampleGaussian(ob::State* state,
                              const ob::State* mean,
                              double std_dev) override;

};



class SE3FromSE2SamplerAllocator {

  // Parameters.
  ParamsConstPtr params_;

  std::shared_ptr<Map> map_;

public:

  SE3FromSE2SamplerAllocator(const ParamsConstPtr& params) :
    params_(params) {}

  void setMap(const std::shared_ptr<Map>& map);

  std::shared_ptr<SE3FromSE2Sampler> getSampler(const ob::StateSpace* space);


};



}
