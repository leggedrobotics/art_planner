#include "art_planner/sampler.h"

#include <iostream>

#include "art_planner/utils.h"



using namespace art_planner;



SE3FromSE2Sampler::SE3FromSE2Sampler(const ob::StateSpace *space,
                                     const std::shared_ptr<Map> &map,
                                     const ParamsConstPtr &params)
    : ob::StateSampler(space),
      params_(params),
      space_real_vec_(new ob::RealVectorStateSpace(3)),
      space_rot_(new ob::SO2StateSpace()),
      state_pos_(space_real_vec_),
      state_rot_(space_rot_),
      near_pos_(space_real_vec_),
      near_rot_(space_rot_),
      base_real_vec_(space_real_vec_.get()),
      base_rot_(space_rot_.get()),
      map_(map) {
  // Set SE2 bounds from SE3.
  const auto bounds_se3 = space_->as<ob::SE3StateSpace>()->getBounds();
//  std::cout << "Lower SE3 position bounds: " << bounds_se3.low[0] << "\t"
//                                             << bounds_se3.low[1] << "\t"
//                                             << bounds_se3.low[2] << std::endl;
//  std::cout << "Upper SE3 position bounds: " << bounds_se3.high[0] << "\t"
//                                             << bounds_se3.high[1] << "\t"
//                                             << bounds_se3.high[2] << std::endl;
  space_real_vec_->as<ob::RealVectorStateSpace>()->setBounds(bounds_se3);
}



grid_map::Position SE3FromSE2Sampler::samplePositionInMap() {
  grid_map::Position pos;

  // Keep sampling until we hit something inside map.
  // TODO: Do this more efficiently.
  do {
    base_real_vec_.sampleUniform(state_pos_.get());
    pos.x() = state_pos_.get()->values[0];
    pos.y() = state_pos_.get()->values[1];
  } while (!map_->isInside(pos));

  return pos;
}



grid_map::Position SE3FromSE2Sampler::samplePositionInMapFromDist() {
  // Sample uniform values.
  const auto samp_col = rng_.uniform01();
  const auto samp_row = rng_.uniform01();

  Eigen::Index col, row;

  const auto& cum_prob = map_->getLayer("cum_prob");
  const Eigen::Matrix<grid_map::DataType, Eigen::Dynamic, 1> cum_prob_rowwise =
      map_->getLayer("cum_prob_rowwise_hack").col(0);

  // Apply distribution
  for (row = 0; row < cum_prob_rowwise.rows()-1; ++row) {
    if (cum_prob_rowwise(row, 0) > samp_row) break;
  }
  for (col = 0; col < cum_prob.cols()-1; ++col) {
    if (cum_prob(row, col) > samp_col) break;
  }

  return map_->getPositionOfIndex(grid_map::Index(row, col));

  // TODO: Sample continuous value inside sampled cell.
}



void SE3FromSE2Sampler::sampleUniform(ob::State* state) {
  auto state_se3 = state->as<ob::SE3StateSpace::StateType>();

  // Update z value with height from map.
  grid_map::Position pos;
  if (params_->sampler.sample_from_distribution) {
    pos = samplePositionInMapFromDist();
  } else {
    pos = samplePositionInMap();
  }

  // Get index of position to slightly speed up next operations.
  const auto ind = map_->getIndexOfPosition(pos);

  state_pos_.get()->values[0] = pos.x();
  state_pos_.get()->values[1] = pos.y();
  state_pos_.get()->values[2] = map_->getHeightAtIndex(ind);

  // Apply small random perturbation in normal direction.
  const Eigen::Vector3d normal_w = map_->getNormal(ind);

  const auto std = map_->getPlaneFitStdDev(ind);

  const auto pert = rng_.uniformReal(-1, 1) * std::min(std, 0.5f) * params_->robot.feet.reach.z;

  state_pos_.get()->values[0] += normal_w.x() * pert;
  state_pos_.get()->values[1] += normal_w.y() * pert;
  state_pos_.get()->values[2] += normal_w.z() * pert;

  state_se3->setX(state_pos_.get()->values[0]);
  state_se3->setY(state_pos_.get()->values[1]);
  state_se3->setZ(state_pos_.get()->values[2]);

  // Sample rotation.
  rng_.eulerRPY(state_pos_.get()->values);  // Abuse position to avoid memory allocation.

  // Get roll and pitch from elevation map.

  const Eigen::Quaterniond R_wb(Eigen::AngleAxisd(state_pos_.get()->values[2],
                                                  Eigen::Vector3d::UnitZ()));

  const auto normal_b = R_wb.inverse() * normal_w;

  state_pos_.get()->values[0] = -atan2(normal_b.y(), normal_b.z())
                                + state_pos_.get()->values[0] * params_->sampler.max_roll_pert / M_PI_2; // Roll.
  state_pos_.get()->values[1] = atan2(normal_b.x(), normal_b.z())
                                + state_pos_.get()->values[1] * params_->sampler.max_pitch_pert / M_PI_4; // Pitch.

  setSO3FromRPY(state_se3->rotation(), state_pos_.get()->values);
}



void SE3FromSE2Sampler::sampleUniformNear(ob::State* state,
                                          const ob::State* near,
                                          double distance) {
  auto state_se3 = state->as<ob::SE3StateSpace::StateType>();
  auto near_se3 = near->as<ob::SE3StateSpace::StateType>();

  near_pos_->values[0] = near_se3->getX();
  near_pos_->values[1] = near_se3->getY();
  near_pos_->values[2] = near_se3->getZ();

  near_rot_->value = getYawFromSO3(state_se3->rotation());

  // Sample position.
  base_real_vec_.sampleUniformNear(state_pos_.get(), near_pos_.get(), distance);
  state_se3->setX(state_pos_.get()->values[0]);
  state_se3->setY(state_pos_.get()->values[1]);
  state_se3->setZ(state_pos_.get()->values[2]);

  // Sample rotation.
  base_rot_.sampleUniformNear(state_rot_.get(), near_rot_.get(), distance);
  setSO3FromYaw(state_se3->rotation(), state_rot_->value);
}



void SE3FromSE2Sampler::sampleGaussian(ob::State* state,
                                       const ob::State* mean,
                                       double std_dev) {
  auto state_se3 = state->as<ob::SE3StateSpace::StateType>();
  auto near_se3 = mean->as<ob::SE3StateSpace::StateType>();

  near_pos_->values[0] = near_se3->getX();
  near_pos_->values[1] = near_se3->getY();
  near_pos_->values[2] = near_se3->getZ();

  near_rot_->value = getYawFromSO3(state_se3->rotation());

  // Sample position.
  base_real_vec_.sampleGaussian(state_pos_.get(), near_pos_.get(), std_dev);
  state_se3->setX(state_pos_.get()->values[0]);
  state_se3->setY(state_pos_.get()->values[1]);
  state_se3->setZ(state_pos_.get()->values[2]);

  // Sample rotation.
  base_rot_.sampleGaussian(state_rot_.get(), near_rot_.get(), std_dev);
  setSO3FromYaw(state_se3->rotation(), state_rot_->value);
}



void SE3FromSE2SamplerAllocator::setMap(const std::shared_ptr<Map> &map) {
  map_ = map;
}



std::shared_ptr<SE3FromSE2Sampler> SE3FromSE2SamplerAllocator::getSampler(const ob::StateSpace* space) {
  return std::make_shared<SE3FromSE2Sampler>(space, map_, params_);
}
