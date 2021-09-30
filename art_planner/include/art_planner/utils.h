# pragma once

#include <memory>
#include <vector>

#include <Eigen/Dense>
#include <grid_map_core/GridMap.hpp>
#include <ompl/base/spaces/SE3StateSpace.h>


namespace ob = ompl::base;

namespace art_planner {



using Scalar = float;



using Pose3 = Eigen::Transform<Scalar,3,Eigen::Affine>;



inline Pose3 Pose3FromSE3(const ob::State* state) {
  const auto state_se3 = state->as<ob::SE3StateSpace::StateType>();
  const auto& rotation = state_se3->rotation();
  Pose3 pose;
  pose.translation().x() = state_se3->getX();
  pose.translation().y() = state_se3->getY();
  pose.translation().z() = state_se3->getZ();
  pose.matrix().topLeftCorner(3,3) =
      Eigen::Quaternion<Scalar>(rotation.w,
                                rotation.x,
                                rotation.y,
                                rotation.z).toRotationMatrix();
  return pose;
}



inline Pose3 Pose3FromXYZ(Scalar x, Scalar y, Scalar z) {
  Pose3 pose = Pose3::Identity();
  pose.translation().x() = x;
  pose.translation().y() = y;
  pose.translation().z() = z;
  return pose;
}



template <typename T>
inline T getRollFromQuat(T w, T x, T y, T z) {
  return atan2(2*(w*x + y*z), 1-2*(x*x + y*y));
}



template <typename T>
inline T getPitchFromQuat(T w, T x, T y, T z) {
  return asin(2*(w*y - x*z));
}



template <typename T>
inline T getYawFromQuat(T w, T x, T y, T z) {
  return atan2(2*(w*z + x*y), 1-2*(y*y + z*z));
}



inline Scalar getYawFromSO3(const ob::SO3StateSpace::StateType& s) {
  return getYawFromQuat(s.w, s.x, s.y, s.z);
}



inline void setSO3FromYaw(ob::SO3StateSpace::StateType& s, double yaw) {
  s.w = cos(yaw);
  s.x = 0;
  s.y = 0;
  s.z = sin(yaw);
}



inline void setSO3FromRPY(ob::SO3StateSpace::StateType& s, double* rpy) {
  const auto r2 = rpy[0]*0.5;
  const auto p2 = rpy[1]*0.5;
  const auto y2 = rpy[2]*0.5;
  const auto cr = cos(r2);
  const auto cp = cos(p2);
  const auto cy = cos(y2);
  const auto sr = sin(r2);
  const auto sp = sin(p2);
  const auto sy = sin(y2);
  s.w = cy * cp * cr + sy * sp * sr;
  s.x = cy * cp * sr - sy * sp * cr;
  s.y = sy * cp * sr + cy * sp * cr;
  s.z = sy * cp * cr - cy * sp * sr;
}



grid_map::Matrix inpaintMatrix(const grid_map::Matrix &mat);



grid_map::Matrix blurMatrix(const grid_map::Matrix &mat, int size);



grid_map::Matrix gaussianBlurMatrix(const grid_map::Matrix &mat, int size, double std_dev);



grid_map::Matrix dilateAndErodeMatrix(const grid_map::Matrix& mat, int size);



grid_map::Matrix erodeMatrix(const grid_map::Matrix& mat, int size);



grid_map::Matrix dilateMatrix(const grid_map::Matrix& mat, int size);



void estimateNormals(grid_map::GridMap& map,
                     double estimation_radius,
                     const std::string& input_layer,
                     const std::string& output_layer_prefix = "normal");

}
