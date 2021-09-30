#pragma once

#include <string>

#include <art_planner/params.h>
#include <ros/node_handle.h>



namespace art_planner {



template <typename T>
inline T getParamWithDefaultWarning(const ros::NodeHandle& nh,
                             const std::string& name,
                             const T& default_val) {
  T param;

  if (!nh.param(name, param, default_val)) {
    ROS_WARN_STREAM("Could not find ROS param \"" << name <<
                    "\", set to default: " << default_val);
  }

  return param;
}

template <>
inline unsigned int getParamWithDefaultWarning<unsigned int>(const ros::NodeHandle& nh,
                                                      const std::string& name,
                                                      const unsigned int& default_val) {
  return getParamWithDefaultWarning(nh, name, static_cast<int>(default_val));
}



ParamsPtr loadRosParameters(const ros::NodeHandle& nh);



}
