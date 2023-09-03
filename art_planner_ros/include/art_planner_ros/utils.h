#pragma once

#include <string>
#include <vector>

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

// Specialize for unsigned int.
template <>
inline unsigned int getParamWithDefaultWarning<unsigned int>(
                      const ros::NodeHandle& nh,
                      const std::string& name,
                      const unsigned int& default_val) {
  return getParamWithDefaultWarning(nh, name, static_cast<int>(default_val));
}

// Overload for std::vector.
template <typename T>
inline std::vector<T> getParamWithDefaultWarning(
                        const ros::NodeHandle& nh,
                        const std::string& name,
                        const std::vector<T>& default_val) {
  std::vector<T> param;

  if (!nh.param(name, param, default_val)) {
    std::string default_string;
    for (size_t i = 0; i < default_val.size(); ++i) {
      if (i > 0) default_string += std::string(", ");
      default_string += std::to_string(default_val[i]);
    }
    ROS_WARN_STREAM("Could not find ROS param \"" << name <<
                    "\", set to default: [" << default_string << "]");
  }

  return param;
}

// Specialize for std::vector of std::string.
template <>
inline std::vector<std::string> getParamWithDefaultWarning(
                        const ros::NodeHandle& nh,
                        const std::string& name,
                        const std::vector<std::string>& default_val) {
  std::vector<std::string> param;

  if (!nh.param(name, param, default_val)) {
    std::string default_string;
    for (size_t i = 0; i < default_val.size(); ++i) {
      if (i > 0) default_string += std::string(", ");
      default_string += default_val[i];
    }
    ROS_WARN_STREAM("Could not find ROS param \"" << name <<
                    "\", set to default: [" << default_string << "]");
  }

  return param;
}



ParamsPtr loadRosParameters(const ros::NodeHandle& nh);



}
