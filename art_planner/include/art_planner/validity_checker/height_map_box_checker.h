#pragma once

#include <memory>
#include <mutex>
#include <vector>

#include <ode/ode.h>
#include <grid_map_core/GridMap.hpp>

#include "art_planner/map/map.h"



namespace art_planner {



class HeightMapBoxChecker {
public:
  struct dPose {
    std::array<dReal, 4> origin{0, 0, 0, 0};
    std::array<dReal, 12> rotation{1, 0, 0, 0,
                                   0, 1, 0, 0,
                                   0, 0, 1, 0};
  };

private:

  struct {
    Eigen::MatrixXf mat;
    dHeightfieldDataID data;
    std::array<dReal, 12> rot;
  } field_;

  // Collision world members.
  dWorldID world_;
  dSpaceID space_;
  dBodyID body_field_;
  dGeomID geom_field_;
  dBodyID body_box_;
  dGeomID geom_box_;
  std::unique_ptr<dContactGeom> contact_geom_{new dContactGeom};

  mutable std::mutex mutex_;

  void addCollisionObjectsToWorld();

public:

  HeightMapBoxChecker(float length_x,
                      float length_y,
                      float length_z);

  ~HeightMapBoxChecker();

  void setHeightField(std::shared_ptr<Map> field,
                      const std::string& layer_name);

  int checkCollision(const std::vector<dPose>& box_poses) const;

};



}
