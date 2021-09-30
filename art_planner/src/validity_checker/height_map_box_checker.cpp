#include "art_planner/validity_checker/height_map_box_checker.h"

#include "art_planner/utils.h"



using namespace art_planner;



HeightMapBoxChecker::HeightMapBoxChecker(float length_x,
                                         float length_y,
                                         float length_z) {
  dInitODE();
  world_ = dWorldCreate();
  space_ = dHashSpaceCreate(0);
  body_box_ = dBodyCreate(world_);
  body_field_ = dBodyCreate(world_);
  geom_box_ = dCreateBox(space_, length_x, length_y, length_z);
  field_.data = dGeomHeightfieldDataCreate();
  geom_field_ = dCreateHeightfield(space_, field_.data, 1);
  dRFrom2Axes(field_.rot.data(), -1, 0, 0, 0, 0, 1);
  dGeomSetBody(geom_box_, body_box_);
  dGeomSetBody(geom_field_, body_field_);
  dBodySetRotation(body_field_, field_.rot.data());
}



HeightMapBoxChecker::~HeightMapBoxChecker() {
  dSpaceDestroy (space_);
  dWorldDestroy (world_);
  dCloseODE();
}



void HeightMapBoxChecker::setHeightField(std::shared_ptr<Map> map,
                                         const std::string& layer_name) {
  std::lock_guard<std::mutex> lock(mutex_);

  const auto& field = map->getMap();
  // Get height field from height map.
  field_.mat = field.get(layer_name).rowwise().reverse();
  const auto field_length = field.getLength();
  const auto field_size = field.getSize();
  const auto min = field_.mat.minCoeffOfFinites();
  const auto max = field_.mat.maxCoeffOfFinites();
  dGeomHeightfieldDataBuildSingle(field_.data, field_.mat.data(), 0, field_length.x(), field_length.y(), field_size.x(), field_size.y(), 1, 0, 0, 0);
  dGeomHeightfieldDataSetBounds(field_.data, min, max);
  dGeomHeightfieldSetHeightfieldData(geom_field_, field_.data);
  const auto pos = field.getPosition();
  dBodySetPosition(body_field_, pos.x(), pos.y(), 0);
}



int HeightMapBoxChecker::checkCollision(const std::vector<dPose>& box_poses) const {
  int n_manifold_with_contact = 0;

  std::lock_guard<std::mutex> lock(mutex_);

  for (const auto& pose: box_poses) {
    // Set box pose.
    dBodySetPosition(body_box_, pose.origin[0], pose.origin[1], pose.origin[2]);
    dBodySetRotation(body_box_, pose.rotation.data());
    const int n_col = dCollide(geom_box_, geom_field_, 1, contact_geom_.get(), sizeof(dContactGeom));
    if (n_col) ++n_manifold_with_contact;
  }

  return n_manifold_with_contact;
}

