#include "art_planner_ros/visualizer.h"

#include <art_planner/planner.h>



using namespace art_planner;



Visualizer::Visualizer(const ros::NodeHandle& nh, const ParamsConstPtr &params) : nh_(nh), params_(params) {
  collision_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("collision_boxes", 1);
  path_collision_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("path_collisions", 1, true);

  visualizeCollisionBoxes();
}



Visualizer::~Visualizer() {
  if (collision_pub_thread_.joinable()) {
    collision_pub_thread_.join();
  }
}



ros::Publisher& Visualizer::getGraphPublisher(const std::string& ns_prefix) {
  if (graph_pub_.find(ns_prefix) == graph_pub_.end()) {
    graph_pub_[ns_prefix] = nh_.advertise<visualization_msgs::MarkerArray>(ns_prefix + "graph", 1, true);
    // Remove old markers.
    visualization_msgs::Marker marker;
    visualization_msgs::MarkerArray array;
    marker.action = visualization_msgs::Marker::DELETEALL;
    marker.ns = "vertices";
    array.markers.push_back(marker);
    marker.ns = "edges";
    array.markers.push_back(marker);
    marker.ns = "start_goal";
    array.markers.push_back(marker);
    graph_pub_[ns_prefix].publish(array);
  }
  return graph_pub_[ns_prefix];
}



inline void setMarkerPoseFromState(const ob::State* state, visualization_msgs::Marker& marker) {
  marker.pose.position.x = state->as<Planner::StateType>()->getX();
  marker.pose.position.y = state->as<Planner::StateType>()->getY();
  marker.pose.position.z = state->as<Planner::StateType>()->getZ();
  marker.pose.orientation.w = state->as<Planner::StateType>()->rotation().w;
  marker.pose.orientation.x = state->as<Planner::StateType>()->rotation().x;
  marker.pose.orientation.y = state->as<Planner::StateType>()->rotation().y;
  marker.pose.orientation.z = state->as<Planner::StateType>()->rotation().z;
}



inline void setMarkerPointFromState(const ob::State* state, geometry_msgs::Point& point) {
  point.x = state->as<Planner::StateType>()->getX();
  point.y = state->as<Planner::StateType>()->getY();
  point.z = state->as<Planner::StateType>()->getZ();
}



void Visualizer::addVertices(const ompl::base::PlannerData& dat,
                             visualization_msgs::MarkerArray& array) {
  visualization_msgs::Marker marker;

  // Do vertices.
  marker.ns = "vertices";
  marker.action = visualization_msgs::Marker::MODIFY;
  marker.id = 0;
  marker.type = visualization_msgs::Marker::ARROW;
  marker.scale.x = 0.1;
  marker.scale.y = 0.05;
  marker.scale.z = 0.01;
  marker.color.r = 0.8f;
  const auto n_vertices = dat.numVertices();
  for (size_t i = 0; i < n_vertices; ++i) {
    if (i == last_num_vertices) {
//      marker.color.r = 0.0f;
//      marker.color.b = 0.8f;
//      marker.scale.x = 0.2;
//      marker.scale.y = 0.1;
    }

    const auto cur_vert = dat.getVertex(i);
    setMarkerPoseFromState(cur_vert.getState(), marker);
    array.markers.push_back(marker);
    ++marker.id;
  }

  const int n_markers = marker.id;
  marker.action = visualization_msgs::Marker::DELETE;
  while (marker.id++ < last_num_vertices) {
    array.markers.push_back(marker);
  }
  last_num_vertices = n_markers;
}



void Visualizer::addEdges(const ompl::base::PlannerData& dat,
                          visualization_msgs::MarkerArray& array) {
  visualization_msgs::Marker marker;

  // Do edges.
  marker.ns = "edges";
  marker.action = visualization_msgs::Marker::MODIFY;
  marker.id = 0;
  marker.type = visualization_msgs::Marker::LINE_LIST;
  marker.scale.x = 0.01;
  marker.scale.y = 0;
  marker.scale.z = 0;
  marker.color.r = 0;
  marker.color.g = 0;
  marker.color.b = 0.8f;
  marker.points.resize(2*dat.numEdges());
  marker.pose.position.x = 0;
  marker.pose.position.y = 0;
  marker.pose.position.z = 0;
  marker.pose.orientation.w = 1;
  marker.pose.orientation.x = 0;
  marker.pose.orientation.y = 0;
  marker.pose.orientation.z = 0;
  std::vector<unsigned int> edges;
  const auto n_vertices = dat.numVertices();
  size_t k = 0;
  for (size_t i = 0; i < n_vertices; ++i) {
    const auto cur_vert = dat.getVertex(i);
    dat.getIncomingEdges(i, edges);
    const auto cur_state= cur_vert.getState();
    for (const auto& j: edges) {
      setMarkerPointFromState(cur_state, marker.points[k++]);
      const auto other_vert = dat.getVertex(j);
      setMarkerPointFromState(other_vert.getState(), marker.points[k++]);
    }
  }
  array.markers.push_back(marker);

}



void Visualizer::addStartGoal(const ompl::base::PlannerData& dat,
                              visualization_msgs::MarkerArray &array) {
  visualization_msgs::Marker marker;

  marker.ns = "start_goal";
  marker.action = visualization_msgs::Marker::MODIFY;
  marker.id = 0;
  marker.scale.x = 0.4;
  marker.scale.y = 0.2;
  marker.scale.z = 0.1;
  marker.color.r = 0.0f;
  marker.color.g = 0.8f;
  for (unsigned int i = 0; i < dat.numGoalVertices(); ++i) {
    const auto goal_vert = dat.getGoalVertex(i);
    setMarkerPoseFromState(goal_vert.getState(), marker);
    array.markers.push_back(marker);
    ++marker.id;
  }
  for (unsigned int i = 0; i < dat.numStartVertices(); ++i) {
    const auto goal_vert = dat.getStartVertex(i);
    setMarkerPoseFromState(goal_vert.getState(), marker);
    array.markers.push_back(marker);
    ++marker.id;
  }

  const int n_markers = marker.id;
  marker.action = visualization_msgs::Marker::DELETE;
  while (marker.id++ < last_num_start_goal) {
    array.markers.push_back(marker);
  }
  last_num_start_goal = n_markers;
}



void Visualizer::visualizePlannerGraph(const ompl::base::PlannerData& dat,
                                       const std::string& frame_id,
                                       const std::string& ns_prefix) {
  // Do nothing if no one's watching.
  auto& graph_pub = getGraphPublisher(ns_prefix);
  if (graph_pub.getNumSubscribers() == 0) return;

  visualization_msgs::MarkerArray array;

  addVertices(dat, array);
  addEdges(dat, array);
  addStartGoal(dat, array);

  // Set values for all markers.
  const auto time = ros::Time::now();

  for (auto& marker: array.markers) {
    marker.header.stamp = time;
    marker.header.frame_id = frame_id;
    marker.color.a = 1.0;
  }

  graph_pub.publish(array);
}



void Visualizer::visualizeCollisionBoxesThread(const visualization_msgs::MarkerArray& array) {
  ros::Rate rate(10);
  while (ros::ok()) {
    if (collision_pub_.getNumSubscribers()) {
      collision_pub_.publish(array);
    }
    rate.sleep();
  }
}



void Visualizer::visualizeCollisionBoxes() {
  visualization_msgs::MarkerArray array;
  visualization_msgs::Marker marker;

  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::MODIFY;
  marker.header.frame_id = params_->robot.base_frame;
  marker.color.a = 0.5;
  marker.pose.orientation.w = 1;
  marker.pose.orientation.x = 0;
  marker.pose.orientation.y = 0;
  marker.pose.orientation.z = 0;

  // Torso.

  marker.ns = "torso";
  marker.id = 0;
  marker.scale.x = params_->robot.torso.length;
  marker.scale.y = params_->robot.torso.width;
  marker.scale.z = params_->robot.torso.height;
  marker.color.r = 0;
  marker.color.g = 0;
  marker.color.b = 0.8f;
  marker.pose.position.x = params_->robot.torso.offset.x;
  marker.pose.position.y = params_->robot.torso.offset.y;
  marker.pose.position.z = params_->robot.torso.offset.z;

  array.markers.push_back(marker);

  // Feet.
  marker.ns = "reachability";
  marker.scale.x = params_->robot.feet.reach.x;
  marker.scale.y = params_->robot.feet.reach.y;
  marker.scale.z = params_->robot.feet.reach.z;
  marker.color.r = 0.8f;
  marker.color.b = 0;
  marker.pose.position.z = params_->robot.feet.offset.z;

  // Add foot markers to separate vector first so we can iterate over them.
  std::vector<visualization_msgs::Marker> feet_markers;
  marker.pose.position.x = params_->robot.feet.offset.x;
  marker.pose.position.y = params_->robot.feet.offset.y;
  feet_markers.push_back(marker);
  ++marker.id;

  for (const auto& plane: params_->robot.feet.plane_symmetries) {
    size_t n_boxes = feet_markers.size();
    if (plane == "sagittal") {
      // Need to iterate like this because we alter vector size during loop.
      for (size_t i = 0; i < n_boxes; ++i) {
        feet_markers.push_back(feet_markers[i]);
        feet_markers.back().pose.position.y *= -1;
        feet_markers.back().id = marker.id++;
      }
    } else if (plane == "coronal") {
      // Need to iterate like this because we alter vector size during loop.
      for (size_t i = 0; i < n_boxes; ++i) {
        feet_markers.push_back(feet_markers[i]);
        feet_markers.back().pose.position.x *= -1;
        feet_markers.back().id = marker.id++;
      }
    }
  }

  // Add foot markers to output array.
  array.markers.insert(array.markers.end(), feet_markers.begin(), feet_markers.end());

  collision_pub_thread_ = std::thread(std::bind(&Visualizer::visualizeCollisionBoxesThread,
                                                this,
                                                array));
}



geometry_msgs::Pose applyOffsetTo(const geometry_msgs::Pose& offset, const geometry_msgs::Pose& pose) {
  geometry_msgs::Pose pose_out;
  pose_out.orientation = pose.orientation;

  const Eigen::Quaterniond quat(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
  const Eigen::Vector3d offset_o = quat * Eigen::Vector3d(offset.position.x, offset.position.y, offset.position.z);

  pose_out.position.x = pose.position.x + offset_o.x();
  pose_out.position.y = pose.position.y + offset_o.y();
  pose_out.position.z = pose.position.z + offset_o.z();

  return pose_out;
}



void Visualizer::visualizePathCollisions(const nav_msgs::Path& path) {
  if (path_collision_pub_.getNumSubscribers()) {
    visualization_msgs::MarkerArray array;
    visualization_msgs::Marker marker;

    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::MODIFY;
    marker.header.frame_id = path.header.frame_id;
    marker.header.stamp = path.header.stamp;
    marker.color.a = 0.5;

    marker.scale.x = params_->robot.torso.length;
    marker.scale.y = params_->robot.torso.width;
    marker.scale.z = params_->robot.torso.height;
    marker.color.r = 0;
    marker.color.g = 0;
    marker.color.b = 0.8f;

    int i = 0;
    marker.id = 0;
    for (const auto& pose: path.poses) {
      marker.ns = std::to_string(i++);
      for (unsigned int i = 0; i < 4; ++i) {
        marker.pose.position.x = params_->robot.torso.offset.x;
        marker.pose.position.y = params_->robot.torso.offset.y;
        marker.pose.position.z = params_->robot.torso.offset.z - params_->robot.feet.offset.z;

        marker.pose = applyOffsetTo(marker.pose, pose.pose);

        array.markers.push_back(marker);
      }
    }

    marker.scale.x = params_->robot.feet.reach.x;
    marker.scale.y = params_->robot.feet.reach.y;
    marker.scale.z = params_->robot.feet.reach.z;
    marker.color.r = 0.8f;
    marker.color.g = 0;
    marker.color.b = 0;

    i = 0;
    for (const auto& pose: path.poses) {
      marker.ns = std::to_string(i++);
      marker.id = 1;
      for (unsigned int i = 0; i < 4; ++i) {
        marker.pose.position.x = i < 2 ? params_->robot.feet.offset.x : -params_->robot.feet.offset.x;
        marker.pose.position.y = i % 2 ? params_->robot.feet.offset.y : -params_->robot.feet.offset.y;
        marker.pose.position.z = 0;

        marker.pose = applyOffsetTo(marker.pose, pose.pose);

        array.markers.push_back(marker);
        ++marker.id;
      }
    }

    const auto n_collisions = i;

    marker.action = visualization_msgs::Marker::DELETE;
    for (; i < last_num_path_collisions;) {
      marker.ns = std::to_string(i++);
      for (int j = 0; j < 5; ++j) {
        marker.id = j;
        array.markers.push_back(marker);
      }
    }

    last_num_path_collisions = n_collisions;

    path_collision_pub_.publish(array);
  }
}
