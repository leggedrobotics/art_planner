#include "art_planner_ros/planner_ros.h"

#include <ros/spinner.h>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/MarkerArray.h>

#include <art_planner_ros/utils.h>



using namespace art_planner;



void PlannerRos::mapCallback(const grid_map_msgs::GridMapConstPtr& map_msg) {
  std::lock_guard<std::mutex> lock(map_queue_mutex_);

  if (!map_queue_.map) map_queue_.map.reset(new grid_map::GridMap());
  grid_map::GridMapRosConverter::fromMessage(*map_msg, *map_queue_.map);
  map_queue_.map->convertToDefaultStartIndex();

  map_queue_.info = map_msg->info;
}



void PlannerRos::stopPlanningContinuously() {
  planning_continuously_ = false;
  std::lock_guard<std::mutex> lock(planning_thread_mutex_);
  if (continuous_planning_thread_.joinable()) {
    continuous_planning_thread_.join();
  }
}



void PlannerRos::planContinuouslyThread() {
  ros::Time last_plan_start;
  ros::Duration replan_time(1/params_->planner.replan_freq);

  while (planning_continuously_) {
    last_plan_start = ros::Time::now();

    planFromCurrentRobotPose();

    const auto cur_time = ros::Time::now();
    const auto sleep_time = replan_time - (cur_time - last_plan_start);
    if (sleep_time.toSec() > 0.0) {
      // Sleep until we want to replan.
      sleep_time.sleep();
    }
  }
}



void PlannerRos::planFromCurrentRobotPose() {
  // Get robot pose.
  geometry_msgs::TransformStamped pose_tf;
  try {
    pose_tf = tf_buffer_.lookupTransform(map_queue_.info.header.frame_id,
                                    params_->robot.base_frame,
                                    ros::Time::now(),
                                    ros::Duration(0.1));
  } catch (tf2::TransformException& ex) {
    ROS_WARN("%s",ex.what());
    ROS_ERROR("Could not get robot pose from TF. Cannot plan!");
    publishFeedback(Feedback::NO_ROBOT_TF);
    return;
  }

  // Convert goal pose to map frame (map might drift w.r.t goal).
  geometry_msgs::PoseStamped pose_goal_transformed;
  try {
    std::lock_guard<std::mutex> lock(pose_goal_mutex_);
    // Set goal stamp to map time because goal might have very old stamp.
    pose_goal_.header.stamp = map_queue_.info.header.stamp;
    tf_buffer_.transform(pose_goal_,
                         pose_goal_transformed,
                         map_queue_.info.header.frame_id,
                         ros::Duration(0.1));
  } catch (tf2::TransformException& ex) {
    ROS_WARN("%s",ex.what());
    ROS_ERROR("Could not transform goal pose to map frame. Not planning.");
    publishFeedback(Feedback::NO_GOAL_TF);
    return;
  }

  tf2::Stamped<tf2::Transform> tf_tf2;
  tf2::fromMsg(pose_tf, tf_tf2);

  // Get feet center pose from base.
  tf2::Transform offset;
  offset.setIdentity();
  offset.setOrigin(tf2::Vector3(0, 0, params_->robot.feet.offset.z));

  tf_tf2 *= offset;

  geometry_msgs::PoseStamped pose_robot;
  tf2::toMsg(tf_tf2, pose_robot);

  ob::ScopedState<> start = converter_.poseRosToOmpl(pose_robot);
  ob::ScopedState<> goal = converter_.poseRosToOmpl(pose_goal_transformed);

  publishFeedback(Feedback::PLANNING);
  const auto result = updateMapAndPlan(start, goal);
  visualizePlannerGraph();

  switch (result) {
    case PlannerStatus::INVALID_GOAL: publishFeedback(Feedback::INVALID_GOAL); break;
    case PlannerStatus::INVALID_START: publishFeedback(Feedback::INVALID_START); break;
    case PlannerStatus::NO_MAP: publishFeedback(Feedback::NO_MAP); break;
    case PlannerStatus::NOT_SOLVED: publishFeedback(Feedback::NO_SOLUTION); break;
    case PlannerStatus::SOLVED: publishFeedback(Feedback::FOUND_SOLUTION); break;
    case PlannerStatus::UNKNOWN: ROS_ERROR_STREAM("Unknown planner feedback. Something is wrong!");
  }

  // Publish path if successful and we did not reach the goal.
  if (result == PlannerStatus::SOLVED && planning_continuously_) {
    auto plan_ros = converter_.pathOmplToRos(getSolutionPath(params_->planner.simplify_solution));
    plan_ros.header.frame_id = planning_map_info_.header.frame_id;
    plan_ros.header.stamp = ros::Time::now();
    publishPath(plan_ros, true);
  }
}



void PlannerRos::publishFeedback(FeedbackStatus feedback) const {
  Feedback feeback_msg;
  feeback_msg.status = feedback;
  plan_act_srv_->publishFeedback(feeback_msg);
}



void PlannerRos::cancelGoalCallback() {
  ROS_INFO_STREAM("Stop continuous planning requested.");
  stopPlanningContinuously();
  plan_act_srv_->setPreempted();
}



void PlannerRos::goalCallback() {
  goalPoseCallback(plan_act_srv_->acceptNewGoal());
}



void PlannerRos::goalPoseCallback(const art_planner_msgs::PlanToGoalGoalConstPtr& goal_msg) {
  std::lock_guard<std::mutex> lock(pose_goal_mutex_);

  pose_goal_ = goal_msg->goal;
  ROS_INFO_STREAM("Received goal pose:\n" << pose_goal_);
  publishFeedback(Feedback::GOAL_RECEIVED);
  if (!planning_continuously_) {
    std::lock_guard<std::mutex> lock(planning_thread_mutex_);
    planning_continuously_ = true;
    continuous_planning_thread_ = std::thread(&PlannerRos::planContinuouslyThread, this);
  }

}



bool PlannerRos::transformRosPoseToMapFrame(const geometry_msgs::PoseStamped& in,
                                            geometry_msgs::PoseStamped& out) const {
  geometry_msgs::TransformStamped tf;
  try {
    tf = tf_buffer_.lookupTransform(map_queue_.info.header.frame_id,
                                    tf2::getFrameId(in),
                                    tf2::getTimestamp(in),
                                    ros::Duration(0.01));
  } catch (tf2::TransformException& ex) {
    ROS_WARN("%s",ex.what());
    return false;
  }
  tf2::doTransform(in, out, tf);
  return true;
}



bool PlannerRos::getPlanService(nav_msgs::GetPlanRequest& req,
                                nav_msgs::GetPlanResponse& res) {
  if (!transformRosPoseToMapFrame(req.start, req.start)) {
    ROS_WARN("Could not transform start pose to map frame. Assuming it's already map.");
  }
  if (!transformRosPoseToMapFrame(req.goal, req.goal)) {
    ROS_WARN("Could not transform goal pose to map frame. Assuming it's already map.");
  }

  ob::ScopedState<> start = converter_.poseRosToOmpl(req.start);
  ob::ScopedState<> goal = converter_.poseRosToOmpl(req.goal);

  const auto result = updateMapAndPlan(start, goal);

  if (result == PlannerStatus::SOLVED) {
    res.plan = converter_.pathOmplToRos(getSolutionPath(params_->planner.simplify_solution));
    res.plan.header.frame_id = planning_map_info_.header.frame_id;
    res.plan.header.stamp = ros::Time::now();
    publishPath(res.plan, false);
    return true;
  } else {
    return false;
  }
}



void PlannerRos::publishPath(nav_msgs::Path path,
                             const bool& connect_callback) {
  // Publish regular ROS message.
  path_pub_.publish(path);
  visualizer_.visualizePathCollisions(path);
}

PlannerRos::~PlannerRos() {
  // Wait for planning thread to finish.
  if (planning_continuously_) {
    std::cout << "Stopping continuous planning before shutdown." << std::endl;
    stopPlanningContinuously();
    publishFeedback(Feedback::NODE_SHUTDOWN);
    plan_act_srv_->setAborted();
  }
}



void PlannerRos::visualizePlannerGraph() {
  ob::PlannerData dat(ss_->getSpaceInformation());
  ss_->getPlannerData(dat);

  visualizer_.visualizePlannerGraph(dat, planning_map_info_.header.frame_id);
}



PlannerRos::PlannerRos(const ros::NodeHandle& nh)
    : Planner(loadRosParameters(nh)),
      nh_(nh),
      visualizer_(nh, params_),
      converter_(space_) {

  map_sub_ = nh_.subscribe("elevation_map", 1, &PlannerRos::mapCallback, this);
  plan_act_srv_ = std::make_unique<PlanningActionServer>(nh_,
                                                         "plan_to_goal",
                                                         false);
  plan_act_srv_->registerGoalCallback(std::bind(&PlannerRos::goalCallback,
                                                this));
  plan_act_srv_->registerPreemptCallback(std::bind(&PlannerRos::cancelGoalCallback,
                                                this));
  plan_act_srv_->start();
  plan_srv_ = nh_.advertiseService("plan", &PlannerRos::getPlanService, this);
  path_pub_ = nh_.advertise<nav_msgs::Path>("path", 1, true);
  map_pub_ = nh_.advertise<grid_map_msgs::GridMap>("map", 1, true);
}



void PlannerRos::updateMap() {
  std::lock_guard<std::mutex> lock(map_queue_mutex_);

  if (!map_queue_.map) {
    ROS_WARN_STREAM("No new map received since last planning call.");
  } else {
    planning_map_info_ = map_queue_.info;
    setMap(std::move(map_queue_.map));
  }
}



void PlannerRos::publishMap() const {
  if (map_pub_.getNumSubscribers() > 0) {
    grid_map_msgs::GridMap out_msg;
    grid_map::GridMapRosConverter::toMessage(map_->getMap(), out_msg);
    out_msg.info = planning_map_info_;
    map_pub_.publish(out_msg);
  }
}



PlannerStatus PlannerRos::updateMapAndPlan(const ob::ScopedState<>& start,
                                           const ob::ScopedState<>& goal) {
  updateMap();

  const auto result = plan(start, goal);

  publishMap();

  return result;
}
