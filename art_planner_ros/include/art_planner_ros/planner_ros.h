#pragma once

#include <thread>

#include <actionlib/server/simple_action_server.h>
#include <art_planner/planner.h>
#include <art_planner_msgs/PlanToGoalAction.h>
#include <geometry_msgs/PoseStamped.h>
#include <grid_map_msgs/GridMap.h>
#include <nav_msgs/GetPlan.h>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>

#include <art_planner_ros/converter.h>
#include <art_planner_ros/visualizer.h>
#include <art_planner/params.h>



namespace art_planner {



class PlannerRos : protected Planner {

 protected:

  // ROS members.
  using PlanningActionServer = actionlib::SimpleActionServer<art_planner_msgs::PlanToGoalAction>;
  using Feedback = art_planner_msgs::PlanToGoalFeedback;
  using FeedbackStatus = Feedback::_status_type;

  ros::NodeHandle nh_;

  ros::Subscriber map_sub_;
  ros::ServiceServer plan_srv_;
  ros::ServiceClient cost_srv_client_;
  ros::ServiceClient cost_no_update_srv_client_;
  std::unique_ptr<PlanningActionServer> plan_act_srv_;
  ros::Publisher path_pub_;
  ros::Publisher map_pub_;
  ros::Publisher timer_pub_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_{tf_buffer_};

  Visualizer visualizer_;

  // Planning members.
  geometry_msgs::PoseStamped pose_goal_;
  mutable std::mutex pose_goal_mutex_;
  std::atomic<bool> planning_continuously_{false};
  std::thread continuous_planning_thread_;
  std::mutex planning_thread_mutex_;

  grid_map_msgs::GridMapInfo planning_map_info_;

  struct {
    std::unique_ptr<grid_map::GridMap> map;
    grid_map_msgs::GridMapInfo info;
  } map_queue_;
  mutable std::mutex map_queue_mutex_;

  Converter converter_;

  // Member functions.

  void stopPlanningContinuously();

  virtual void planContinuouslyThread();

  virtual void planFromCurrentRobotPose();

  nav_msgs::Path getAndPublishPathFromTo(const geometry_msgs::PoseStamped& pose_start,
                                         const geometry_msgs::PoseStamped& pose_goal);

  bool getCurrentRobotPose(geometry_msgs::PoseStamped *pose) const;

  bool transformRosPoseToMapFrame(const geometry_msgs::PoseStamped& in,
                                  geometry_msgs::PoseStamped& out) const;

  void publishFeedback(FeedbackStatus feedback) const;

  void mapCallback(const grid_map_msgs::GridMapConstPtr& map_msg);

  void cancelGoalCallback();

  void goalCallback();

  void goalPoseCallback(const art_planner_msgs::PlanToGoalGoalConstPtr& goal);

  bool getPlanService(nav_msgs::GetPlanRequest& req,
                      nav_msgs::GetPlanResponse& res);

  bool planPath(const ob::ScopedState<>& start,
                const ob::ScopedState<>& goal,
                nav_msgs::Path &path_out);

  virtual void publishPath(nav_msgs::Path path);

  void visualizePlannerGraph(const std::string& ns_prefix="", bool get_invalid=false);

  void updateMap();

  void publishMap() const;

  void publishTiming(const double& timing) const;

  PlannerStatus updateMapAndPlan(const ob::ScopedState<>& start,
                                 const ob::ScopedState<>& goal);

  PlannerStatus updateMapAndPlanFromCurrentRobotPose(const ob::ScopedState<>& goal);

public:

  ~PlannerRos();

  explicit PlannerRos(const ros::NodeHandle& nh);

};



}
