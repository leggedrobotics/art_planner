#pragma once

#include <thread>

#include <nav_msgs/Path.h>
#include <ompl/base/PlannerData.h>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

#include <art_planner/params.h>



namespace art_planner {



class Visualizer {

  ros::NodeHandle nh_;
  ros::Publisher graph_pub_;
  ros::Publisher collision_pub_;
  ros::Publisher path_collision_pub_;

  std::thread collision_pub_thread_;

  ParamsConstPtr params_;

  int last_num_vertices{0};
  int last_num_new_vertices{0};
  int last_num_edges{0};
  int last_num_start_goal{0};
  int last_num_path_collisions{100};

  void addVertices(const ompl::base::PlannerData& dat,
                   visualization_msgs::MarkerArray& array);

  void addEdges(const ompl::base::PlannerData& dat,
                visualization_msgs::MarkerArray& array);

  void addStartGoal(const ompl::base::PlannerData& dat,
                    visualization_msgs::MarkerArray& array);

  void visualizeCollisionBoxesThread(const visualization_msgs::MarkerArray &array);

  void visualizeCollisionBoxes();

public:

  Visualizer(const ros::NodeHandle& nh, const ParamsConstPtr& params);

  ~Visualizer();

  void visualizePlannerGraph(const ompl::base::PlannerData& dat,
                             const std::string &frame_id);


  void visualizePathCollisions(const nav_msgs::Path& path);

};



}
