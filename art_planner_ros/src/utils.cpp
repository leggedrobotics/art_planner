#include "art_planner_ros/utils.h"



using namespace art_planner;



ParamsPtr art_planner::loadRosParameters(const ros::NodeHandle& nh) {
  ParamsPtr params = std::make_shared<Params>();

  // Planner.

  params->planner.name =
      getParamWithDefaultWarning(nh,
                                 "planner/name",
                                 params->planner.name);
  params->planner.elevation_layer =
      getParamWithDefaultWarning(nh,
                                 "planner/elevation_layer",
                                 params->planner.elevation_layer);
  params->planner.traversability_layer =
      getParamWithDefaultWarning(nh,
                                 "planner/traversability_layer",
                                 params->planner.traversability_layer);
  params->planner.plan_time =
      getParamWithDefaultWarning(nh,
                                 "planner/plan_time",
                                 params->planner.plan_time);
  params->planner.n_threads =
      getParamWithDefaultWarning(nh,
                                 "planner/n_threads",
                                 params->planner.n_threads);
  params->planner.replan_freq =
      getParamWithDefaultWarning(nh,
                                 "planner/replan_freq",
                                 params->planner.replan_freq);
  params->planner.traversability_thres =
      getParamWithDefaultWarning(nh,
                                 "planner/traversability_thres",
                                 params->planner.traversability_thres);
  params->planner.simplify_solution =
      getParamWithDefaultWarning(nh,
                                 "planner/simplify_solution",
                                 params->planner.simplify_solution);
  params->planner.snap_goal_to_map =
      getParamWithDefaultWarning(nh,
                                 "planner/snap_goal_to_map",
                                 params->planner.snap_goal_to_map);
  params->planner.unknown_space_untraversable =
      getParamWithDefaultWarning(nh,
                                 "planner/unknown_space_untraversable",
                                 params->planner.unknown_space_untraversable);

  // Planner / Safety

  params->planner.safety.foothold_margin =
      getParamWithDefaultWarning(nh,
                                 "planner/safety/foothold_margin",
                                 params->planner.safety.foothold_margin);
  params->planner.safety.foothold_margin_max_hole_size =
      getParamWithDefaultWarning(nh,
                                 "planner/safety/foothold_margin_max_hole_size",
                                 params->planner.safety.foothold_margin_max_hole_size);
  params->planner.safety.foothold_margin_max_drop =
      getParamWithDefaultWarning(nh,
                                 "planner/safety/foothold_margin_max_drop",
                                 params->planner.safety.foothold_margin_max_drop);
  params->planner.safety.foothold_margin_max_drop_search_radius =
      getParamWithDefaultWarning(nh,
                                 "planner/safety/foothold_margin_max_drop_search_radius",
                                 params->planner.safety.foothold_margin_max_drop_search_radius);
  params->planner.safety.foothold_margin_min_step =
      getParamWithDefaultWarning(nh,
                                 "planner/safety/foothold_margin_min_step",
                                 params->planner.safety.foothold_margin_min_step);
  params->planner.safety.foothold_size =
      getParamWithDefaultWarning(nh,
                                 "planner/safety/foothold_size",
                                 params->planner.safety.foothold_size);

  // Planner / Start Goal Search.

  params->planner.start_goal_search.start_radius =
      getParamWithDefaultWarning(nh,
                                 "planner/start_goal_search/start_radius",
                                 params->planner.start_goal_search.start_radius);
  params->planner.start_goal_search.goal_radius =
      getParamWithDefaultWarning(nh,
                                 "planner/start_goal_search/goal_radius",
                                 params->planner.start_goal_search.goal_radius);
  params->planner.start_goal_search.n_iter =
      getParamWithDefaultWarning(nh,
                                 "planner/start_goal_search/n_iter",
                                 params->planner.start_goal_search.n_iter);

  // Planner / Lazy PRM Star Min Update.

  params->planner.lazy_prm_star_min_update.invalidate_updated_graph_components =
      getParamWithDefaultWarning(nh,
                                 "planner/lazy_prm_star_min_update/invalidate_updated_graph_components",
                                 params->planner.lazy_prm_star_min_update.invalidate_updated_graph_components);
  params->planner.lazy_prm_star_min_update.height_change_for_update =
      getParamWithDefaultWarning(nh,
                                 "planner/lazy_prm_star_min_update/height_change_for_update",
                                 params->planner.lazy_prm_star_min_update.height_change_for_update);
  params->planner.lazy_prm_star_min_update.cleanup_when_not_planning =
      getParamWithDefaultWarning(nh,
                                 "planner/lazy_prm_star_min_update/cleanup_when_not_planning",
                                 params->planner.lazy_prm_star_min_update.cleanup_when_not_planning);

  // Planner / PRM Motion Cost.

  params->planner.prm_motion_cost.max_sample_time =
      getParamWithDefaultWarning(nh,
                                 "planner/prm_motion_cost/max_sample_time",
                                 params->planner.prm_motion_cost.max_sample_time);

  params->planner.prm_motion_cost.max_n_vertices =
      getParamWithDefaultWarning(nh,
                                 "planner/prm_motion_cost/max_n_vertices",
                                 params->planner.prm_motion_cost.max_n_vertices);

  params->planner.prm_motion_cost.max_n_edges =
      getParamWithDefaultWarning(nh,
                                 "planner/prm_motion_cost/max_n_edges",
                                 params->planner.prm_motion_cost.max_n_edges);

  params->planner.prm_motion_cost.recompute_density_after_n_samples =
      getParamWithDefaultWarning(nh,
                                 "planner/prm_motion_cost/recompute_density_after_n_samples",
                                 params->planner.prm_motion_cost.recompute_density_after_n_samples);

  params->planner.prm_motion_cost.max_query_edge_length =
      getParamWithDefaultWarning(nh,
                                 "planner/prm_motion_cost/max_query_edge_length",
                                 params->planner.prm_motion_cost.max_query_edge_length);

  params->planner.prm_motion_cost.risk_threshold =
      getParamWithDefaultWarning(nh,
                                 "planner/prm_motion_cost/risk_threshold",
                                 params->planner.prm_motion_cost.risk_threshold);


  // Planner / PRM Motion Cost / Cost Weights.

  params->planner.prm_motion_cost.cost_weights.energy =
      getParamWithDefaultWarning(nh,
                                 "planner/prm_motion_cost/cost_weights/energy",
                                 params->planner.prm_motion_cost.cost_weights.energy);
  params->planner.prm_motion_cost.cost_weights.time =
      getParamWithDefaultWarning(nh,
                                 "planner/prm_motion_cost/cost_weights/time",
                                 params->planner.prm_motion_cost.cost_weights.time);
  params->planner.prm_motion_cost.cost_weights.risk =
      getParamWithDefaultWarning(nh,
                                 "planner/prm_motion_cost/cost_weights/risk",
                                 params->planner.prm_motion_cost.cost_weights.risk);

  // Objectives / Custom Path Length.

  params->objectives.custom_path_length.use_directional_cost =
      getParamWithDefaultWarning(nh,
                                 "objectives/custom_path_length/use_directional_cost",
                                 params->objectives.custom_path_length.use_directional_cost);
  params->objectives.custom_path_length.max_lon_vel =
      getParamWithDefaultWarning(nh,
                                 "objectives/custom_path_length/max_lon_vel",
                                 params->objectives.custom_path_length.max_lon_vel);
  params->objectives.custom_path_length.max_lat_vel =
      getParamWithDefaultWarning(nh,
                                 "objectives/custom_path_length/max_lat_vel",
                                 params->objectives.custom_path_length.max_lat_vel);
  params->objectives.custom_path_length.max_ang_vel =
      getParamWithDefaultWarning(nh,
                                 "objectives/custom_path_length/max_ang_vel",
                                 params->objectives.custom_path_length.max_ang_vel);

  // Sampler.

  params->sampler.max_pitch_pert =
      getParamWithDefaultWarning(nh,
                                 "sampler/max_pitch_pert",
                                 params->sampler.max_pitch_pert);
  params->sampler.max_roll_pert =
      getParamWithDefaultWarning(nh,
                                 "sampler/max_roll_pert",
                                 params->sampler.max_roll_pert);
  params->sampler.sample_from_distribution =
      getParamWithDefaultWarning(nh,
                                 "sampler/sample_from_distribution",
                                 params->sampler.sample_from_distribution);
  params->sampler.use_inverse_vertex_density =
      getParamWithDefaultWarning(nh,
                                 "sampler/use_inverse_vertex_density",
                                 params->sampler.use_inverse_vertex_density);
  params->sampler.use_max_prob_unknown_samples =
      getParamWithDefaultWarning(nh,
                                 "sampler/use_max_prob_unknown_samples",
                                 params->sampler.use_max_prob_unknown_samples);
  params->sampler.max_prob_unknown_samples =
      getParamWithDefaultWarning(nh,
                                 "sampler/max_prob_unknown_samples",
                                 params->sampler.max_prob_unknown_samples);
  // Convert degrees to rad.
  params->sampler.max_pitch_pert *= M_PI / 180;
  params->sampler.max_roll_pert *= M_PI / 180;

  // Robot.

  params->robot.base_frame =
      getParamWithDefaultWarning(nh,
                                 "robot/base_frame",
                                 params->robot.base_frame);

  // Robot / Torso.

  params->robot.torso.length =
      getParamWithDefaultWarning(nh,
                                 "robot/torso/length",
                                 params->robot.torso.length);
  params->robot.torso.width =
      getParamWithDefaultWarning(nh,
                                 "robot/torso/width",
                                 params->robot.torso.width);
  params->robot.torso.height =
      getParamWithDefaultWarning(nh,
                                 "robot/torso/height",
                                 params->robot.torso.height);

  // Robot / Torso / Offset.

  params->robot.torso.offset.x =
      getParamWithDefaultWarning(nh,
                                 "robot/torso/offset/x",
                                 params->robot.torso.offset.x);
  params->robot.torso.offset.y =
      getParamWithDefaultWarning(nh,
                                 "robot/torso/offset/y",
                                 params->robot.torso.offset.y);
  params->robot.torso.offset.z =
      getParamWithDefaultWarning(nh,
                                 "robot/torso/offset/z",
                                 params->robot.torso.offset.z);

  // Robot / Feet / Offset.

  params->robot.feet.offset.x =
      getParamWithDefaultWarning(nh,
                                 "robot/feet/offset/x",
                                 params->robot.feet.offset.x);
  params->robot.feet.offset.y =
      getParamWithDefaultWarning(nh,
                                 "robot/feet/offset/y",
                                 params->robot.feet.offset.y);
  params->robot.feet.offset.z =
      getParamWithDefaultWarning(nh,
                                 "robot/feet/offset/z",
                                 params->robot.feet.offset.z);

  // Robot / Feet / Reach.

  params->robot.feet.reach.x =
      getParamWithDefaultWarning(nh,
                                 "robot/feet/reach/x",
                                 params->robot.feet.reach.x);
  params->robot.feet.reach.y =
      getParamWithDefaultWarning(nh,
                                 "robot/feet/reach/y",
                                 params->robot.feet.reach.y);
  params->robot.feet.reach.z =
      getParamWithDefaultWarning(nh,
                                 "robot/feet/reach/z",
                                 params->robot.feet.reach.z);

  // Path following.

  params->path_following.maximum_lookahead =
      getParamWithDefaultWarning(nh,
                                 "path_following/maximum_lookahead",
                                 params->path_following.maximum_lookahead);

  params->verbose = getParamWithDefaultWarning(nh, "verbose", false);

  return params;
}
