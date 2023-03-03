#pragma once

#include <string>
#include <cmath>
#include <memory>



namespace art_planner{



// For parameter description see "params.yaml".
struct Params {

  struct {
    std::string    name{"lazy_prm_star"};
    std::string    elevation_layer{"elevation"};
    std::string    traversability_layer{"traversability"};
    double         plan_time{1.0};
    unsigned int   n_threads{1};
    double         replan_freq{1.0};
    float          traversability_thres{0.5f};
    bool           simplify_solution{true};
    bool           snap_goal_to_map{true};
    bool           unknown_space_untraversable{true};

    struct {
      double       foothold_margin{0.0};
      double       foothold_margin_max_hole_size{0.0};
      double       foothold_margin_max_drop{0.0};
      double       foothold_margin_max_drop_search_radius{0.0};
      double       foothold_margin_min_step{0.0};
      double       foothold_size{0.0};
    } safety;

    struct {
      double       start_radius{0.0};
      double       goal_radius{0.0};
      unsigned int n_iter{0};
    } start_goal_search;

    struct {
      bool         invalidate_updated_graph_components{false};
      float        height_change_for_update{0.05f};
      bool         cleanup_when_not_planning{false};
    } lazy_prm_star_min_update;

    struct {
      double max_sample_time{2.0};
      unsigned int max_n_vertices{10000u};
      unsigned int max_n_edges{50000u};
      unsigned int recompute_density_after_n_samples{1000u};
      float max_query_edge_length{0.5};
      float risk_threshold{0.1};

      struct {
        float energy{0.0};
        float time{1.0};
        float risk{5.0};
      } cost_weights;

    } prm_motion_cost;

  } planner;

  struct {

    struct {
      bool         use_directional_cost{false};
      double       max_lon_vel{0.5};
      double       max_lat_vel{0.1};
      double       max_ang_vel{0.5};
    } custom_path_length;

  } objectives;

  struct {
    double         max_pitch_pert{10.0 / 180*M_PI};
    double         max_roll_pert{3.33 / 180*M_PI};
    bool           sample_from_distribution{true};
    bool           use_inverse_vertex_density{false};
    bool           use_max_prob_unknown_samples{false};
    double         max_prob_unknown_samples{0.1};
  } sampler;

  struct {
    std::string    base_frame{"base"};

    struct {
      double         length{1.05};
      double         width{0.55};
      double         height{0.2};

      struct {
        double         x{0.0};
        double         y{0.0};
        double         z{0.0};
      } offset;

    } torso;

    struct {

      struct {
        double         x{0.362};
        double         y{0.225};
        double         z{-0.525};
      } offset;

      struct {
        double         x{0.25};
        double         y{0.1};
        double         z{0.15};
      } reach;

    } feet;

  } robot;

  bool verbose{false};

};



using ParamsPtr = std::shared_ptr<Params>;
using ParamsConstPtr = std::shared_ptr<const Params>;



}
