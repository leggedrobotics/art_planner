#include "art_planner/map/processors/probability_distribution.h"



using namespace art_planner::processors;



void art_planner::processors::applyBaseSampleDistribution(const art_planner::GridMapPtr& map) {
  if (!map->exists("sample_probability")) {
    map->add("sample_probability", 1.0);
  }
  if (map->exists("traversability_sample_filter")) {
    map->get("sample_probability").array() *= map->get("traversability_sample_filter").array();
  }
}



void art_planner::processors::computeCumulativeProbabilityDistribution(const art_planner::GridMapPtr& map) {
  const auto& prob = map->get("sample_probability");

  Eigen::Matrix<grid_map::DataType, Eigen::Dynamic, 1> prob_rowwise = prob.rowwise().sum();

  // Normalize prob vectors such that they sum to one.
  prob_rowwise.array() /= prob_rowwise.sum();
  grid_map::Matrix cum_prob = prob;
  cum_prob.array().colwise() /= cum_prob.array().rowwise().sum();


  // Compute cumulative distributions.
  Eigen::Matrix<grid_map::DataType, Eigen::Dynamic, 1> cum_prob_rowwise = prob_rowwise;
  for (Eigen::Index i = 1; i < cum_prob_rowwise.rows(); ++i) {
    cum_prob_rowwise(i, 0) += cum_prob_rowwise(i-1, 0);
  }

  for (Eigen::Index i = 1; i < cum_prob.cols(); ++i) {
    cum_prob.col(i) += cum_prob.col(i-1);
  }

  grid_map::Matrix cum_prob_rowwise_hack = cum_prob;
  cum_prob_rowwise_hack.colwise() = cum_prob_rowwise;

  map->add("cum_prob", cum_prob);
  map->add("cum_prob_rowwise_hack", cum_prob_rowwise_hack);
}



void art_planner::processors::applyMaxUnknownProbability(const art_planner::GridMapPtr& map,
                                                         double max_prob_unknown_samples) {
  auto& prob = map->get("sample_probability");
  const auto& valid_mask = map->get("observed");

  grid_map::Matrix prob_unknown_mult(prob.rows(), prob.cols());

  double cum_prob_known = 0;
  std::vector<std::pair<Eigen::Index, Eigen::Index> > ind_trav_known;
  double cum_prob_unknown = 0;
  std::vector<std::pair<Eigen::Index, Eigen::Index> > ind_trav_unknown;
  for (Eigen::Index i = 0; i < prob.rows(); ++i) {
    for (Eigen::Index j = 0; j < prob.cols(); ++j) {
      if (valid_mask(i, j) > 0) {
        cum_prob_known += prob(i, j);
        ind_trav_known.push_back(std::make_pair(i, j));
      } else {
        cum_prob_unknown += prob(i, j);
        ind_trav_unknown.push_back(std::make_pair(i, j));
      }
    }
  }

  const double base_prob_unknown = cum_prob_unknown / (cum_prob_known + cum_prob_unknown);

  if (cum_prob_known > 0 && cum_prob_unknown > 0 && base_prob_unknown > max_prob_unknown_samples) {
    prob_unknown_mult.setZero();
    const auto known_mult = (1 - max_prob_unknown_samples) / cum_prob_known;
    const auto unknown_mult = max_prob_unknown_samples / cum_prob_unknown;
    for (const auto& ind: ind_trav_known) {
      prob_unknown_mult(ind.first, ind.second) = known_mult;
    }
    for (const auto& ind: ind_trav_unknown) {
      prob_unknown_mult(ind.first, ind.second) = unknown_mult;
    }
  } else {
    prob_unknown_mult.setOnes();
  }

  map->add("prob_unknown_mult", prob_unknown_mult);
  prob.array() *= prob_unknown_mult.array();
}
