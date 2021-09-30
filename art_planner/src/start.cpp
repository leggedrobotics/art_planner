#include "art_planner/start.h"

#include <ompl/base/spaces/SE3StateSpace.h>



void StartState::sampleGoal(ob::State *state) const {
  si_->copyState(state, state_);

  // Check if center of region is valid.
  if (si_->isValid(state)) {
    if (verbose_) {
      std::cout << "Initial start is valid" << std::endl;
    }
    return;
  } else {
    // Try to sample from the goal region.
    if (verbose_) {
      std::cout << "Initial start state not valid. Trying to sample around it." << std::endl;
    }
    std::vector<double> offset(2);
    auto sample_state = state->as<ob::SE3StateSpace::StateType>();
    const auto center_state = state_->as<ob::SE3StateSpace::StateType>();
    for (unsigned int i = 0; i < max_num_samples_; ++i) {
      rng_.uniformInBall(threshold_, offset);
      sample_state->setX(center_state->getX() + offset[0]);
      sample_state->setY(center_state->getY() + offset[1]);
      if (si_->isValid(state)) {
        if (verbose_) {
          std::cout << "Found valid start state offset by " << offset[0] << " " << offset[1] << std::endl;
          std::cout << "Start position " << sample_state->getX() << " " << sample_state->getY() << std::endl;
        }
        return;
      }
    }

    // We could not sample a valid point.
    std::cout << "\nEpic fail, dude" << std::endl;
    state = nullptr;
  }
}



void StartState::setMaxNumSamples(const unsigned int& num_samples) {
  max_num_samples_ = num_samples;
}

void StartState::setVerbosity(bool verbose) {
  verbose_ = verbose;
}
