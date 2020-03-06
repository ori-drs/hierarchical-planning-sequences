#pragma once
#include "FeasibilityChecker.h"

class FeasibilityCheckerDistanceToStates : public FeasibilityChecker
{
public:
  typedef boost::shared_ptr<FeasibilityCheckerDistanceToStates> Ptr;
  FeasibilityCheckerDistanceToStates(const std::vector<State::Ptr>& states, double maxDistance) : states_(states), maxDistance_(maxDistance) {
  }
  FeasibilityCheckerDistanceToStates(const std::vector<State::Ptr>& states, double maxDistance, const std::vector<double>& weights) : states_(states), maxDistance_(maxDistance), weights_(weights) {
  }
  virtual bool feasible(const State::Ptr& state) {
    if (states_.size() == 0) return true;
    //double mymin = 1e10;
    //int imymin = 0;
    State::Ptr proj = states_[0]->space_->projectFrom(state, state->space_); // TODO: check whether all states_[i] are in the same space
    if (!proj) return false;
    for (unsigned int i = 0; i < states_.size(); i++) {
      double dist = 0;
      if (weights_.size() == 0)
        dist = proj->space_->distance(proj, states_[i]);
      else
        dist = proj->space_->distance(proj, states_[i], weights_);
      if (dist <= maxDistance_) return true;
      //if (dist < mymin) { mymin = dist; imymin = i; }
    }
    //State::Ptr proj = states_[imymin]->space_->projectFrom(state, state->space_);
    //std::cout << "STATE NOT IN SET!! " << state->space_->toString(state) << " = " << proj->space_->toString(proj) << "  to reference state  " << states_[imymin]->space_->toString(states_[imymin]) << "  distance is = " << mymin << "\n";
    return false;
  }
  virtual bool feasible(const StateTransition& transition) {
    if (states_.size() == 0) return true;
    return feasible(transition.newState) && feasible(transition.state);
  }
  virtual bool feasible(const StateTransition& transition, double& validFraction) {
    if (feasible(transition)) {
      validFraction = 1.0;
      return true;
    } else {
      validFraction = 0.0;
      return false;
    }
  }

protected:
  std::vector<State::Ptr> states_;
  double maxDistance_;
  std::vector<double> weights_;
};

