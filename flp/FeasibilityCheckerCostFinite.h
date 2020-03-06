#pragma once
#include "FeasibilityChecker.h"
#include "StateTransitionCost.h"

class FeasibilityCheckerCostFinite : public FeasibilityChecker
{
public:
  typedef boost::shared_ptr<FeasibilityCheckerCostFinite> Ptr;
  FeasibilityCheckerCostFinite(const StateTransitionCost::Ptr& cost) : cost_(cost) {
  }
  virtual bool feasible(const State::Ptr& state) {
    assert(environment_);
    double cost = cost_->evaluate(environment_, state);
    return std::isfinite(cost);
  }
  virtual bool feasible(const StateTransition& transition) {
    assert(environment_);
    StateTransition trans = transition;
    double cost = cost_->evaluate(environment_, trans);
    return std::isfinite(cost);
  }
  virtual bool feasible(const StateTransition& transition, double& validFraction) {
    assert(environment_);
    StateTransition trans = transition;
    double cost = cost_->evaluate(environment_, trans);
    if (std::isfinite(cost)) {
      validFraction = 1.0;
      return true;
    } else {
      validFraction = 0.0;
      return false;
    }
  }
  virtual void updateEnvironment(const Environment::Ptr& environment) {
    environment_ = environment;
  }

protected:
  StateTransitionCost::Ptr cost_;
  Environment::Ptr environment_;
};

