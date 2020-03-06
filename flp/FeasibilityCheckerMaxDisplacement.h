#pragma once
#include "FeasibilityChecker.h"

class FeasibilityCheckerMaxDisplacement : public FeasibilityChecker
{
public:
  typedef boost::shared_ptr<FeasibilityCheckerMaxDisplacement> Ptr;
  FeasibilityCheckerMaxDisplacement(double dz) : max_dz_(fabs(dz)) {
    isFast_ = true;
  }
  virtual bool feasible(const State::Ptr& state) {
    return true;
  }
  virtual bool feasible(const StateTransition& transition) {
    double dummy;
    return feasible(transition, dummy);
  }
  virtual bool feasible(const StateTransition& transition, double& validFraction) {
    validFraction = 1.0;
    flp::Transform T1, T2;
    if (!transition.state->space_->getTransform(transition.state, T1))
      return true;
    if (!transition.newState->space_->getTransform(transition.newState, T2))
      return true;
    if (fabs(T2.z - T1.z) <= max_dz_)
      return true;
    // valid fraction
    validFraction = max_dz_ / fabs(T2.z - T1.z);
    return false;
  }
  virtual void updateEnvironment(const Environment::Ptr& environment) {
  }

protected:
  double max_dz_;
};

