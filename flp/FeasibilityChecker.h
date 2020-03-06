#pragma once
#include "State.h"
#include "StateSpace.h"
#include "StateTransition.h"
#include "Environment.h"

class FeasibilityChecker
{
public:
  typedef boost::shared_ptr<FeasibilityChecker> Ptr;
  FeasibilityChecker() : isFast_(false) {}
  virtual bool feasible(const State::Ptr& state) = 0;
  virtual bool feasible(const StateTransition& transition) = 0;
  virtual bool feasible(const StateTransition& transition, double& validFraction) = 0;
  virtual void updateEnvironment(const Environment::Ptr& environment) {}
  bool isFast_;
};

