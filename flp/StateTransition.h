#pragma once
#include "State.h"
#include "StateSpace.h"
#include "Action.h"
#include "ActionSpace.h"
#include <limits>

//---------------------------------------------
// transition

struct StateTransition
{
  typedef boost::shared_ptr<StateTransition> Ptr;

  StateTransition() : cost(std::numeric_limits<double>::infinity()), duration(1.0) {}

  State::Ptr state;
  Action::Ptr action;
  State::Ptr newState;
  double cost;
  double duration;
  std::vector<double> costs;
  // TODO: could also add something like warm-start for lower-level planner/controller
};

//---------------------------------------------
// machine

class StateTransitionMachine
{
public:
  typedef boost::shared_ptr<StateTransitionMachine> Ptr;

  virtual State::Ptr getNextState(const State::Ptr& state, const Action::Ptr& action, StateTransition& data) const = 0;
  virtual ActionSpace::Ptr getAccessibleActionSpaceOption(const State::Ptr& state, const ActionSpace::Ptr& actionSpace) const = 0;
  virtual ActionSpace::Ptr getAccessibleActionSpaceControl(const State::Ptr& state, const ActionSpace::Ptr& actionSpace) const = 0;
};

//---------------------------------------------
// comparators

struct StateTransitionComparator {
  virtual bool operator() (const StateTransition& st1, const StateTransition& st2) = 0;
};
struct StateTransitionComparatorCost : public StateTransitionComparator {
  bool operator() (const StateTransition& st1, const StateTransition& st2) {
    return st1.cost < st2.cost;
  }
};
struct StateTransitionComparatorTarget : public StateTransitionComparator {
  StateTransitionComparatorTarget(const State::Ptr& target) : target_(target) {}
  bool operator() (const StateTransition& st1, const StateTransition& st2) {
    return st1.newState->space_->distance(st1.newState, target_) < st2.newState->space_->distance(st2.newState, target_);
  }
  State::Ptr target_;
};

