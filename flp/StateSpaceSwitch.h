#pragma once
#include "State.h"
#include "StateSpace.h"
#include "StateSpaceGraph.h"

struct StateSwitch : public StateCompound
{
  typedef boost::shared_ptr<StateSwitch> Ptr;
  StateSwitch() {
  }
  StateSwitch(const StateSpaceConstPtr& space) : StateCompound(space) {
  }
  virtual State::Ptr clone() const {
    StateSwitch::Ptr state( new StateSwitch(space_) );
    state->weights_ = weights_;
    for (unsigned int i = 0; i < states_.size(); i++)
      state->states_.push_back(states_[i]->clone());
    return state;
  }
};

class StateSpaceSwitch : public StateSpaceCompound<StateSwitch>
{
public:
  typedef boost::shared_ptr<StateSpaceSwitch> Ptr;
  typedef boost::shared_ptr<StateSpaceSwitch const> ConstPtr;
  StateSpaceSwitch(const std::vector<StateSpace::Ptr>& spaces, const std::vector<double>& weights) : StateSpaceCompound(spaces, weights) {
    assert(spaces.size() == weights.size());
    assert(spaces.size() >= 2);
    assert(boost::dynamic_pointer_cast<StateSpaceGraph>(spaces[0]));
  }
  virtual State::Ptr interpolate(const State::Ptr& state1, const State::Ptr& state2, double fraction) const {
    State::Ptr state3 = state1->clone();
    const StateSwitch& s1 = *boost::static_pointer_cast<StateSwitch>(state1);
    const StateSwitch& s2 = *boost::static_pointer_cast<StateSwitch>(state2);
    StateSwitch& s3 = *boost::static_pointer_cast<StateSwitch>(state3);
    // interpolate only active space
    int activeSpace1 = boost::static_pointer_cast<StateGraph>(s1.states_[0])->value_;
    s3.states_[activeSpace1] = spaces_[activeSpace1]->interpolate(s1.states_[activeSpace1], s2.states_[activeSpace1], fraction);
    // get the list of possible switch values for the next state
    std::vector<State::Ptr> nei = spaces_[0]->getNeighbors(s1.states_[0]);
    // the target will decide which value to use (in case there are several edges on the current graph vertex) i.e. in OMPL we will sample graph transitions uniformly
    int activeSpace2 = boost::static_pointer_cast<StateGraph>(s2.states_[0])->value_;
    s3.states_[0] = nei[activeSpace2 % nei.size()];
    //boost::static_pointer_cast<StateGraph>(s3.states_[0])->value_ = activeSpace2;
    return state3;
  }
  virtual std::vector<State::Ptr> getNeighbors(const State::Ptr& state) const {
    const StateSwitch& s = *boost::static_pointer_cast<StateSwitch>(state);
    // get the neighbors of the active part of the state
    int activeSpace = boost::static_pointer_cast<StateGraph>(s.states_[0])->value_;
    std::vector<State::Ptr> activeNei = spaces_[activeSpace]->getNeighbors(s.states_[activeSpace]);
    // get the list of possible switch values for the next state
    std::vector<State::Ptr> switchNei = spaces_[0]->getNeighbors(s.states_[0]);
    // get all neighbors (combinations of active-state-value and switch-value)
    std::vector<State::Ptr> nei;
    for (unsigned int i = 0; i < activeNei.size(); i++) {
      for (unsigned int v = 0; v < switchNei.size(); v++) {
        StateSwitch::Ptr newstate = boost::static_pointer_cast<StateSwitch>(state->clone());
        newstate->states_[activeSpace] = activeNei[i]->clone();
        newstate->states_[0] = switchNei[v]->clone();
        int newActiveSpace = boost::static_pointer_cast<StateGraph>(newstate->states_[0])->value_;
        assert(newActiveSpace >= 1 && newActiveSpace < spaces_.size());
        nei.push_back(newstate);
      }
    }
    return nei;
  }
};

