#pragma once
#include "State.h"
#include "StateSpace.h"

// volume (a box) of feasible 3D positions (relative to base). implemented by the standard lb,ub,nbins
// - in practice this is just a StateSpaceReal with an extra pointer to the state space of the base link (so we can compute world from relative positions)
// - if we want to use a set of primitives then add feasibility checker which checks that separately
// - get neighbors requires resolution to return all possible positions inside box. use bins for that
// - need a special motion checker for ompl that either 1) makes sure only initial and final positions are checked (no interpolation is made)
//   or 2) uses func(start,goal,environment) to decide whether the motion is feasible (advanced)

class StateSpaceContactPoint : public StateSpaceReal
{
public:
  typedef boost::shared_ptr<StateSpaceContactPoint> Ptr;
  typedef boost::shared_ptr<StateSpaceContactPoint const> ConstPtr;
  StateSpaceContactPoint(const std::vector<double>& lb, const std::vector<double>& ub, const std::vector<int>& nbins)
    : StateSpaceReal(lb,ub,nbins) {
  }
  virtual std::vector<State::Ptr> getNeighbors(const State::Ptr& state) const {
    // SpaceContactPoint returns ALL the nbins (or a random number of them if we make that a parameter)
    std::vector<unsigned int> vec = toVectorInt(state);
    std::vector<unsigned int> tmp = vec;
    std::vector<State::Ptr> nei;
    nei.reserve(getNumberOfCells());
    for (unsigned int i = 0; i < nbins_.size(); i++) {
      for (unsigned int j = 0; j < nbins_[i]; j++) {
        tmp[i] = j;
        nei.push_back(fromVectorInt(tmp));
        tmp[i] = vec[i];
      }
    }
    return nei;
  }
  //StateSpace::Ptr base_; // using this we can get positions in the world (externally e.g. inside a feasibility checker)
};

