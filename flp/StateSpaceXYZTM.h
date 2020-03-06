#pragma once
#include "State.h"
#include "StateSpace.h"

struct StateXYZTM : public StateCompound
{
  typedef boost::shared_ptr<StateXYZTM> Ptr;
  StateXYZTM() {
  }
  StateXYZTM(const StateSpaceConstPtr& space) : StateCompound(space) {
  }
  virtual State::Ptr clone() const {
    StateXYZTM::Ptr state( new StateXYZTM(space_) );
    state->weights_ = weights_;
    for (unsigned int i = 0; i < states_.size(); i++)
      state->states_.push_back(states_[i]->clone());
    return state;
  }
  flp::Transform getTransform() const {
    const StateReal& xyz = *boost::static_pointer_cast<StateReal>(states_[0]);
    const StateSO2& theta = *boost::static_pointer_cast<StateSO2>(states_[1]);
    flp::Transform T;
    T.x = xyz.values_[0];
    T.y = xyz.values_[1];
    T.z = xyz.values_[2];
    T.setRPY(0,0,theta.value_);
    return T;
  }
};

class StateSpaceXYZTM : public StateSpaceCompound<StateXYZTM>
{
public:
  typedef boost::shared_ptr<StateSpaceXYZTM> Ptr;
  typedef boost::shared_ptr<StateSpaceXYZTM const> ConstPtr;
  StateSpaceXYZTM(const std::vector<double>& XYZlb, const std::vector<double>& XYZub, const std::vector<int>& XYZnbins, int SO2nbins, int nmodes)
    : StateSpaceCompound(initS(XYZlb,XYZub,XYZnbins,SO2nbins,nmodes), initW()) {
    assert(XYZlb.size() == 3 && XYZub.size() == 3 && XYZnbins.size() == 3);
  }
  static std::vector<StateSpace::Ptr> initS(const std::vector<double>& XYZlb, const std::vector<double>& XYZub, const std::vector<int>& XYZnbins, int SO2nbins, int nmodes) {
    std::vector<StateSpace::Ptr> spaces;
    spaces.push_back(StateSpace::Ptr( new StateSpaceReal(XYZlb,XYZub,XYZnbins) ));
    spaces.push_back(StateSpace::Ptr( new StateSpaceSO2(SO2nbins) ));
    spaces.push_back(StateSpace::Ptr( new StateSpaceDiscreteCircular(nmodes) ));
    return spaces;
  }
  static std::vector<double> initW() {
    std::vector<double> w;
    w.push_back(1.0);
    w.push_back(0.5);
    w.push_back(0.0);
    return w;
  }
  virtual bool getTransform(const State::Ptr& state, flp::Transform& T) const {
    const StateXYZTM& s = *boost::static_pointer_cast<StateXYZTM>(state);
    const StateReal& xyz = *boost::static_pointer_cast<StateReal>(s.states_[0]);
    const StateSO2& theta = *boost::static_pointer_cast<StateSO2>(s.states_[1]);
    T.x = xyz.values_[0];
    T.y = xyz.values_[1];
    T.z = xyz.values_[2];
    T.setRPY(0,0,theta.value_);
    return true;
  }
  virtual bool setTransform(State::Ptr& state, const flp::Transform& T) const {
    StateXYZTM& s = *boost::static_pointer_cast<StateXYZTM>(state);
    StateReal& xyz = *boost::static_pointer_cast<StateReal>(s.states_[0]);
    StateSO2& theta = *boost::static_pointer_cast<StateSO2>(s.states_[1]);
    xyz.values_[0] = T.x;
    xyz.values_[1] = T.y;
    xyz.values_[2] = T.z;
    double tmp1,tmp2;
    T.getRPY(tmp1,tmp2,theta.value_);
    return true;
  }
  virtual bool getMode(const State::Ptr& state, int& mode) const {
    StateXYZTM& s = *boost::static_pointer_cast<StateXYZTM>(state);
    const StateDiscreteCircular& m = *boost::static_pointer_cast<StateDiscreteCircular>(s.states_[2]);
    mode = m.value_;
    return true;
  }
  virtual bool getMode(const State::Ptr& state, Eigen::VectorXd& mode) const {
    StateXYZTM& s = *boost::static_pointer_cast<StateXYZTM>(state);
    const StateDiscreteCircular& m = *boost::static_pointer_cast<StateDiscreteCircular>(s.states_[2]);
    if (mode.size() != 1) mode.resize(1);
    mode(0) = m.value_;
    return true;
  }
  virtual std::vector<int> getModes() const {
    return spaces_[2]->getModes();
  }
  virtual std::vector<State::Ptr> getNeighbors(const State::Ptr& state) const {
    // attempt a partial product...
    const StateXYZTM& s = *boost::static_pointer_cast<StateXYZTM>(state);
    std::vector<State::Ptr> nei_xyz = spaces_[0]->getNeighbors(s.states_[0]);
    std::vector<State::Ptr> nei_so2 = spaces_[1]->getNeighbors(s.states_[1]);
    std::vector<State::Ptr> nei_mod = spaces_[2]->getNeighbors(s.states_[2]);
    std::vector<State::Ptr> nei;
    for (unsigned int n = 0; n < nei_xyz.size(); n++) {
      // xyz changes, mode as before (assuming neighbors dont include itself)
      boost::shared_ptr<StateXYZTM> newstate1= boost::static_pointer_cast<StateXYZTM>(s.clone());
      newstate1->states_[0] = nei_xyz[n];
      nei.push_back(newstate1);
      // xyz changes, mode as well (product)
      for (unsigned int i = 0; i < nei_mod.size(); i++) {
        boost::shared_ptr<StateXYZTM> newstate = boost::static_pointer_cast<StateXYZTM>(s.clone());
        newstate->states_[0] = nei_xyz[n];
        newstate->states_[2] = nei_mod[i];
        nei.push_back(newstate);
      }
    }
    for (unsigned int n = 0; n < nei_so2.size(); n++) {
      // so2 changes, mode as before (assuming neighbors dont include itself)
      boost::shared_ptr<StateXYZTM> newstate1= boost::static_pointer_cast<StateXYZTM>(s.clone());
      newstate1->states_[1] = nei_so2[n];
      nei.push_back(newstate1);
      // so2 changes, mode as well (product)
      for (unsigned int i = 0; i < nei_mod.size(); i++) {
        boost::shared_ptr<StateXYZTM> newstate = boost::static_pointer_cast<StateXYZTM>(s.clone());
        newstate->states_[1] = nei_so2[n];
        newstate->states_[2] = nei_mod[i];
        nei.push_back(newstate);
      }
    }
    return nei;
  }
  virtual State::Ptr projectFrom(const State::Ptr& state, const StateSpace::ConstPtr& source) const {
    if (boost::dynamic_pointer_cast<StateXYZTM>(state))
      return state->clone();
    // get 6D pose
    flp::Transform T;
    if (!source->getTransform(state, T))
      return State::Ptr();
    // projected state: set 6D pose
    State::Ptr proj = getStateZero();
    if (!setTransform(proj, T))
      return State::Ptr();
    return projectToFeasible(proj);
  }
};

