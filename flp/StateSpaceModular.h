#pragma once
#include "StateSpace.h"

class StateSpaceModular : public StateSpaceCompound<StateCompound>
{
public:
  typedef boost::shared_ptr<StateSpaceModular> Ptr;
  typedef boost::shared_ptr<StateSpaceModular const> ConstPtr;
  StateSpaceModular(const std::vector<StateSpace::Ptr>& spaces, const std::vector<double>& weights) : StateSpaceCompound(spaces, weights) {
  }
  virtual bool getTransform(const State::Ptr& state, flp::Transform& T) const {
    if (dimNamesMap_.size() == 0) return false;
    std::vector<double> vec = toVector(state);
    // get XYZ
    T.x = T.y = T.z = 0;
    auto X   = dimNamesMap_.find("X");
    auto Y   = dimNamesMap_.find("Y");
    auto Z   = dimNamesMap_.find("Z");
    if (X != dimNamesMap_.end()) T.x = vec[X->second];
    if (Y != dimNamesMap_.end()) T.y = vec[Y->second];
    if (Z != dimNamesMap_.end()) T.z = vec[Z->second];
    // get rpy
    double roll(0), pitch(0), yaw(0);
    auto r = dimNamesMap_.find("r");
    auto p = dimNamesMap_.find("p");
    auto y = dimNamesMap_.find("y");
    if (r != dimNamesMap_.end()) roll = vec[r->second];
    if (p != dimNamesMap_.end()) pitch = vec[p->second];
    if (y != dimNamesMap_.end()) yaw = vec[y->second];
    T.setRPY(roll, pitch, yaw);
    return true;
  }
  virtual bool setTransform(State::Ptr& state, const flp::Transform& T) const {
    if (dimNamesMap_.size() == 0) return false;
    std::vector<double> vec = toVector(state);
    // get XYZ
    auto X   = dimNamesMap_.find("X");
    auto Y   = dimNamesMap_.find("Y");
    auto Z   = dimNamesMap_.find("Z");
    if (X != dimNamesMap_.end()) vec[X->second] = T.x;
    if (Y != dimNamesMap_.end()) vec[Y->second] = T.y;
    if (Z != dimNamesMap_.end()) vec[Z->second] = T.z;
    // get rpy
    double roll, pitch, yaw;
    T.getRPY(roll, pitch, yaw);
    auto r = dimNamesMap_.find("r");
    auto p = dimNamesMap_.find("p");
    auto y = dimNamesMap_.find("y");
    if (r != dimNamesMap_.end()) vec[r->second] = roll;
    if (p != dimNamesMap_.end()) vec[p->second] = pitch;
    if (y != dimNamesMap_.end()) vec[y->second] = yaw;
    state = fromVector(vec);
    return true;
  }
  virtual bool getMode(const State::Ptr& state, int& mode) const {
    if (dimNamesMap_.size() == 0) return false;
    auto M = dimNamesMap_.find("M");
    if (M != dimNamesMap_.end()) {
      std::vector<double> vec = toVector(state);
      double dmode = vec[M->second];
      mode = (int)dmode;
      return true;
    }
    return false;
  }
  virtual bool getMode(const State::Ptr& state, Eigen::VectorXd& mode) const {
    if (dimNamesMap_.size() == 0) return false;
    auto M = dimNamesMap_.find("M");
    if (M != dimNamesMap_.end()) {
      std::vector<double> vec = toVector(state);
      double dmode = vec[M->second];
      if (mode.size() != 1) mode.resize(1);
      mode(0) = dmode;
      return true;
    }
    return false;
  }
  virtual std::vector<int> getModes() const {
    auto M = dimNamesToSpacesMap_.find("M");
    if (M != dimNamesToSpacesMap_.end()) {
      return spaces_[M->second]->getModes();
    } else {
      return std::vector<int>();
    }
  }
  virtual State::Ptr projectFrom(const State::Ptr& state, const StateSpace::ConstPtr& source) const {
    // check we are projecting from another modular space
    if (!boost::dynamic_pointer_cast<StateSpaceModular const>(source)) {
      std::cout << "ERROR: Projection not implemented!\n";
      return State::Ptr();
    }
    // state vectors
    State::Ptr newstate = getStateZero();
    std::vector<double> vec = source->toVector(state);
    std::vector<double> newvec = toVector(newstate);
    // copy common sub-space values
    std::map<std::string, unsigned int> sourceDimNamesMap = source->getDimNamesMap();
    for (std::map<std::string, unsigned int>::const_iterator it = dimNamesMap_.begin(); it != dimNamesMap_.end(); ++it) {
      auto m = sourceDimNamesMap.find(it->first);
      if (m != sourceDimNamesMap.end()) {
        newvec[it->second] = vec[m->second];
      }
    }
    return fromVector(newvec);
  }
};

