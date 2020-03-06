#pragma once
#include "State.h"
#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <random>
#include <vector>
#include <iostream>

class StateSpace : public boost::enable_shared_from_this<StateSpace>
{
public:
  typedef boost::shared_ptr<StateSpace> Ptr;
  typedef boost::shared_ptr<StateSpace const> ConstPtr;
  StateSpace(const std::vector<double>& lb, const std::vector<double>& ub, const std::vector<int>& nbins) : lb_(lb), ub_(ub), nbins_(nbins) {
    assert(lb_.size() == ub_.size() && lb_.size() == nbins.size());
    dim_ = lb_.size();
  }
  StateSpace(const std::vector<StateSpace::Ptr>& spaces, const std::vector<double>& weights) : spaces_(spaces), weights_(weights) {
    for (unsigned int i = 0; i < spaces.size(); i++) {
      std::vector<double> lb,ub;
      std::vector<int> nbins;
      spaces[i]->getBounds(lb,ub,nbins);
      lb_.insert(lb_.end(), lb.begin(), lb.end());
      ub_.insert(ub_.end(), ub.begin(), ub.end());
      nbins_.insert(nbins_.end(), nbins.begin(), nbins.end());
    }
    assert(lb_.size() == ub_.size() && lb_.size() == nbins_.size());
    assert(spaces_.size() == weights_.size());
    dim_ = lb_.size();
  }
  virtual State::Ptr applyNormalizedChange(const State::Ptr& state, const std::vector<double>& control) const {
    // control will be in [0,1]
    std::vector<double> vec = toVector(state);
    for (unsigned int i = 0; i < vec.size(); i++)
      vec[i] = control[i] * (ub_[i] - lb_[i]) + lb_[i];
    return fromVector(vec);
  }
  virtual std::vector<double> getNormalizedChange(const State::Ptr& state1, const State::Ptr& state2) const {
    // control will be in [0,1]
    std::vector<double> control = toVector(state2);
    for (unsigned int i = 0; i < control.size(); i++)
      control[i] = (control[i] - lb_[i]) / (ub_[i] - lb_[i]);
    return control;
  }
  virtual void getBounds(std::vector<double>& lb, std::vector<double>& ub, std::vector<int>& nbins) const {
    lb = lb_;
    ub = ub_;
    nbins = nbins_; 
  }
  virtual unsigned int getDimension() const {
    return dim_;
  }
  virtual unsigned int getNumberOfCells() const {
    int num = nbins_[0];
    for (unsigned int i = 1; i < nbins_.size(); i++)
      num *= nbins_[i];
    return num;
  }
  virtual unsigned int getMaximumExtent() const {
    double ext = 0;
    for (unsigned int i = 0; i < dim_; i++)
      ext += ub_[i] - lb_[i]; // by default use conservative upper bound (manhattan distance)
    return ext;
  }
  virtual bool getTransform(const State::Ptr& state, flp::Transform& T) const {
    return false;
  }
  virtual bool setTransform(State::Ptr& state, const flp::Transform& T) const {
    return false;
  }
  virtual bool getContacts(const State::Ptr& state, std::vector<Eigen::Vector3d>& contacts) const {
    return false;
  }
  virtual bool setContacts(State::Ptr& state, const std::vector<Eigen::Vector3d>& contacts) const {
    return false;
  }
  virtual bool getMode(const State::Ptr& state, int& mode) const {
    return false;
  }
  virtual bool getMode(const State::Ptr& state, Eigen::VectorXd& mode) const {
    return false;
  }
  virtual std::map<std::string, unsigned int> getDimNamesMap() const {
    return dimNamesMap_;
  }
  virtual std::map<std::string, unsigned int> getDimNamesToSpacesMap() const {
    return dimNamesToSpacesMap_;
  }
  virtual std::vector<std::string> getDimNames() const {
    return dimNames_;
  }
  virtual bool setDimNames(const std::vector<std::string>& names, const std::vector<unsigned int>& spaceIndices) {
    dimNames_ = names;
    dimNamesMap_.clear();
    for (unsigned int i = 0; i < names.size(); i++) {
      dimNamesMap_[names[i]] = i;
      dimNamesToSpacesMap_[names[i]] = spaceIndices[i];
    }
    return true;
  }
  virtual std::vector<int> getModes() const {
    return std::vector<int>();
  }
  virtual bool getFeasibility(const State::Ptr& state) const {
    std::vector<double> vec = toVector(state);
    for (unsigned int i = 0; i < dim_; i++)
      if (vec[i] < lb_[i] || vec[i] > ub_[i])
        return false;
    return true;
  }
  virtual bool isMotionInstantaneous() const {
    return false;
  }
  virtual State::Ptr interpolate(const State::Ptr& state1, const State::Ptr& state2, double fraction) const {
    std::vector<double> vec1 = state1->space_->toVector(state1);
    std::vector<double> vec2 = state2->space_->toVector(state2);
    assert(vec1.size() == vec2.size());
    assert(vec1.size() == dim_);
    std::vector<double> vec(vec1.size());
    for (unsigned int i = 0; i < vec.size(); i++)
      vec[i] = vec1[i]*(1-fraction) + vec2[i]*fraction;
    return fromVector(vec);
  }
  virtual std::vector<State::Ptr> getNeighbors(const State::Ptr& state) const {
    std::vector<State::Ptr> nei;
    std::vector<unsigned int> vec = toVectorInt(state);
    for (unsigned int i = 0; i < dim_; i++) {
      if (vec[i] > 0) {
        vec[i]--;
        nei.push_back(fromVectorInt(vec));
        vec[i]++;
      }
      if (vec[i] < nbins_[i]-1) {
        vec[i]++;
        nei.push_back(fromVectorInt(vec));
        vec[i]--;
      }
    }
    return nei;
  }
  virtual State::Ptr fromVectorInt(const std::vector<unsigned int>& ivec) const {
    std::vector<double> vec(dim_);
    double binsize;
    for (unsigned int i = 0; i < dim_; i++) {
      binsize = (ub_[i] - lb_[i]) / nbins_[i];
      vec[i] = lb_[i] + ivec[i] * binsize + binsize/2; // center of bin
    }
    return fromVector(vec);
  }
  virtual std::vector<unsigned int> toVectorInt(const State::Ptr& state) const {
    std::vector<double> vec = toVector(state);
    std::vector<unsigned int> ivec(dim_);
    double tmp;
    for (unsigned int i = 0; i < dim_; i++) {
      tmp = (vec[i] - lb_[i]) / ((ub_[i]-lb_[i])/nbins_[i]);
      if (tmp >= nbins_[i])
        ivec[i] = nbins_[i]-1;
      else if (tmp <= 0)
        ivec[i] = 0;
      else
        ivec[i] = tmp;
    }
    return ivec;
  }
  virtual std::string toString(const State::Ptr& state) const {
    std::vector<double> vec = toVector(state);
    std::stringstream ss;
    for (unsigned int i = 0; i < vec.size(); i++)
      ss << vec[i] << " ";
    return ss.str();
  }
  virtual State::Ptr project(const State::Ptr& state, const StateSpace::ConstPtr& target) const {
    return target->projectFrom(state, shared_from_this());
  }
  virtual State::Ptr projectFrom(const State::Ptr& state, const StateSpace::ConstPtr& source) const {
    std::cout << "ERROR: Projection not implemented!\n";
    return State::Ptr();
  }
  virtual std::vector<StateSpace::Ptr> getSubSpaces() const { return spaces_; }
  virtual std::vector<double> getSubSpaceWeights() const { return weights_; }

  virtual State::Ptr getStateZero() const = 0;
  virtual State::Ptr sampleUniform(std::mt19937& generator) const = 0;
  virtual State::Ptr projectToFeasible(const State::Ptr& state) const = 0;
  virtual State::Ptr fromVector(const std::vector<double>& vec) const = 0;
  virtual std::vector<double> toVector(const State::Ptr& state) const = 0;
  virtual double distance(const State::Ptr& state1, const State::Ptr& state2) const = 0;
  virtual double distance(const State::Ptr& state1, const State::Ptr& state2, const std::vector<double>& weights) const { return distance(state1, state2); }

protected:
  std::vector<std::string> dimNames_; // names (string IDs) for each of the dimensions
  std::map<std::string, unsigned int> dimNamesMap_; // map from dimension name (string ID) to vector index
  std::map<std::string, unsigned int> dimNamesToSpacesMap_; // map from dimension name (string ID) to state space index
  std::vector<double> lb_; // lower bounds
  std::vector<double> ub_; // upper bounds
  std::vector<int> nbins_; // number of bins within bounds when discretizing space
  unsigned int dim_; // dimension of the space
  std::vector<StateSpace::Ptr> spaces_;
  std::vector<double> weights_;
};


//-----------------------------------------------------------------------------------------
// particular state spaces


class StateSpaceReal : public StateSpace
{
public:
  typedef boost::shared_ptr<StateSpaceReal> Ptr;
  typedef boost::shared_ptr<StateSpaceReal const> ConstPtr;
  StateSpaceReal(const std::vector<double>& lb, const std::vector<double>& ub, const std::vector<int>& nbins) : StateSpace(lb,ub,nbins) {
    zero_.resize(dim_);
    for (unsigned int i = 0; i < dim_; i++)
      zero_[i] = (lb_[i] + ub_[i]) / 2;
  }
  virtual State::Ptr getStateZero() const {
    return State::Ptr( new StateReal(zero_, shared_from_this()) );
  }
  virtual State::Ptr sampleUniform(std::mt19937& generator) const {
    StateReal::Ptr state( new StateReal(std::vector<double>(dim_,0.0), shared_from_this()) );
    for (unsigned int i = 0; i < dim_; i++) {
      std::uniform_real_distribution<double> dis(lb_[i], ub_[i]);
      state->values_[i] = dis(generator);
    }
    return state;
  }
  virtual State::Ptr projectToFeasible(const State::Ptr& state) const {
    const StateReal::Ptr& s = boost::static_pointer_cast<StateReal>(state);
    std::vector<double> newvalues(dim_);
    for (unsigned int i = 0; i < dim_; i++) {
      newvalues[i] = std::max(std::min(s->values_[i],ub_[i]),lb_[i]);
    }
    return StateReal::Ptr( new StateReal(newvalues, shared_from_this()) );
  }
  virtual State::Ptr fromVector(const std::vector<double>& vec) const {
    return State::Ptr( new StateReal(vec, shared_from_this()) );
  }
  virtual std::vector<double> toVector(const State::Ptr& state) const {
    return boost::static_pointer_cast<StateReal>(state)->values_;
  }
  virtual double distance(const State::Ptr& state1, const State::Ptr& state2) const {
    double distance = 0;
    const StateReal& s1 = *boost::static_pointer_cast<StateReal>(state1);
    const StateReal& s2 = *boost::static_pointer_cast<StateReal>(state2);
    for (unsigned int i = 0; i < dim_; i++)
      distance += pow(s1.values_[i] - s2.values_[i], 2.0);
    return sqrt(distance);
  }
protected:
  std::vector<double> zero_;
};


class StateSpaceDiscrete : public StateSpace
{
public:
  typedef boost::shared_ptr<StateSpaceDiscrete> Ptr;
  typedef boost::shared_ptr<StateSpaceDiscrete const> ConstPtr;
  StateSpaceDiscrete(int lb, int ub) : StateSpace(std::vector<double>(1,lb), std::vector<double>(1,ub+0.9999), std::vector<int>(1,ub-lb+1)) {
    assert(lb <= ub);
  }
  StateSpaceDiscrete(int n) : StateSpace(std::vector<double>(1,0), std::vector<double>(1,n-1+0.9999), std::vector<int>(1,n)) {
    assert(n > 0);
  }
  //virtual State::Ptr interpolate(const State::Ptr& state1, const State::Ptr& state2, double fraction) const { //TODO: test see if OMPL changes discrete states more
  //  StateDiscrete::Ptr state = boost::static_pointer_cast<StateDiscrete>(getStateZero());
  //  double v1 = boost::static_pointer_cast<StateDiscrete>(state1)->value_;
  //  double v2 = boost::static_pointer_cast<StateDiscrete>(state2)->value_;
  //  if (v1 < v2) {
  //    v1 = 0.0000 + (int)v1;
  //    v2 = 0.9999 + (int)v2;
  //  }
  //  if (v2 < v1) {
  //    v2 = 0.0000 + (int)v2;
  //    v1 = 0.9999 + (int)v1;
  //  }
  //  v1 = (int)v1;
  //  v2 = (int)v2 + 0.9999;
  //  double v  = v1*(1-fraction) + v2*fraction;
  //  return State::Ptr( new StateDiscrete(v) );
  //}
  //virtual State::Ptr interpolate(const State::Ptr& state1, const State::Ptr& state2, double fraction) const { //TODO: test see if OMPL changes discrete states more
  //  StateDiscrete::Ptr state = boost::static_pointer_cast<StateDiscrete>(getStateZero());
  //  double v1 = boost::static_pointer_cast<StateDiscrete>(state1)->value_;
  //  double v2 = boost::static_pointer_cast<StateDiscrete>(state2)->value_;
  //  if (fraction > 0)
  //    return State::Ptr( new StateDiscrete(v2) );
  //  else
  //    return State::Ptr( new StateDiscrete(v1) );
  //  int iv1 = v1;
  //  int iv2 = v2;
  //  if (v1 == (double)iv1) {
  //    if (fraction > 0)
  //      return State::Ptr( new StateDiscrete(iv2) );
  //    else
  //      return State::Ptr( new StateDiscrete(iv1) );
  //  }
  //  if (v2 == (double)iv2) {
  //    if (fraction < 1)
  //      return State::Ptr( new StateDiscrete(iv2) );
  //    else
  //      return State::Ptr( new StateDiscrete(iv1) );
  //  }
  //  printf("interpolate inputs unexpected (not integer)\n");
  //  return State::Ptr( new StateDiscrete(iv2) );
  //}
  //TODO: discrete getneighbors should get all cells? or there should be StateDiscreteOrdered, StateDiscreteUnordered
  virtual std::vector<int> getModes() const {
    int lb = lb_[0];
    int ub = ub_[0];
    std::vector<int> modes(ub-lb+1);
    for (int i = lb; i <= ub; i++)
      modes[i-lb] = i;
    return modes;
  }
  virtual State::Ptr getStateZero() const {
    return State::Ptr( new StateDiscrete(lb_[0], shared_from_this()) );
  }
  virtual State::Ptr sampleUniform(std::mt19937& generator) const {
    std::uniform_real_distribution<double> dis(lb_[0], ub_[0]);
    return StateDiscrete::Ptr( new StateDiscrete(dis(generator), shared_from_this()) );
  }
  virtual State::Ptr projectToFeasible(const State::Ptr& state) const {
    const StateDiscrete::Ptr& s = boost::static_pointer_cast<StateDiscrete>(state);
    double newvalue = std::max(std::min(s->value_,ub_[0]),lb_[0]);
    return StateDiscrete::Ptr( new StateDiscrete(newvalue, shared_from_this()) );
  }
  virtual State::Ptr fromVector(const std::vector<double>& vec) const {
    return State::Ptr( new StateDiscrete(vec[0], shared_from_this()) );
  }
  virtual std::vector<double> toVector(const State::Ptr& state) const {
    return std::vector<double>(1, boost::static_pointer_cast<StateDiscrete>(state)->value_);
  }
  virtual double distance(const State::Ptr& state1, const State::Ptr& state2) const {
    const StateDiscrete& s1 = *boost::static_pointer_cast<StateDiscrete>(state1);
    const StateDiscrete& s2 = *boost::static_pointer_cast<StateDiscrete>(state2);
    return std::abs(s1.value_ - s2.value_);
  }
};


class StateSpaceDiscreteCircular : public StateSpaceDiscrete
{
public:
  typedef boost::shared_ptr<StateSpaceDiscreteCircular> Ptr;
  typedef boost::shared_ptr<StateSpaceDiscreteCircular const> ConstPtr;
  StateSpaceDiscreteCircular(int lb, int ub) : StateSpaceDiscrete(lb, ub) {
  }
  StateSpaceDiscreteCircular(int n) : StateSpaceDiscrete(n) {
  }
  virtual std::vector<int> getModes() const {
    int lb = lb_[0];
    int ub = ub_[0];
    std::vector<int> modes(ub-lb+1);
    for (int i = lb; i <= ub; i++)
      modes[i-lb] = i;
    return modes;
  }
  virtual State::Ptr interpolate(const State::Ptr& state1, const State::Ptr& state2, double fraction) const {
    StateDiscreteCircular::Ptr state = boost::static_pointer_cast<StateDiscreteCircular>(getStateZero());
    const StateDiscreteCircular& s1 = *boost::static_pointer_cast<StateDiscreteCircular>(state1);
    const StateDiscreteCircular& s2 = *boost::static_pointer_cast<StateDiscreteCircular>(state2);
    const double mid = (lb_[0] + ub_[0]) / 2;
    const double ext =  ub_[0] - lb_[0];
    double& value = state->value_;
    double diff = s2.value_ - s1.value_;
    if (fabs(diff) <= ext/2) {
      value = s1.value_ + diff * fraction;
    } else {
      if (diff > mid)
        diff = ext - diff;
      else
        diff =-ext - diff;
      value = s1.value_ - diff * fraction;
      if (value > ub_[0])
        value -= ext;
      else if (value < lb_[0])
        value += ext;
    }
    return state;
  }
  virtual std::vector<State::Ptr> getNeighbors(const State::Ptr& state) const {
    const StateDiscreteCircular& s = *boost::static_pointer_cast<StateDiscreteCircular>(state);
    const double ext =  ub_[0] - lb_[0];
    std::vector<State::Ptr> nei;
    double v1 = s.value_-1;
    if (v1 < lb_[0]) v1 = (int)ub_[0];
    nei.push_back(State::Ptr( new StateDiscreteCircular(v1, shared_from_this()) ));
    double v2 = s.value_+1;
    if (v2 > ub_[0]) v2 = (int)lb_[0];
    nei.push_back(State::Ptr( new StateDiscreteCircular(v2, shared_from_this()) ));
    return nei;
  }
  virtual double distance(const State::Ptr& state1, const State::Ptr& state2) const {
    const StateDiscreteCircular& s1 = *boost::static_pointer_cast<StateDiscreteCircular>(state1);
    const StateDiscreteCircular& s2 = *boost::static_pointer_cast<StateDiscreteCircular>(state2);
    const double ext =  ub_[0] - lb_[0];
    double d = fabs(s1.value_ - s2.value_);
    return (d > ext/2) ? ext - d + 0.0001 : d;
  }
};


class StateSpaceSO2 : public StateSpace
{
public:
  typedef boost::shared_ptr<StateSpaceSO2> Ptr;
  typedef boost::shared_ptr<StateSpaceSO2 const> ConstPtr;
  StateSpaceSO2(int nbins) : StateSpace(std::vector<double>(1,-M_PI), std::vector<double>(1,M_PI), std::vector<int>(1,nbins)) {
  }
  virtual State::Ptr interpolate(const State::Ptr& state1, const State::Ptr& state2, double fraction) const {
    StateSO2::Ptr state = boost::static_pointer_cast<StateSO2>(getStateZero());
    const StateSO2& s1 = *boost::static_pointer_cast<StateSO2>(state1);
    const StateSO2& s2 = *boost::static_pointer_cast<StateSO2>(state2);
    double& value = state->value_;
    double diff = s2.value_ - s1.value_;
    if (fabs(diff) <= M_PI) {
      value = s1.value_ + diff * fraction;
    } else {
      if (diff > 0.0)
        diff = 2.0 * M_PI - diff;
      else
        diff =-2.0 * M_PI - diff;
      value = s1.value_ - diff * fraction;
      if (value > M_PI)
        value -= 2.0 * M_PI;
      else if (value < -M_PI)
        value += 2.0 * M_PI;
    }
    return state;
  }
  virtual std::vector<State::Ptr> getNeighbors(const State::Ptr& state) const {
    std::vector<State::Ptr> nei;
    std::vector<unsigned int> vec = toVectorInt(state);
    for (unsigned int i = 0; i < dim_; i++) {
      if (vec[i] > 0) {
        vec[i]--;
        nei.push_back(fromVectorInt(vec));
        vec[i]++;
      } else {
        vec[i] = nbins_[i]-1;
        nei.push_back(fromVectorInt(vec));
        vec[i] = 0;
      }
      if (vec[i] < nbins_[i]-1) {
        vec[i]++;
        //State::Ptr aa = fromVectorInt(vec);
        //std::vector<unsigned int> bb = toVectorInt(aa);
        //if (bb[i] != vec[i])
        //  std::cout << "WARNING: strange to<>from vectorInt\n";
        nei.push_back(fromVectorInt(vec));
        vec[i]--;
      } else {
        vec[i] = 0;
        nei.push_back(fromVectorInt(vec));
        vec[i] = nbins_[i]-1;
      }
    }
    return nei;
  }
  virtual State::Ptr getStateZero() const {
    return State::Ptr( new StateSO2(0, shared_from_this()) );
  }
  virtual State::Ptr sampleUniform(std::mt19937& generator) const {
    std::uniform_real_distribution<double> dis(lb_[0], ub_[0]);
    return State::Ptr( new StateSO2(dis(generator), shared_from_this()) );
  }
  virtual State::Ptr projectToFeasible(const State::Ptr& state) const {
    const StateSO2::Ptr& s = boost::static_pointer_cast<StateSO2>(state);
    double newvalue = std::max(std::min(s->value_,ub_[0]),lb_[0]);
    return StateSO2::Ptr( new StateSO2(newvalue, shared_from_this()) );
  }
  virtual State::Ptr fromVector(const std::vector<double>& vec) const {
    return State::Ptr( new StateSO2(vec[0], shared_from_this()) );
  }
  virtual State::Ptr fromVectorInt(const std::vector<unsigned int>& ivec) const {
    std::vector<double> vec(dim_);
    double binsize;
    for (unsigned int i = 0; i < dim_; i++) {
      binsize = (ub_[i] - lb_[i]) / nbins_[i];
      vec[i] = lb_[i] + ivec[i] * binsize + 1e-6; // we don't use the center of the bins (so we get 0 as a discrete state) but we need epsilon to avoid to<>from rounding issues
    }
    return fromVector(vec);
  }
  virtual std::vector<double> toVector(const State::Ptr& state) const {
    return std::vector<double>(1, boost::static_pointer_cast<StateSO2>(state)->value_);
  }
  virtual double distance(const State::Ptr& state1, const State::Ptr& state2) const {
    const StateSO2& s1 = *boost::static_pointer_cast<StateSO2>(state1);
    const StateSO2& s2 = *boost::static_pointer_cast<StateSO2>(state2);
    double d = fabs(s1.value_ - s2.value_);
    return (d > M_PI) ? 2.0 * M_PI - d : d;
  }
};


class StateSpaceSO3 : public StateSpace
{
public:
  typedef boost::shared_ptr<StateSpaceSO3> Ptr;
  typedef boost::shared_ptr<StateSpaceSO3 const> ConstPtr;
  StateSpaceSO3(int nbins) : StateSpace(std::vector<double>(4,0), std::vector<double>(4,1), std::vector<int>(4,nbins)) {
  }
  virtual State::Ptr interpolate(const State::Ptr& state1, const State::Ptr& state2, double fraction) const {
    const StateSO3& s1 = *boost::static_pointer_cast<StateSO3>(state1);
    const StateSO3& s2 = *boost::static_pointer_cast<StateSO3>(state2);
    return StateSO3::Ptr( new StateSO3(s1.quat_.slerp(fraction, s2.quat_), shared_from_this()) );
  }
  virtual State::Ptr getStateZero() const {
    return State::Ptr( new StateSO3(shared_from_this()) );
  }
  virtual State::Ptr sampleUniform(std::mt19937& generator) const {
    StateSO3::Ptr state( new StateSO3(shared_from_this()) );
    std::uniform_real_distribution<double> dis(0, 1);
    state->quat_.x() = dis(generator);
    state->quat_.y() = dis(generator);
    state->quat_.z() = dis(generator);
    state->quat_.w() = dis(generator);
    state->quat_.normalize();
    return state; 
  }
  virtual State::Ptr projectToFeasible(const State::Ptr& state) const {
    const StateSO3::Ptr& s = boost::static_pointer_cast<StateSO3>(state);
    StateSO3::Ptr proj( new StateSO3(shared_from_this()) );
    proj->quat_ = s->quat_.normalized();
    return proj;
  }
  virtual State::Ptr fromVector(const std::vector<double>& vec) const {
    return State::Ptr( new StateSO3(vec, shared_from_this()) );
  }
  virtual std::vector<double> toVector(const State::Ptr& state) const {
    const StateSO3::Ptr& s = boost::static_pointer_cast<StateSO3>(state);
    std::vector<double> vec(4);
    vec[0] = s->quat_.x();
    vec[1] = s->quat_.y();
    vec[2] = s->quat_.z();
    vec[3] = s->quat_.w();
    return vec;
  }
  virtual double distance(const State::Ptr& state1, const State::Ptr& state2) const {
    const StateSO3& s1 = *boost::static_pointer_cast<StateSO3>(state1);
    const StateSO3& s2 = *boost::static_pointer_cast<StateSO3>(state2);
    return s1.quat_.angularDistance(s2.quat_);
  }
};


//-----------------------------------------------------------------------------------------
// compound state spaces


template<typename StateType>
class StateSpaceCompound : public StateSpace
{
public:
  typedef boost::shared_ptr<StateSpaceCompound> Ptr;
  typedef boost::shared_ptr<StateSpaceCompound const> ConstPtr;
  StateSpaceCompound(const std::vector<StateSpace::Ptr>& spaces, const std::vector<double>& weights) : StateSpace(spaces, weights) {
    assert(spaces.size() == weights.size());
  }
  virtual unsigned int getMaximumExtent() const {
    double ext = 0;
    for (unsigned int i = 0; i < spaces_.size(); i++)
      ext += weights_[i] * spaces_[i]->getMaximumExtent();
    return ext;
  }
  virtual bool getFeasibility(const State::Ptr& state) const {
    const StateType& s = *boost::static_pointer_cast<StateType>(state);
    for (unsigned int i = 0; i < spaces_.size(); i++)
      if (!spaces_[i]->getFeasibility(s.states_[i]))
        return false;
    return true;
  }
  virtual State::Ptr interpolate(const State::Ptr& state1, const State::Ptr& state2, double fraction) const {
    boost::shared_ptr<StateType> state( new StateType(shared_from_this()) );
    const StateType& s1 = *boost::static_pointer_cast<StateType>(state1);
    const StateType& s2 = *boost::static_pointer_cast<StateType>(state2);
    state->weights_ = weights_;
    state->states_.resize(spaces_.size());
    for (unsigned int i = 0; i < spaces_.size(); i++)
      state->states_[i] = spaces_[i]->interpolate(s1.states_[i], s2.states_[i], fraction);
    return state;
  }
  virtual std::vector<State::Ptr> getNeighbors(const State::Ptr& state) const {
    const StateType& s = *boost::static_pointer_cast<StateType>(state);
    std::vector<State::Ptr> nei;
    for (unsigned int i = 0; i < spaces_.size(); i++) {
      std::vector<State::Ptr> nei_i = spaces_[i]->getNeighbors(s.states_[i]);
      for (unsigned int n = 0; n < nei_i.size(); n++) {
        boost::shared_ptr<StateType> newstate = boost::static_pointer_cast<StateType>(s.clone());
        newstate->states_[i] = nei_i[n];
        nei.push_back(newstate);
      }
    }
    return nei;
  }
  virtual State::Ptr getStateZero() const {
    boost::shared_ptr<StateType> zero( new StateType(shared_from_this()) );
    zero->weights_ = weights_;
    zero->states_.resize(spaces_.size());
    for (unsigned int i = 0; i < spaces_.size(); i++)
      zero->states_[i] = spaces_[i]->getStateZero();
    return zero;
  }
  virtual State::Ptr sampleUniform(std::mt19937& generator) const {
    boost::shared_ptr<StateType> state( new StateType(shared_from_this()) );
    for (unsigned int i = 0; i < spaces_.size(); i++)
      state->states_.push_back(spaces_[i]->sampleUniform(generator));
    return state;
  }
  virtual State::Ptr projectToFeasible(const State::Ptr& state) const {
    boost::shared_ptr<StateType> proj( new StateType(shared_from_this()) );
    *proj = *boost::static_pointer_cast<StateType>(state);
    for (unsigned int i = 0; i < spaces_.size(); i++)
      proj->states_[i] = spaces_[i]->projectToFeasible(proj->states_[i]);
    return proj;
  }
  virtual State::Ptr fromVector(const std::vector<double>& vec) const {
    boost::shared_ptr<StateType> state( new StateType(shared_from_this()) );
    state->weights_ = weights_;
    unsigned int d1 = 0, d2 = 0;
    for (unsigned int i = 0; i < spaces_.size(); i++) {
      d2 = d1 + spaces_[i]->getDimension();
      std::vector<double> v(vec.begin()+d1, vec.begin()+d2);
      state->states_.push_back(spaces_[i]->fromVector(v));
      d1 = d2;
    }
    return state;
  }
  virtual std::vector<double> toVector(const State::Ptr& state) const {
    const boost::shared_ptr<StateType>& s = boost::static_pointer_cast<StateType>(state);
    std::vector<double> vec;
    for (unsigned int i = 0; i < spaces_.size(); i++) {
      std::vector<double> v = spaces_[i]->toVector(s->states_[i]);
      vec.insert(vec.end(), v.begin(), v.end());
    }
    return vec;
  }
  virtual double distance(const State::Ptr& state1, const State::Ptr& state2) const {
    const StateType& s1 = *boost::static_pointer_cast<StateType>(state1);
    const StateType& s2 = *boost::static_pointer_cast<StateType>(state2);
    double distance = 0;
    for (unsigned int i = 0; i < spaces_.size(); i++)
      distance += weights_[i] * spaces_[i]->distance(s1.states_[i], s2.states_[i]);
    return distance;
  }
  virtual double distance(const State::Ptr& state1, const State::Ptr& state2, const std::vector<double>& weights) const {
    assert(weights.size() == weights_.size());
    const StateType& s1 = *boost::static_pointer_cast<StateType>(state1);
    const StateType& s2 = *boost::static_pointer_cast<StateType>(state2);
    double distance = 0;
    for (unsigned int i = 0; i < spaces_.size(); i++)
      distance += weights[i] * spaces_[i]->distance(s1.states_[i], s2.states_[i]);
    return distance;
  }
protected:
  //std::vector<StateSpace::Ptr> spaces_;
  //std::vector<double> weights_;
};


class StateSpaceSE3 : public StateSpaceCompound<StateSE3>
{
public:
  typedef boost::shared_ptr<StateSpaceSE3> Ptr;
  typedef boost::shared_ptr<StateSpaceSE3 const> ConstPtr;
  StateSpaceSE3(const std::vector<double>& XYZlb, const std::vector<double>& XYZub, const std::vector<int>& XYZnbins, int SO3nbins)
    : StateSpaceCompound(initS(XYZlb,XYZub,XYZnbins,SO3nbins), initW()) {
    assert(XYZlb.size() == 3 && XYZub.size() == 3 && XYZnbins.size() == 3);
  }
  static std::vector<StateSpace::Ptr> initS(const std::vector<double>& XYZlb, const std::vector<double>& XYZub, const std::vector<int>& XYZnbins, int SO3nbins) {
    std::vector<StateSpace::Ptr> spaces;
    spaces.push_back(StateSpaceReal::Ptr( new StateSpaceReal(XYZlb,XYZub,XYZnbins) ));
    spaces.push_back(StateSpaceSO3::Ptr( new StateSpaceSO3(SO3nbins) ));
    return spaces;
  }
  static std::vector<double> initW() {
    std::vector<double> w;
    w.push_back(1.0);
    w.push_back(1.0);
    return w;
  }
  virtual bool getTransform(const State::Ptr& state, flp::Transform& T) const {
    const StateSE3& s = *boost::static_pointer_cast<StateSE3>(state);
    const StateReal& xyz = *boost::static_pointer_cast<StateReal>(s.states_[0]);
    const StateSO3& rot = *boost::static_pointer_cast<StateSO3>(s.states_[1]);
    T.x = xyz.values_[0];
    T.y = xyz.values_[1];
    T.z = xyz.values_[2];
    T.qx = rot.quat_.x();
    T.qy = rot.quat_.y();
    T.qz = rot.quat_.z();
    T.qw = rot.quat_.w();
    return true;
  }
  virtual bool setTransform(State::Ptr& state, const flp::Transform& T) const {
    StateSE3& s = *boost::static_pointer_cast<StateSE3>(state);
    StateReal& xyz = *boost::static_pointer_cast<StateReal>(s.states_[0]);
    StateSO3& rot = *boost::static_pointer_cast<StateSO3>(s.states_[1]);
    xyz.values_[0] = T.x;
    xyz.values_[1] = T.y;
    xyz.values_[2] = T.z;
    rot.quat_.x() = T.qx;
    rot.quat_.y() = T.qy;
    rot.quat_.z() = T.qz;
    rot.quat_.w() = T.qw;
    return true;
  }
  virtual State::Ptr projectFrom(const State::Ptr& state, const StateSpace::ConstPtr& source) const {
    if (boost::dynamic_pointer_cast<StateSE3>(state))
      return state->clone();
    // get 6D pose
    flp::Transform T;
    if (!source->getTransform(state, T))
      return State::Ptr();
    // set values
    StateSE3::Ptr proj = boost::static_pointer_cast<StateSE3>(getStateZero());
    StateReal& xyz = *boost::static_pointer_cast<StateReal>(proj->states_[0]);
    StateSO3& rot = *boost::static_pointer_cast<StateSO3>(proj->states_[1]);
    xyz.values_[0] = T.x;
    xyz.values_[1] = T.y;
    xyz.values_[2] = T.z;
    rot.quat_.x() = T.qx;
    rot.quat_.y() = T.qy;
    rot.quat_.z() = T.qz;
    rot.quat_.w() = T.qw;
    // make sure it is within bounds
    return projectToFeasible(proj);
  }
};


class StateSpaceJointsSE3 : public StateSpaceCompound<StateJointsSE3>
{
public:
  typedef boost::shared_ptr<StateSpaceJointsSE3> Ptr;
  typedef boost::shared_ptr<StateSpaceJointsSE3 const> ConstPtr;
  StateSpaceJointsSE3(const std::vector<double>& Jlb, const std::vector<double>& Jub, const std::vector<int>& Jnbins,
    const std::vector<double>& XYZlb, const std::vector<double>& XYZub, const std::vector<int>& XYZnbins, int SO3nbins)
    : StateSpaceCompound(initS(Jlb,Jub,Jnbins,XYZlb,XYZub,XYZnbins,SO3nbins), initW()) {
    assert(XYZlb.size() == 3 && XYZub.size() == 3 && XYZnbins.size() == 3);
  }
  static std::vector<StateSpace::Ptr> initS(const std::vector<double>& Jlb, const std::vector<double>& Jub, const std::vector<int>& Jnbins,
    const std::vector<double>& XYZlb, const std::vector<double>& XYZub, const std::vector<int>& XYZnbins, int SO3nbins) {
    std::vector<StateSpace::Ptr> spaces;
    spaces.push_back(StateSpaceReal::Ptr( new StateSpaceReal(Jlb,Jub,Jnbins) ));
    spaces.push_back(StateSpaceReal::Ptr( new StateSpaceReal(XYZlb,XYZub,XYZnbins) ));
    spaces.push_back(StateSpaceSO3::Ptr( new StateSpaceSO3(SO3nbins) ));
    return spaces;
  }
  static std::vector<double> initW() {
    std::vector<double> w;
    w.push_back(1.0);
    w.push_back(1.0);
    w.push_back(1.0);
    return w;
  }
  virtual bool getTransform(const State::Ptr& state, flp::Transform& T) const {
    const StateJointsSE3& s = *boost::static_pointer_cast<StateJointsSE3>(state);
    const StateReal& xyz = *boost::static_pointer_cast<StateReal>(s.states_[1]);
    const StateSO3& rot = *boost::static_pointer_cast<StateSO3>(s.states_[2]);
    T.x = xyz.values_[0];
    T.y = xyz.values_[1];
    T.z = xyz.values_[2];
    T.qx = rot.quat_.x();
    T.qy = rot.quat_.y();
    T.qz = rot.quat_.z();
    T.qw = rot.quat_.w();
    return true;
  }
  virtual bool setTransform(State::Ptr& state, const flp::Transform& T) const {
    StateJointsSE3& s = *boost::static_pointer_cast<StateJointsSE3>(state);
    StateReal& xyz = *boost::static_pointer_cast<StateReal>(s.states_[1]);
    StateSO3& rot = *boost::static_pointer_cast<StateSO3>(s.states_[2]);
    xyz.values_[0] = T.x;
    xyz.values_[1] = T.y;
    xyz.values_[2] = T.z;
    rot.quat_.x() = T.qx;
    rot.quat_.y() = T.qy;
    rot.quat_.z() = T.qz;
    rot.quat_.w() = T.qw;
    return true;
  }
  virtual State::Ptr projectFrom(const State::Ptr& state, const StateSpace::ConstPtr& source) const {
    if (boost::dynamic_pointer_cast<StateJointsSE3>(state))
      return state->clone();
    // get 6D pose
    flp::Transform T;
    if (!source->getTransform(state, T))
      return State::Ptr();
    // set values
    StateJointsSE3::Ptr proj = boost::static_pointer_cast<StateJointsSE3>(getStateZero());
    StateReal& xyz = *boost::static_pointer_cast<StateReal>(proj->states_[1]);
    StateSO3& rot = *boost::static_pointer_cast<StateSO3>(proj->states_[2]);
    xyz.values_[0] = T.x;
    xyz.values_[1] = T.y;
    xyz.values_[2] = T.z;
    rot.quat_.x() = T.qx;
    rot.quat_.y() = T.qy;
    rot.quat_.z() = T.qz;
    rot.quat_.w() = T.qw;
    // make sure it is within bounds
    return projectToFeasible(proj);
  }
};


class StateSpaceXYT : public StateSpaceCompound<StateXYT>
{
public:
  typedef boost::shared_ptr<StateSpaceXYT> Ptr;
  typedef boost::shared_ptr<StateSpaceXYT const> ConstPtr;
  StateSpaceXYT(const std::vector<double>& XYlb, const std::vector<double>& XYub, const std::vector<int>& XYnbins, int SO2nbins)
    : StateSpaceCompound(initS(XYlb,XYub,XYnbins,SO2nbins), initW()) {
    assert(XYlb.size() == 2 && XYub.size() == 2 && XYnbins.size() == 2);
  }
  static std::vector<StateSpace::Ptr> initS(const std::vector<double>& XYlb, const std::vector<double>& XYub, const std::vector<int>& XYnbins, int SO2nbins) {
    std::vector<StateSpace::Ptr> spaces;
    spaces.push_back(StateSpaceReal::Ptr( new StateSpaceReal(XYlb,XYub,XYnbins) ));
    spaces.push_back(StateSpaceSO2::Ptr( new StateSpaceSO2(SO2nbins) ));
    return spaces;
  }
  static std::vector<double> initW() {
    std::vector<double> w;
    w.push_back(1.0);
    w.push_back(0.5);
    return w;
  }
  virtual bool getTransform(const State::Ptr& state, flp::Transform& T) const {
    const StateXYT& s = *boost::static_pointer_cast<StateXYT>(state);
    const StateReal& xyz = *boost::static_pointer_cast<StateReal>(s.states_[0]);
    const StateSO2& theta = *boost::static_pointer_cast<StateSO2>(s.states_[1]);
    T.x = xyz.values_[0];
    T.y = xyz.values_[1];
    T.z = 0.0;
    T.setRPY(0,0,theta.value_);
    return true;
  }
  virtual bool setTransform(State::Ptr& state, const flp::Transform& T) const {
    StateXYT& s = *boost::static_pointer_cast<StateXYT>(state);
    StateReal& xyz = *boost::static_pointer_cast<StateReal>(s.states_[0]);
    StateSO2& theta = *boost::static_pointer_cast<StateSO2>(s.states_[1]);
    xyz.values_[0] = T.x;
    xyz.values_[1] = T.y;
    double tmp1,tmp2;
    T.getRPY(tmp1,tmp2,theta.value_);
    return true;
  }
};


class StateSpaceXYZT : public StateSpaceCompound<StateXYZT>
{
public:
  typedef boost::shared_ptr<StateSpaceXYZT> Ptr;
  typedef boost::shared_ptr<StateSpaceXYZT const> ConstPtr;
  StateSpaceXYZT(const std::vector<double>& XYZlb, const std::vector<double>& XYZub, const std::vector<int>& XYZnbins, int SO2nbins)
    : StateSpaceCompound(initS(XYZlb,XYZub,XYZnbins,SO2nbins), initW()) {
    assert(XYZlb.size() == 3 && XYZub.size() == 3 && XYZnbins.size() == 3);
  }
  static std::vector<StateSpace::Ptr> initS(const std::vector<double>& XYZlb, const std::vector<double>& XYZub, const std::vector<int>& XYZnbins, int SO2nbins) {
    std::vector<StateSpace::Ptr> spaces;
    spaces.push_back(StateSpaceReal::Ptr( new StateSpaceReal(XYZlb,XYZub,XYZnbins) ));
    spaces.push_back(StateSpaceSO2::Ptr( new StateSpaceSO2(SO2nbins) ));
    return spaces;
  }
  static std::vector<double> initW() {
    std::vector<double> w;
    w.push_back(1.0);
    w.push_back(0.5);
    return w;
  }
  virtual bool getTransform(const State::Ptr& state, flp::Transform& T) const {
    const StateXYZT& s = *boost::static_pointer_cast<StateXYZT>(state);
    const StateReal& xyz = *boost::static_pointer_cast<StateReal>(s.states_[0]);
    const StateSO2& theta = *boost::static_pointer_cast<StateSO2>(s.states_[1]);
    T.x = xyz.values_[0];
    T.y = xyz.values_[1];
    T.z = xyz.values_[2];
    T.setRPY(0,0,theta.value_);
    return true;
  }
  virtual bool setTransform(State::Ptr& state, const flp::Transform& T) const {
    StateXYZT& s = *boost::static_pointer_cast<StateXYZT>(state);
    StateReal& xyz = *boost::static_pointer_cast<StateReal>(s.states_[0]);
    StateSO2& theta = *boost::static_pointer_cast<StateSO2>(s.states_[1]);
    xyz.values_[0] = T.x;
    xyz.values_[1] = T.y;
    xyz.values_[2] = T.z;
    double tmp1,tmp2;
    T.getRPY(tmp1,tmp2,theta.value_);
    return true;
  }
};

