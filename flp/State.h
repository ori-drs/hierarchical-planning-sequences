#pragma once
#include "flp.h"
#include "Action.h"
#include <boost/shared_ptr.hpp>
#include <string>

class StateSpace;
typedef boost::shared_ptr<StateSpace const> StateSpaceConstPtr;


struct State
{
  typedef boost::shared_ptr<State> Ptr;
  State() {}
  State(const StateSpaceConstPtr& space) : space_(space) {}
  virtual State::Ptr clone() const = 0;
  StateSpaceConstPtr space_;
};


struct StateReal : public State
{
  typedef boost::shared_ptr<StateReal> Ptr;
  StateReal(const std::vector<double>& values) : values_(values) {
  }
  StateReal(const std::vector<double>& values, const StateSpaceConstPtr& space) : State(space), values_(values) {
  }
  virtual State::Ptr clone() const {
    return State::Ptr( new StateReal(values_, space_) );
  }
  std::vector<double> values_;
};


struct StateDiscrete : public State
{
  typedef boost::shared_ptr<StateDiscrete> Ptr;
  StateDiscrete(double value) : value_(value) {
  }
  StateDiscrete(double value, const StateSpaceConstPtr& space) : State(space), value_(value) {
  }
  virtual State::Ptr clone() const {
    return State::Ptr( new StateDiscrete(value_, space_) );
  }
  double value_;
};


struct StateDiscreteCircular : public State
{
  typedef boost::shared_ptr<StateDiscreteCircular> Ptr;
  StateDiscreteCircular(double value) : value_(value) {
  }
  StateDiscreteCircular(double value, const StateSpaceConstPtr& space) : State(space), value_(value) {
  }
  virtual State::Ptr clone() const {
    return State::Ptr( new StateDiscreteCircular(value_, space_) );
  }
  double value_;
};


struct StateSO2 : public State
{
  typedef boost::shared_ptr<StateSO2> Ptr;
  StateSO2(double value) : value_(value) {
  }
  StateSO2(double value, const StateSpaceConstPtr& space) : State(space), value_(value) {
  }
  virtual State::Ptr clone() const {
    return State::Ptr( new StateSO2(value_, space_) );
  }
  double value_;
};


struct StateSO3 : public State
{
  typedef boost::shared_ptr<StateSO3> Ptr;
  StateSO3() : quat_(Eigen::Quaterniond::Identity()) {
  }
  StateSO3(const StateSpaceConstPtr& space) : State(space), quat_(Eigen::Quaterniond::Identity()) {
  }
  StateSO3(const Eigen::Quaterniond& quat) : quat_(quat) {
  }
  StateSO3(const Eigen::Quaterniond& quat, const StateSpaceConstPtr& space) : State(space), quat_(quat) {
  }
  StateSO3(const std::vector<double>& values) {
    quat_.x() = values[0];
    quat_.y() = values[1];
    quat_.z() = values[2];
    quat_.w() = values[3];
  }
  StateSO3(const std::vector<double>& values, const StateSpaceConstPtr& space) : State(space) {
    quat_.x() = values[0];
    quat_.y() = values[1];
    quat_.z() = values[2];
    quat_.w() = values[3];
  }
  virtual State::Ptr clone() const {
    return State::Ptr( new StateSO3(quat_, space_) );
  }
  Eigen::Quaterniond quat_;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


//-----------------------------------------------------------------------------------------
// compound states


struct StateCompound : public State
{
  typedef boost::shared_ptr<StateCompound> Ptr;
  StateCompound() {
  }
  StateCompound(const StateSpaceConstPtr& space) : State(space) {
  }
  virtual State::Ptr clone() const {
    StateCompound::Ptr state( new StateCompound(space_) );
    state->weights_ = weights_;
    for (unsigned int i = 0; i < states_.size(); i++)
      state->states_.push_back(states_[i]->clone());
    return state;
  }
  std::vector<State::Ptr> states_;
  std::vector<double> weights_;
};


struct StateSE3 : public StateCompound
{
  typedef boost::shared_ptr<StateSE3> Ptr;
  StateSE3() {
  }
  StateSE3(const StateSpaceConstPtr& space) : StateCompound(space) {
  }
  StateSE3(const std::vector<double>& xyz, const Eigen::Quaterniond& quat, const StateSpaceConstPtr& space) : StateCompound(space) {
    states_.push_back(State::Ptr( new StateReal(xyz) ));
    states_.push_back(State::Ptr( new StateSO3(quat) ));
    weights_.push_back(1.0);
    weights_.push_back(1.0);
  }
  virtual State::Ptr clone() const {
    StateSE3::Ptr state( new StateSE3(space_) );
    state->weights_ = weights_;
    for (unsigned int i = 0; i < states_.size(); i++)
      state->states_.push_back(states_[i]->clone());
    return state;
  }
  flp::Transform getTransform() const {
    const StateReal& xyz = *boost::static_pointer_cast<StateReal>(states_[0]);
    const StateSO3& rot = *boost::static_pointer_cast<StateSO3>(states_[1]);
    flp::Transform T;
    T.x = xyz.values_[0];
    T.y = xyz.values_[1];
    T.z = xyz.values_[2];
    T.qx = rot.quat_.x();
    T.qy = rot.quat_.y();
    T.qz = rot.quat_.z();
    T.qw = rot.quat_.w();
    return T;
  }
};


struct StateJointsSE3 : public StateCompound
{
  typedef boost::shared_ptr<StateJointsSE3> Ptr;
  StateJointsSE3() {
  }
  StateJointsSE3(const StateSpaceConstPtr& space) : StateCompound(space) {
  }
  virtual State::Ptr clone() const {
    StateJointsSE3::Ptr state( new StateJointsSE3(space_) );
    state->weights_ = weights_;
    for (unsigned int i = 0; i < states_.size(); i++)
      state->states_.push_back(states_[i]->clone());
    return state;
  }
  flp::Transform getTransform() const {
    const StateReal& xyz = *boost::static_pointer_cast<StateReal>(states_[1]);
    const StateSO3& rot = *boost::static_pointer_cast<StateSO3>(states_[2]);
    flp::Transform T;
    T.x = xyz.values_[0];
    T.y = xyz.values_[1];
    T.z = xyz.values_[2];
    T.qx = rot.quat_.x();
    T.qy = rot.quat_.y();
    T.qz = rot.quat_.z();
    T.qw = rot.quat_.w();
    return T;
  }
};


struct StateXYT : public StateCompound
{
  typedef boost::shared_ptr<StateXYT> Ptr;
  StateXYT() {
  }
  StateXYT(const StateSpaceConstPtr& space) : StateCompound(space) {
  }
  virtual State::Ptr clone() const {
    StateXYT::Ptr state( new StateXYT(space_) );
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
    T.z = 0.0;
    T.setRPY(0,0,theta.value_);
    return T;
  }
};


struct StateXYZT : public StateCompound
{
  typedef boost::shared_ptr<StateXYZT> Ptr;
  StateXYZT() {
  }
  StateXYZT(const StateSpaceConstPtr& space) : StateCompound(space) {
  }
  virtual State::Ptr clone() const {
    StateXYZT::Ptr state( new StateXYZT(space_) );
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

