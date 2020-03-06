#pragma once
#include "StateSpaceSwitch.h"
#include "StateSpaceContact.h"

class StateSpaceSwitchContacts : public StateSpaceSwitch
{
public:
  typedef boost::shared_ptr<StateSpaceSwitchContacts> Ptr;
  typedef boost::shared_ptr<StateSpaceSwitchContacts const> ConstPtr;
  StateSpaceSwitchContacts(const std::vector<StateSpace::Ptr>& spaces, const std::vector<double>& weights) : StateSpaceSwitch(spaces, weights) {
  }
  virtual State::Ptr sampleUniform(std::mt19937& generator) const {
    State::Ptr state = getStateZero();
    StateSwitch& s = *boost::static_pointer_cast<StateSwitch>(state);
    // uniform base
    std::vector<double> lbBase(3), ubBase(3), base(3);
    std::vector<int> nbinsBase;
    spaces_[1]->getBounds(lbBase, ubBase, nbinsBase);
    for (unsigned int i = 0; i < 3; i++) {
      std::uniform_real_distribution<double> dis(lbBase[i], ubBase[i]);
      base[i] = dis(generator);
    }
    std::uniform_real_distribution<double> disTheta(-M_PI, M_PI);
    double theta  = disTheta(generator);
    flp::Transform T;
    T.x = base[0];
    T.y = base[1];
    T.z = base[2];
    T.setRPY(0,0,theta);
    // uniform contacts
    std::vector<double> contact(3);
    for (unsigned int d = 1; d < spaces_.size(); d++) {
      const StateSpaceContact& ss = *boost::static_pointer_cast<StateSpaceContact>(spaces_[d]);
      for (unsigned int i = 0; i < 3; i++) {
        std::uniform_real_distribution<double> dis(ss.lbBaseRel_[i], ss.ubBaseRel_[i]);
        contact[i] = base[i] + dis(generator);
      }
      s.states_[d] = State::Ptr( new StateContact(contact, T, s.states_[d]->space_) );
    }
    return state;
  }
  virtual bool getTransform(const State::Ptr& state, flp::Transform& T) const {
    std::vector<Eigen::Vector3d> contacts;
    if (!getContacts(state, contacts))
      return false;
    // get center of contact
    Eigen::Vector3d coc(0,0,0);
    for (unsigned int i = 0; i < contacts.size(); i++)
      coc += contacts[i];
    coc /= (double)contacts.size();
    // add mid height (middle of feasible space)
    const StateSpaceContact& ssc = *boost::static_pointer_cast<StateSpaceContact>(spaces_[1]);
    double height = - (ssc.lbBaseRel_[2] + ssc.ubBaseRel_[2]) / 2;
    T.x = coc(0);
    T.y = coc(1);
    T.z = coc(2) + height;
    // rotation
    if (contacts.size() == 4) {
      Eigen::Vector3d front = (contacts[0] + contacts[2]) / 2;
      Eigen::Vector3d back  = (contacts[1] + contacts[3]) / 2;
      double theta = atan2(front(1)-back(1), front(0)-back(0));
      T.setRPY(0,0,theta);
    }
    return true;
  }
  virtual bool setTransform(State::Ptr& state, const flp::Transform& T) const {
    StateSwitch& s = *boost::static_pointer_cast<StateSwitch>(state);
    bool ok = true;
    for (unsigned int i = 1; i < spaces_.size() && ok; i++) {
      ok = ok && spaces_[i]->setTransform(s.states_[i], T);
    }
    return ok;
  }
  virtual bool getContacts(const State::Ptr& state, std::vector<Eigen::Vector3d>& contacts) const {
    const StateSwitch& s = *boost::static_pointer_cast<StateSwitch>(state);
    bool ok = true;
    for (unsigned int i = 1; i < spaces_.size() && ok; i++)
      ok = ok && spaces_[i]->getContacts(s.states_[i], contacts);
    return ok;
  }
  virtual bool setContacts(State::Ptr& state, const std::vector<Eigen::Vector3d>& contacts) const {
    assert(contacts.size() == spaces_.size() - 1);
    StateSwitch& s = *boost::static_pointer_cast<StateSwitch>(state);
    bool ok = true;
    std::vector<Eigen::Vector3d> tmp(1);
    for (unsigned int i = 1; i < spaces_.size() && ok; i++) {
      tmp[0] = contacts[i-1];
      ok = ok && spaces_[i]->setContacts(s.states_[i], tmp);
    }
    // update base
    flp::Transform T;
    ok = ok && getTransform(state, T);
    for (unsigned int i = 1; i < spaces_.size() && ok; i++)
      boost::static_pointer_cast<StateContact>(s.states_[i])->base_ = T;
    return ok;
  }
  virtual bool getMode(const State::Ptr& state, int& mode) const {
    mode = 2; // "step planning" mode=2
    return true;
  }
  virtual bool getMode(const State::Ptr& state, Eigen::VectorXd& mode) const {
    if (mode.size() != 1) mode.resize(1);
    mode(0) = 2; // "step planning" mode=2
    return true;
  }
  virtual std::vector<int> getModes() const {
    return std::vector<int>(1,2); // "step planning" mode=2
  }
  virtual bool isMotionInstantaneous() const {
    return true;
  }
  virtual State::Ptr interpolate(const State::Ptr& state1, const State::Ptr& state2, double fraction) const {
    State::Ptr state3 = StateSpaceSwitch::interpolate(state1, state2, fraction);
    StateSwitch& s3 = *boost::static_pointer_cast<StateSwitch>(state3);
    // get base
    flp::Transform T;
    if (!getTransform(state3, T))
      return State::Ptr();
    // update base
    for (unsigned int i = 1; i < s3.states_.size(); i++)
      boost::static_pointer_cast<StateContact>(s3.states_[i])->base_ = T;
    return state3;
  }
  virtual std::vector<State::Ptr> getNeighbors(const State::Ptr& state) const {
    std::vector<State::Ptr> nei = StateSpaceSwitch::getNeighbors(state);
    for (unsigned int n = 0; n < nei.size(); n++) {
      StateSwitch& s = *boost::static_pointer_cast<StateSwitch>(nei[n]);
      // get base
      flp::Transform T;
      if (!getTransform(nei[n], T))
        continue;
      // update base
      for (unsigned int i = 1; i < s.states_.size(); i++)
        boost::static_pointer_cast<StateContact>(s.states_[i])->base_ = T;
    }
    return nei;
  }
  virtual State::Ptr projectFrom(const State::Ptr& state, const StateSpace::ConstPtr& source) const {
    if (boost::dynamic_pointer_cast<StateSwitch>(state))
      return state->clone();
    // get contacts
    StateSwitch::Ptr proj = boost::static_pointer_cast<StateSwitch>(getStateZero());
    for (unsigned int i = 1; i < spaces_.size(); i++) {
      proj->states_[i] = spaces_[i]->projectFrom(state, source);
      //std::cout << spaces_[i]->toString(proj->states_[i]) << "\n";
    }
    //std::cout << toString(proj) << "\n";
    // get base
    flp::Transform T;
    if (!getTransform(proj, T))
      return State::Ptr();
    //
    //flp::Transform Torig;
    //source->getTransform(state, Torig);
    //printf("Torig = %f %f %f \n", Torig.x, Torig.y, Torig.z);
    //printf("Tproj = %f %f %f \n", T.x, T.y, T.z);
    //printf("----\n");
    //
    // update base
    for (unsigned int i = 1; i < spaces_.size(); i++)
      boost::static_pointer_cast<StateContact>(proj->states_[i])->base_ = T;
    return proj;
  }
  virtual State::Ptr fromVector(const std::vector<double>& vec) const {
    State::Ptr state = StateSpaceSwitch::fromVector(vec);
    StateSwitch& s = *boost::static_pointer_cast<StateSwitch>(state);
    // get base
    flp::Transform T;
    if (!getTransform(state, T))
      return State::Ptr();
    // update base
    for (unsigned int i = 1; i < s.states_.size(); i++)
      boost::static_pointer_cast<StateContact>(s.states_[i])->base_ = T;
    return state;
  }
};

