#pragma once
#include "StateSpaceSwitch.h"
#include "StateSpaceContactPoint.h"

struct StateBaseSwitchContacts : public StateCompound
{
  typedef boost::shared_ptr<StateBaseSwitchContacts> Ptr;
  StateBaseSwitchContacts() {
  }
  StateBaseSwitchContacts(const StateSpaceConstPtr& space) : StateCompound(space) {
  }
  virtual State::Ptr clone() const {
    StateBaseSwitchContacts::Ptr state( new StateBaseSwitchContacts(space_) );
    state->weights_ = weights_;
    for (unsigned int i = 0; i < states_.size(); i++)
      state->states_.push_back(states_[i]->clone());
    return state;
  }
};


class StateSpaceBaseSwitchContacts : public StateSpaceCompound<StateBaseSwitchContacts>
{
public:
  typedef boost::shared_ptr<StateSpaceBaseSwitchContacts> Ptr;
  typedef boost::shared_ptr<StateSpaceBaseSwitchContacts const> ConstPtr;
  StateSpaceBaseSwitchContacts(const StateSpace::Ptr& base, const StateSpace::Ptr& swi) : StateSpaceCompound(initS(base,swi), initW()) {
  }
  static std::vector<StateSpace::Ptr> initS(const StateSpace::Ptr& base, const StateSpace::Ptr& swi) {
    std::vector<StateSpace::Ptr> spaces;
    spaces.push_back(base);
    spaces.push_back(swi);
    return spaces;
  }
  static std::vector<double> initW() {
    std::vector<double> weights;
    weights.push_back(1.0);
    weights.push_back(1.0);
    return weights;
  }
  virtual bool getTransform(const State::Ptr& state, flp::Transform& T) const {
    const StateBaseSwitchContacts& s = *boost::static_pointer_cast<StateBaseSwitchContacts>(state);
    return spaces_[0]->getTransform(s.states_[0], T);
  }
  virtual bool setTransform(State::Ptr& state, const flp::Transform& T) const {
    StateBaseSwitchContacts& s = *boost::static_pointer_cast<StateBaseSwitchContacts>(state);
    return spaces_[0]->setTransform(s.states_[0], T);
  }
  virtual bool getContacts(const State::Ptr& state, std::vector<Eigen::Vector3d>& contacts) const {
    // contact points w.r.t. base
    std::vector<double> vecContacts = spaces_[1]->toVector(boost::static_pointer_cast<StateBaseSwitchContacts>(state)->states_[1]);
    for (unsigned int i = 1; i < vecContacts.size()-2; i+=3)
      contacts.push_back(Eigen::Vector3d(vecContacts[i], vecContacts[i+1], vecContacts[i+2]));
    return true;
  }
  virtual bool isMotionInstantaneous() const {
    return true;
  }
  virtual State::Ptr interpolate(const State::Ptr& state1, const State::Ptr& state2, double fraction) const {
    StateBaseSwitchContacts::Ptr state3 = boost::static_pointer_cast<StateBaseSwitchContacts>(state1->clone());
    const StateBaseSwitchContacts& s1 = *boost::static_pointer_cast<StateBaseSwitchContacts>(state1);
    const StateBaseSwitchContacts& s2 = *boost::static_pointer_cast<StateBaseSwitchContacts>(state2);
    StateBaseSwitchContacts& s3 = *state3;
    // interpolate contacts
    s3.states_[1] = spaces_[1]->interpolate(s1.states_[1], s2.states_[1], fraction);
    // update base from contacts
    update(state3);
    return state3;
  }
  virtual std::vector<State::Ptr> getNeighbors(const State::Ptr& state) const {
    const StateBaseSwitchContacts& s = *boost::static_pointer_cast<StateBaseSwitchContacts>(state);
    // get neighbors for contacts
    std::vector<State::Ptr> contactNei = spaces_[1]->getNeighbors(s.states_[1]);
    // then compute full states
    //std::cout << "--------\n";
    //std::cout << toString(state) << "<--- from here to:\n";
    std::vector<State::Ptr> nei;
    for (unsigned int i = 0; i < contactNei.size(); i++) {
      StateBaseSwitchContacts::Ptr newstate( new StateBaseSwitchContacts(shared_from_this()) );
      newstate->states_.push_back(s.states_[0]->clone());
      newstate->states_.push_back(contactNei[i]);
      // update base from contacts
      update(newstate);
      //std::cout << toString(newstate) << "\n";
      nei.push_back(newstate);
    }
    return nei;
  }
  void update(const StateBaseSwitchContacts::Ptr& state) const {
    // get base
    flp::Transform T;
    getTransform(state, T);
    Eigen::Vector3d oldBase(T.x,T.y,T.z);
    Eigen::Quaterniond q(T.qx, T.qy, T.qz, T.qw);
    Eigen::Transform<double,3,Eigen::Affine> eiT = Eigen::Translation<double,3>(oldBase) * q;
    // get contacts in world ref
    std::vector<double> vecContacts = spaces_[1]->toVector(state->states_[1]);
    std::vector<Eigen::Vector3d> contacts;
    for (unsigned int i = 1; i < vecContacts.size()-2; i+=3)
      contacts.push_back(eiT * Eigen::Vector3d(vecContacts[i], vecContacts[i+1], vecContacts[i+2]));
    // update base from contacts
    Eigen::Vector3d newBase(0,0,0);
    for (unsigned int i = 0; i < contacts.size(); i++)
      newBase += contacts[i];
    newBase /= (double)contacts.size();
    newBase(2) += 0.50; //TODO: parameter? or compute from middle of the range of contact Z
    std::vector<double> vecBase = spaces_[0]->toVector(state->states_[0]);
    std::vector<double> newvecBase = vecBase;
    newvecBase[0] = newBase(0);
    newvecBase[1] = newBase(1);
    newvecBase[2] = newBase(2); //TODO: can we do this? base might be 2D .... need something more general
    state->states_[0] = spaces_[0]->fromVector(newvecBase);

    //std::cout << "------\n";
    //std::cout << "old base = " << oldBase.transpose() << "\n";
    //std::cout << "new base = " << newBase.transpose() << "\n";
    //for (unsigned int i = 0; i < contacts.size(); i++)
    //  std::cout << contacts[i].transpose() << "\n";

    // update contacts from base
    eiT = Eigen::Translation<double,3>(newBase) * q;
    Eigen::Transform<double,3,Eigen::Affine> eiTinv = eiT.inverse();
    std::vector<double> newvecContacts;
    newvecContacts.push_back(vecContacts[0]);
    for (unsigned int i = 0; i < contacts.size(); i++) {
      Eigen::Vector3d relative = eiTinv * contacts[i];
      newvecContacts.push_back(relative(0));
      newvecContacts.push_back(relative(1));
      newvecContacts.push_back(relative(2));
    }
    state->states_[1] = spaces_[1]->fromVector(newvecContacts);
  }
};

