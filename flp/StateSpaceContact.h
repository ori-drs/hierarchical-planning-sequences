#pragma once
#include "State.h"
#include "StateSpace.h"

// volume (a box) of feasible 3D positions (relative to base). implemented as (lbBaseRel,ubBaseRel,nbinsBaseRel)
// - states have contact points in world coordinates, and a transform from world to base
// - if we want to use a set of primitives then add feasibility checker which checks that separately
// - need a special motion checker for ompl that either 1) makes sure only initial and final positions are checked (no interpolation is made)
//   or 2) uses func(start,goal,environment) to decide whether the motion is feasible (advanced)

struct StateContact : public State
{
  typedef boost::shared_ptr<StateContact> Ptr;
  StateContact(const std::vector<double>& values, const flp::Transform& base) : values_(values), base_(base) {
  }
  StateContact(const std::vector<double>& values, const flp::Transform& base, const StateSpaceConstPtr& space) : State(space), values_(values), base_(base) {
  }
  StateContact(const std::vector<double>& values) : values_(values) {
  }
  StateContact(const std::vector<double>& values, const StateSpaceConstPtr& space) : State(space), values_(values) {
  }
  virtual State::Ptr clone() const {
    return State::Ptr( new StateContact(values_, base_, space_) );
  }
  std::vector<double> values_;
  flp::Transform base_;
};

class StateSpaceContact : public StateSpace
{
public:
  typedef boost::shared_ptr<StateSpaceContact> Ptr;
  typedef boost::shared_ptr<StateSpaceContact const> ConstPtr;
  StateSpaceContact(const std::vector<double>& lb, const std::vector<double>& ub, const std::vector<int>& nbins,
    const std::vector<double>& lbBaseRel, const std::vector<double>& ubBaseRel, const std::vector<int>& nbinsBaseRel)
    : StateSpace(lb,ub,nbins), lbBaseRel_(lbBaseRel), ubBaseRel_(ubBaseRel), nbinsBaseRel_(nbinsBaseRel)
  {
    assert(lb.size() == 3);
    zero_.resize(dim_);
    for (unsigned int i = 0; i < dim_; i++)
      zero_[i] = (lb_[i] + ub_[i]) / 2;
  }
  virtual bool getTransform(const State::Ptr& state, flp::Transform& T) const {
    T = boost::static_pointer_cast<StateContact>(state)->base_;
    return true;
  }
  virtual bool setTransform(State::Ptr& state, const flp::Transform& T) const {
    // compute transformation from prev to next reference
    flp::Transform Tprev = boost::static_pointer_cast<StateContact>(state)->base_;
    Eigen::Transform<double,3,Eigen::Affine> eiTprev = Tprev.getEigenTransform();
    Eigen::Transform<double,3,Eigen::Affine> eiTnext = T.getEigenTransform();
    // apply it to contact
    std::vector<double>& vec = boost::static_pointer_cast<StateContact>(state)->values_;
    Eigen::Vector3d cprev(vec[0], vec[1], vec[2]);
    Eigen::Vector3d cnext = eiTnext * eiTprev.inverse() * cprev;
    vec[0] = cnext(0);
    vec[1] = cnext(1);
    vec[2] = cnext(2);
    // update value
    boost::static_pointer_cast<StateContact>(state)->base_ = T;
    return true;
  }
  virtual bool getContacts(const State::Ptr& state, std::vector<Eigen::Vector3d>& contacts) const {
    const std::vector<double>& vec = boost::static_pointer_cast<StateContact>(state)->values_;
    contacts.push_back(Eigen::Vector3d(vec[0], vec[1], vec[2]));
    return true;
  }
  virtual bool setContacts(State::Ptr& state, const std::vector<Eigen::Vector3d>& contacts) const {
    assert(contacts.size() == 1);
    std::vector<double>& vec = boost::static_pointer_cast<StateContact>(state)->values_;
    const Eigen::Vector3d& contact = contacts[0];
    vec[0] = contact(0);
    vec[1] = contact(1);
    vec[2] = contact(2);
    return true;
  }
  virtual bool getFeasibility(const State::Ptr& state) const {
    if (!StateSpace::getFeasibility(state)) {
      //std::cout << "outside lb/ub\n";
      return false;
    }
    const StateContact& s = *boost::static_pointer_cast<StateContact>(state);
    // get base to world transform
    const flp::Transform& T = s.base_;
    Eigen::Vector3d xyz(T.x,T.y,T.z);
    Eigen::Quaterniond q(T.qw, T.qx, T.qy, T.qz);
    Eigen::Transform<double,3,Eigen::Affine> eiT = Eigen::Translation<double,3>(xyz) * q;
    // get contact point seen from base
    Eigen::Vector3d ptBaseRel = eiT.inverse() * Eigen::Vector3d(s.values_[0], s.values_[1], s.values_[2]);
    // check feasibility
    for (unsigned int i = 0; i < 3; i++)
      if (ptBaseRel(i) < lbBaseRel_[i] || ptBaseRel(i) > ubBaseRel_[i]) {
        //std::cout << "outside BASE lb/ub " << lbBaseRel_[i] << " < "<< ptBaseRel(i) << " < " << ubBaseRel_[i] << "\n";
        return false;
      }
    return true;
  }
  virtual State::Ptr interpolate(const State::Ptr& state1, const State::Ptr& state2, double fraction) const {
    State::Ptr state3 = state1->clone();
    const StateContact& s1 = *boost::static_pointer_cast<StateContact>(state1);
    const StateContact& s2 = *boost::static_pointer_cast<StateContact>(state2);
    StateContact& s3 = *boost::static_pointer_cast<StateContact>(state3);
    std::vector<double> vec1(3), vec2(3);
    vec1[0] = s1.values_[0] - s1.base_.x;
    vec1[1] = s1.values_[1] - s1.base_.y;
    vec1[2] = s1.values_[2] - s1.base_.z;
    vec2[0] = s2.values_[0] - s2.base_.x;
    vec2[1] = s2.values_[1] - s2.base_.y;
    vec2[2] = s2.values_[2] - s2.base_.z;
    s3.values_[0] += fraction * (vec2[0] - vec1[0]);
    s3.values_[1] += fraction * (vec2[1] - vec1[1]);
    s3.values_[2] += fraction * (vec2[2] - vec1[2]);
    return state3;
  }
  virtual std::vector<State::Ptr> getNeighbors(const State::Ptr& state) const {
    const StateContact& s = *boost::static_pointer_cast<StateContact>(state);
    std::vector<unsigned int> ivec = toVectorInt(state);
    std::vector<State::Ptr> nei;
    nei.reserve(getNumberOfCells());
    // aux
    double binsizeBaseRel[3];
    double binsize[3];
    for (unsigned int i = 0; i < 3; i++) {
      binsizeBaseRel[i] = (ubBaseRel_[i] - lbBaseRel_[i]) / nbinsBaseRel_[i];
      binsize[i] = (ub_[i] - lb_[i]) / nbins_[i];
    }
    // get world to base transform
    const flp::Transform& T = s.base_;
    Eigen::Vector3d xyz(T.x,T.y,T.z);
    Eigen::Quaterniond q(T.qw, T.qx, T.qy, T.qz);
    Eigen::Transform<double,3,Eigen::Affine> eiT = Eigen::Translation<double,3>(xyz) * q;
    // get all possible points given the base transform
    for (unsigned int bx = 0; bx < nbinsBaseRel_[0]; bx++) {
      for (unsigned int by = 0; by < nbinsBaseRel_[1]; by++) {
        for (unsigned int bz = 0; bz < nbinsBaseRel_[2]; bz++) {
          // position w.r.t. base
          Eigen::Vector3d bp(
            lbBaseRel_[0] + bx * binsizeBaseRel[0] + binsizeBaseRel[0]/2,
            lbBaseRel_[1] + by * binsizeBaseRel[1] + binsizeBaseRel[1]/2,
            lbBaseRel_[2] + bz * binsizeBaseRel[2] + binsizeBaseRel[2]/2);
          // position w.r.t. world
          Eigen::Vector3d wp = eiT * bp;
          // get closest world grid cell
          bool ok = true;
          std::vector<double> pt(3);
          std::vector<int> ipt(3);
          for (unsigned int i = 0; i < 3 && ok; i++) {
            int tmp = (wp(i) - lb_[i]) / ((ub_[i]-lb_[i])/nbins_[i]);
            if (tmp < 0 || tmp >= nbins_[i])
              ok = false;
            pt[i] = lb_[i] + tmp * binsize[i] + binsize[i]/2;
            ipt[i] = tmp;
          }
          // skip pt==state
          if (ipt[0] == ivec[0] && ipt[1] == ivec[1] && ipt[2] == ivec[2])
            continue;
          // add
          if (ok) {
            nei.push_back(State::Ptr( new StateContact(pt, T, shared_from_this()) ));
          }
        }
      }
    }
    return nei;
  }
  virtual State::Ptr getStateZero() const {
    return State::Ptr( new StateContact(zero_, shared_from_this()) );
  }
  virtual State::Ptr sampleUniform(std::mt19937& generator) const {
    StateContact::Ptr state( new StateContact(std::vector<double>(dim_,0.0), shared_from_this()) );
    for (unsigned int i = 0; i < dim_; i++) {
      std::uniform_real_distribution<double> dis(lb_[i], ub_[i]);
      state->values_[i] = dis(generator);
    }
    return state;
  }
  virtual State::Ptr projectToFeasible(const State::Ptr& state) const {
    const StateContact::Ptr& s = boost::static_pointer_cast<StateContact>(state);
    std::vector<double> newvalues(dim_);
    for (unsigned int i = 0; i < dim_; i++) {
      newvalues[i] = std::max(std::min(s->values_[i],ub_[i]),lb_[i]);
    }
    return StateContact::Ptr( new StateContact(newvalues, shared_from_this()) );
  }
  virtual State::Ptr fromVector(const std::vector<double>& vec) const {
    return State::Ptr( new StateContact(vec, shared_from_this()) );
  }
  virtual std::vector<double> toVector(const State::Ptr& state) const {
    return boost::static_pointer_cast<StateContact>(state)->values_;
  }
  virtual double distance(const State::Ptr& state1, const State::Ptr& state2) const {
    double distance = 0;
    const StateContact& s1 = *boost::static_pointer_cast<StateContact>(state1);
    const StateContact& s2 = *boost::static_pointer_cast<StateContact>(state2);
    for (unsigned int i = 0; i < dim_; i++)
      distance += pow(s1.values_[i] - s2.values_[i], 2.0);
    return sqrt(distance);
  }
  virtual State::Ptr projectFrom(const State::Ptr& state, const StateSpace::ConstPtr& source) const {
    if (boost::dynamic_pointer_cast<StateContact>(state))
      return state->clone();
    // get base
    flp::Transform T;
    if (!source->getTransform(state, T))
      return State::Ptr();
    // position w.r.t. base (assume it is in the middle of the bounds)
    Eigen::Transform<double,3,Eigen::Affine> eiT = T.getEigenTransform();
    Eigen::Vector3d bp(
      (lbBaseRel_[0] + ubBaseRel_[0])/2,
      (lbBaseRel_[1] + ubBaseRel_[1])/2,
      (lbBaseRel_[2] + ubBaseRel_[2])/2
    );
    // position w.r.t. world
    Eigen::Vector3d wp = eiT * bp;
    std::vector<double> xyz(3);
    xyz[0] = wp(0);
    xyz[1] = wp(1);
    xyz[2] = wp(2);
    // project
    return State::Ptr( new StateContact(xyz, T, shared_from_this()) );
  }
  std::vector<double> lbBaseRel_;
  std::vector<double> ubBaseRel_;
  std::vector<int> nbinsBaseRel_;
protected:
  std::vector<double> zero_;
};

// contacts is just compound of contact. custom gettransform. when state is constructed then base is filled in using gettransform
// switchcontacts is switch of contact. custom gettransform. when state is constructed then base is filled in using gettransform
