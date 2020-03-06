#pragma once
#include "flp.h"
#include "State.h"
#include "StateSpace.h"
#include "Environment.h"
#include <vector>
#include <string>

class RobotModel
{
public:
  typedef boost::shared_ptr<RobotModel> Ptr;
  RobotModel(const StateSpace::Ptr& space) : stateSpace_(space), contactProjectionOffset_(0,0,0) {
  }
  virtual StateSpace::Ptr getStateSpace() const {
    return stateSpace_;
  }
  virtual void setStateSpace(const StateSpace::Ptr& space) {
    stateSpace_ = space;
  }
  virtual std::vector<flp::Primitive> getPrimitives(const State::Ptr& state) {
    std::vector<flp::Primitive> pri = getPrimitivesCollision(state);
    std::vector<flp::Primitive> tmp = getPrimitivesContact(state);
    pri.insert(pri.end(), tmp.begin(), tmp.end());
    return pri;
  }
  virtual void setContactProjectionOffset(const Eigen::Vector3d& contactProjectionOffset) {
    contactProjectionOffset_ = contactProjectionOffset;
  }

  virtual std::vector<flp::Primitive> getPrimitivesCollision(const State::Ptr& state) = 0;
  virtual std::vector<flp::Primitive> getPrimitivesContact(const State::Ptr& state) = 0;
  virtual State::Ptr project(const State::Ptr& state, const Environment::Ptr& environment) = 0;
  virtual Eigen::Vector3d sampleUniformCOP2COM(std::mt19937& generator) = 0;

protected:
  StateSpace::Ptr stateSpace_;
  Eigen::Vector3d contactProjectionOffset_;
};

