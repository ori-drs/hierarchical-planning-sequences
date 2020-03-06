#pragma once
#include "RobotModel.h"

class RobotModelFixedPrimitives : public RobotModel
{
public:
  typedef boost::shared_ptr<RobotModelFixedPrimitives> Ptr;
  RobotModelFixedPrimitives(const StateSpace::Ptr& space, const std::vector<flp::Primitive>& collision, const std::vector<flp::Primitive>& contact);

  virtual std::vector<flp::Primitive> getPrimitivesCollision(const State::Ptr& state);
  virtual std::vector<flp::Primitive> getPrimitivesContact(const State::Ptr& state);
  virtual State::Ptr project(const State::Ptr& state, const Environment::Ptr& environment);
  virtual Eigen::Vector3d sampleUniformCOP2COM(std::mt19937& generator);

//protected:
  std::vector<flp::Primitive> collision_;
  std::vector<flp::Primitive> contact_;
};

