#pragma once
#include "RobotModel.h"
#include <openrave-core.h>

class RobotModelRAVE : RobotModel
{
public:
  typedef boost::shared_ptr<RobotModelRAVE> Ptr;
  RobotModelRAVE(const StateSpaceJointsSE3::Ptr& space, const std::string& robotModelPath, const std::vector<std::string>& linksCollision, const std::vector<std::string>& linksContact);

  std::vector<flp::Primitive> getPrimitivesCollision(const State::Ptr& state);
  std::vector<flp::Primitive> getPrimitivesContact(const State::Ptr& state);
  virtual State::Ptr project(const State::Ptr& state, const Environment::Ptr& environment);
  virtual Eigen::Vector3d sampleUniformCOP2COM(std::mt19937& generator);

protected:
  std::vector<OpenRAVE::KinBody::LinkPtr> linksCollision_;
  std::vector<OpenRAVE::KinBody::LinkPtr> linksContact_;
  OpenRAVE::EnvironmentBasePtr raveEnv_;
  OpenRAVE::RobotBasePtr raveRobot_;
};

