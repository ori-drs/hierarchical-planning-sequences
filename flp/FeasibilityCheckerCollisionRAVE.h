#pragma once
#include "FeasibilityChecker.h"
#include "EnvironmentPCL.h"
#include "EnvironmentGridMap.h"
#include "StateSpace.h"
#include <openrave-core.h>
//TODO: replace this checker by the feasibilitycheckercollision with a EnvironmentRAVE. should make a PCL<>RAVE GridMap<>RAVE conversion for that.

class FeasibilityCheckerCollisionRAVE : public FeasibilityChecker
{
public:
  typedef boost::shared_ptr<FeasibilityCheckerCollisionRAVE> Ptr;

  FeasibilityCheckerCollisionRAVE();
  FeasibilityCheckerCollisionRAVE(const std::string& robotModelPath);
  FeasibilityCheckerCollisionRAVE(const std::string& robotModelPath, const std::string& linkName);
  ~FeasibilityCheckerCollisionRAVE();
  virtual bool feasible(const State::Ptr& state);
  virtual bool feasible(const StateTransition& transition);
  virtual bool feasible(const StateTransition& transition, double& validFraction);
  virtual void updateEnvironment(const Environment::Ptr& environment);

protected:
  void updateEnvironment(const EnvironmentPCL& environment);
  void updateEnvironment(const EnvironmentGridMap& environment);
  void setupBox(const OpenRAVE::AABB& box);
  void setupBox(const std::string& robotModelPath, const std::string& linkName);
  void setupFullbody(const std::string& robotModelPath);
  bool feasibleBox(const OpenRAVE::Transform& T);
  bool feasibleFullbody(const OpenRAVE::Transform& T, const std::vector<double>& joints);

  StateSpace::Ptr robotStateSpace_;

  OpenRAVE::EnvironmentBasePtr raveEnv_;
  OpenRAVE::KinBodyPtr raveMesh_;
  OpenRAVE::CollisionCheckerBasePtr cc_;

  std::string collisionModel_;
  OpenRAVE::RobotBasePtr raveRobot_;
  OpenRAVE::KinBodyPtr raveBox_;
};

