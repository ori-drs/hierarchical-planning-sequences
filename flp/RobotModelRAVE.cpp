#include "RobotModelRAVE.h"

RobotModelRAVE::RobotModelRAVE(const StateSpaceJointsSE3::Ptr& space, const std::string& robotModelPath, const std::vector<std::string>& linksCollision, const std::vector<std::string>& linksContact)
  : RobotModel(space)
{
  // setup openrave
  OpenRAVE::RaveInitialize(true, false ? OpenRAVE::Level_Debug : OpenRAVE::Level_Info);
  raveEnv_ = OpenRAVE::RaveCreateEnvironment();
  raveEnv_->StopSimulation();

  // load robot
  std::vector<OpenRAVE::RobotBasePtr> robots;
  raveEnv_->Load(robotModelPath);
  raveEnv_->GetRobots(robots);
  if (robots.size() == 0)
    return;
  raveRobot_ = robots[0];

  // get links
  for (unsigned int i = 0; i < linksCollision.size(); i++) {
    OpenRAVE::KinBody::LinkPtr link = raveRobot_->GetLink(linksCollision[i]);
    if (link)
      linksCollision_.push_back(link);
  }
  for (unsigned int i = 0; i < linksContact.size(); i++) {
    OpenRAVE::KinBody::LinkPtr link = raveRobot_->GetLink(linksContact[i]);
    if (link)
      linksContact_.push_back(link);
  }
}

std::vector<flp::Primitive> RobotModelRAVE::getPrimitivesCollision(const State::Ptr& state)
{
  StateJointsSE3::Ptr s = boost::static_pointer_cast<StateJointsSE3>(state);
  flp::Transform T = s->getTransform();
  std::vector<double> joints = boost::static_pointer_cast<StateReal>(s->states_[0])->values_;
  // set robot state on openrave
  OpenRAVE::Transform orT(OpenRAVE::Vector(T.qx, T.qy, T.qz, T.qw), OpenRAVE::Vector(T.x, T.y, T.z)); // rot, trans
  raveRobot_->SetTransform(orT); //TODO: check if this is necessary
  raveRobot_->SetDOFValues(joints, orT, true);
  // TODO: get link boxes
  return std::vector<flp::Primitive>();
}

std::vector<flp::Primitive> RobotModelRAVE::getPrimitivesContact(const State::Ptr& state)
{
  StateJointsSE3::Ptr s = boost::static_pointer_cast<StateJointsSE3>(state);
  flp::Transform T = s->getTransform();
  std::vector<double> joints = boost::static_pointer_cast<StateReal>(s->states_[0])->values_;
  // set robot state on openrave
  OpenRAVE::Transform orT(OpenRAVE::Vector(T.qx, T.qy, T.qz, T.qw), OpenRAVE::Vector(T.x, T.y, T.z)); // rot, trans
  raveRobot_->SetTransform(orT); //TODO: check if this is necessary
  raveRobot_->SetDOFValues(joints, orT, true);
  // TODO: get link boxes
  return std::vector<flp::Primitive>();
}

State::Ptr RobotModelRAVE::project(const State::Ptr& state, const Environment::Ptr& environment)
{
  std::cout << "TODO\n";
  return state;
}

Eigen::Vector3d RobotModelRAVE::sampleUniformCOP2COM(std::mt19937& generator)
{
  std::cout << "TODO\n";
  return Eigen::Vector3d(0,0,0);
}

