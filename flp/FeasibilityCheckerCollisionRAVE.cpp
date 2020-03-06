#include "FeasibilityCheckerCollisionRAVE.h"

FeasibilityCheckerCollisionRAVE::FeasibilityCheckerCollisionRAVE()
{
  OpenRAVE::RaveInitialize(true, false ? OpenRAVE::Level_Debug : OpenRAVE::Level_Info);
  raveEnv_ = OpenRAVE::RaveCreateEnvironment();
  raveEnv_->StopSimulation();
  cc_ = OpenRAVE::RaveCreateCollisionChecker(raveEnv_, "ode");
}

FeasibilityCheckerCollisionRAVE::FeasibilityCheckerCollisionRAVE(const std::string& robotModelPath)
  : FeasibilityCheckerCollisionRAVE()
{
  setupFullbody(robotModelPath);
}

FeasibilityCheckerCollisionRAVE::FeasibilityCheckerCollisionRAVE(const std::string& robotModelPath, const std::string& linkName)
  : FeasibilityCheckerCollisionRAVE()
{
  setupBox(robotModelPath, linkName);
}

FeasibilityCheckerCollisionRAVE::~FeasibilityCheckerCollisionRAVE()
{
  raveEnv_->Destroy();
}

bool FeasibilityCheckerCollisionRAVE::feasible(const State::Ptr& state)
{
  if (!robotStateSpace_)
    return false;

  if (collisionModel_ == "fb") {
    // get robot state (Joints + SE3)
    StateJointsSE3::Ptr s = boost::dynamic_pointer_cast<StateJointsSE3>(state);
    if (!s) {
      s = boost::dynamic_pointer_cast<StateJointsSE3>( state->space_->project(state, robotStateSpace_) );
    }
    if (!s) {
      std::cout << "ERROR: input state is not displayable (i.e. can't project to StateJointsSE3)\n";
      return false;
    }
    flp::Transform T = s->getTransform();
    OpenRAVE::Transform orT(OpenRAVE::Vector(T.qx, T.qy, T.qz, T.qw), OpenRAVE::Vector(T.x, T.y, T.z)); // rot, trans
    std::vector<double> joints = boost::static_pointer_cast<StateReal>(s->states_[0])->values_;
    return feasibleFullbody(orT, joints);
  }

  if (collisionModel_ == "box") {
    // get robot state (SE3)
    StateSE3::Ptr s = boost::dynamic_pointer_cast<StateSE3>(state);
    if (!s) {
      s = boost::dynamic_pointer_cast<StateSE3>( state->space_->project(state, robotStateSpace_) );
    }
    if (!s) {
      std::cout << "ERROR: input state is not displayable (i.e. can't project to StateSE3)\n";
      return false;
    }
    flp::Transform T = s->getTransform();
    OpenRAVE::Transform orT(OpenRAVE::Vector(T.qx, T.qy, T.qz, T.qw), OpenRAVE::Vector(T.x, T.y, T.z)); // rot, trans
    return feasibleBox(orT);
  }

  return false;
}

bool FeasibilityCheckerCollisionRAVE::feasible(const StateTransition& transition)
{
  //TODO
  return false;
}

bool FeasibilityCheckerCollisionRAVE::feasible(const StateTransition& transition, double& validFraction)
{ 
  //TODO
  validFraction = 0.0;
  return false;
}

void FeasibilityCheckerCollisionRAVE::updateEnvironment(const Environment::Ptr& environment)
{
  EnvironmentPCL::Ptr p1 = boost::dynamic_pointer_cast<EnvironmentPCL>(environment);
  if (p1 != 0) return updateEnvironment(*p1);
  EnvironmentGridMap::Ptr p2 = boost::dynamic_pointer_cast<EnvironmentGridMap>(environment);
  if (p2 != 0) return updateEnvironment(*p2);
}

void FeasibilityCheckerCollisionRAVE::updateEnvironment(const EnvironmentPCL& environment)
{
  // clean up
  if (raveMesh_)
    raveEnv_->Remove(raveMesh_);
  raveMesh_ = RaveCreateKinBody(raveEnv_);

  // create mesh or set of meshes
  OpenRAVE::TriMesh trimesh;
  trimesh.vertices.resize(environment.cloud_->points.size());
  for (int i = 0; i < environment.cloud_->points.size(); i++) {
    trimesh.vertices[i].x = environment.cloud_->points[i].x;
    trimesh.vertices[i].y = environment.cloud_->points[i].y;
    trimesh.vertices[i].z = environment.cloud_->points[i].z;
  }
  trimesh.indices.resize(environment.mesh_->polygons.size()*3);
  for (int i = 0; i < environment.mesh_->polygons.size(); i++) {
    for (int j = 0; j < 3; j++) {
      trimesh.indices[i*3+j] = environment.mesh_->polygons[i].vertices[j];
    }
  }

  // init
  raveMesh_->InitFromTrimesh(trimesh,true);
  raveMesh_->GetLinks()[0]->GetGeometry(0)->SetAmbientColor(OpenRAVE::Vector(0.3,0.3,0.3));
  raveMesh_->GetLinks()[0]->GetGeometry(0)->SetDiffuseColor(OpenRAVE::Vector(0.3,0.3,0.3));

  // add mesh to env
  raveMesh_->SetName("trimesh");
  raveEnv_->Add(raveMesh_);
}

void FeasibilityCheckerCollisionRAVE::updateEnvironment(const EnvironmentGridMap& environment)
{
  // clean up
  if (raveMesh_) {
    raveEnv_->Remove(raveMesh_);
  }

  // create mesh (all cells same color for now, if we want color per triangle then we need set of trimeshes I guess)
  raveMesh_ = RaveCreateKinBody(raveEnv_);
  OpenRAVE::TriMesh trimesh;

  // add cells
  std::string layerZ = "elevation";
  const float resolution = environment.map_.getResolution();
  grid_map::Position position;
  for (grid_map::GridMapIterator iterator(environment.map_); !iterator.isPastEnd(); ++iterator) {
    environment.map_.getPosition(*iterator, position);
    float x = position.x();
    float y = position.y();
    float z = environment.map_.at(layerZ, *iterator);
    // simple version: two triangles at the same height (ignore neighbor cells)
    OpenRAVE::Vector v1(x-resolution, y-resolution, z);
    OpenRAVE::Vector v2(x+resolution, y-resolution, z);
    OpenRAVE::Vector v3(x+resolution, y+resolution, z);
    OpenRAVE::Vector v4(x-resolution, y+resolution, z);
    trimesh.vertices.push_back(v1);
    trimesh.vertices.push_back(v2);
    trimesh.vertices.push_back(v3);
    trimesh.vertices.push_back(v4);
    trimesh.indices.push_back(trimesh.vertices.size()-4);
    trimesh.indices.push_back(trimesh.vertices.size()-3);
    trimesh.indices.push_back(trimesh.vertices.size()-2);
    trimesh.indices.push_back(trimesh.vertices.size()-2);
    trimesh.indices.push_back(trimesh.vertices.size()-1);
    trimesh.indices.push_back(trimesh.vertices.size()-4);
  }

  // init
  raveMesh_->InitFromTrimesh(trimesh,true);
  raveMesh_->GetLinks()[0]->GetGeometry(0)->SetAmbientColor(OpenRAVE::Vector(0.3,0.3,0.3));
  raveMesh_->GetLinks()[0]->GetGeometry(0)->SetDiffuseColor(OpenRAVE::Vector(0.3,0.3,0.3));

  // add mesh to env
  raveMesh_->SetName("trimesh");
  raveEnv_->Add(raveMesh_);
}

void FeasibilityCheckerCollisionRAVE::setupBox(const OpenRAVE::AABB& box)
{
  std::vector<OpenRAVE::AABB> boxes;
  raveBox_ = RaveCreateKinBody(raveEnv_);
  raveBox_->InitFromBoxes(boxes, false);
  raveBox_->SetName("myBoxManual");
  raveEnv_->Add(raveBox_);
}

void FeasibilityCheckerCollisionRAVE::setupBox(const std::string& robotModelPath, const std::string& linkName)
{
  // load robot
  setupFullbody(robotModelPath);

  // add box
  OpenRAVE::KinBody::LinkPtr link = raveRobot_->GetLink(linkName);
  std::vector<OpenRAVE::AABB> boxes(1);
  boxes[0] = link->ComputeLocalAABB();
  boxes[0].pos = raveRobot_->GetTransform().inverse() * link->GetTransform() * boxes[0].pos;
  raveBox_ = RaveCreateKinBody(raveEnv_);
  raveBox_->InitFromBoxes(boxes, false);
  raveBox_->SetName("myBoxAuto");
  raveEnv_->Add(raveBox_);
  collisionModel_ = "box";

  // get state space
  std::vector<double> XYZlb(3,-1e6);
  std::vector<double> XYZub(3, 1e6);
  std::vector<int> XYZnb(3,0);
  int SO3nb(0);
  robotStateSpace_ = StateSpaceSE3::Ptr( new StateSpaceSE3(XYZlb,XYZub,XYZnb,SO3nb) );
}

void FeasibilityCheckerCollisionRAVE::setupFullbody(const std::string& robotModelPath)
{
  if (raveRobot_) {
    raveRobot_->Destroy();
    raveRobot_.reset();
  }
  // load robot
  std::vector<OpenRAVE::RobotBasePtr> robots;
  raveEnv_->Load(robotModelPath);
  raveEnv_->GetRobots(robots);
  if (robots.size() == 0)
    return;
  raveRobot_ = robots[0];
  collisionModel_ = "fb";

  // get state space
  std::vector<double> Jlb, Jub;
  raveRobot_->GetDOFLimits(Jlb, Jub);
  std::vector<int> Jnb(Jlb.size(),0);
  std::vector<double> XYZlb(3,-1e6);
  std::vector<double> XYZub(3, 1e6);
  std::vector<int> XYZnb(3,0);
  int SO3nb(0);
  robotStateSpace_ = StateSpaceJointsSE3::Ptr( new StateSpaceJointsSE3(Jlb,Jub,Jnb,XYZlb,XYZub,XYZnb,SO3nb) );
}

bool FeasibilityCheckerCollisionRAVE::feasibleBox(const OpenRAVE::Transform& T)
{
  raveBox_->SetTransform(T);
  return !cc_->CheckCollision(raveBox_, raveMesh_);
}

bool FeasibilityCheckerCollisionRAVE::feasibleFullbody(const OpenRAVE::Transform& T, const std::vector<double>& joints)
{
  raveRobot_->SetTransform(T);
  raveRobot_->SetDOFValues(joints, T, true);
  return !cc_->CheckCollision(raveRobot_, raveMesh_);
}

