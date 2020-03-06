#include "ArchitectureSearch.h"
#include "ArchitectureSearchAnymal.h"
#include "RobotModelFixedPrimitives.h"
#include "ViewerTrajopt.h"

bool DEBUG_ = true;
Viewer::Ptr VIEWER_;
arch::Planner PLANNER_;

void debug(const std::vector<flp::Primitive>& pri)
{
  if (DEBUG_) {
    if (!VIEWER_)
      VIEWER_.reset( new ViewerTrajopt() );
    VIEWER_->clearPrimitives();
    VIEWER_->addPrimitives(pri);
    VIEWER_->drawAndIdle();
  }
}

bool equal(const std::vector<flp::Primitive>& col1, const std::vector<flp::Primitive>& col2)
{
  for (unsigned int i = 0; i < col1.size(); i++) {
    bool foundEqual = false;
    for (unsigned int j = 0; j < col2.size(); j++) {
      if (i == j) continue;
      Eigen::Vector3d x1 = col1[i].sphere.center;
      Eigen::Vector3d x2 = col2[j].sphere.center;
      double r1 = col1[i].sphere.radius;
      double r2 = col2[j].sphere.radius;
      // if sphere i approx. equal j
      if (fabs(r1-r2)/std::max(r1,r2) < 0.1 && (x1-x2).norm()/std::max(r1,r2) < 0.1) {
        foundEqual = true;
        break;
      }
    }
    if (!foundEqual)
      return false;
  }
  return true;
}

std::vector<flp::Primitive> intersect(const std::vector<flp::Primitive>& col1, const std::vector<flp::Primitive>& col2)
{
  std::vector<flp::Primitive> inter;
  // whenever two spheres intersect, use an inscribed centered sphere as their approximate intersection
  for (unsigned int i = 0; i < col1.size(); i++) {
    for (unsigned int j = 0; j < col2.size(); j++) {
      Eigen::Vector3d x1 = col1[i].sphere.center;
      Eigen::Vector3d x2 = col2[j].sphere.center;
      double r1 = col1[i].sphere.radius;
      double r2 = col2[j].sphere.radius;
      // if spheres intersect
      if ((x1 - x2).norm() < r1 + r2) {
        if (x1 == x2 && r1 == r2) {
          // if spheres are equal
          inter.push_back(col1[i]);
        } else if (r1 < r2 && (x1 - x2).norm() <= r2 - r1) {
          // if sphere 1 contained in 2
          inter.push_back(col1[i]);
        } else if (r2 < r1 && (x1 - x2).norm() <= r1 - r2) {
          // if sphere 2 contained in 1
          inter.push_back(col2[i]);
        } else {
          // sphere tips
          Eigen::Vector3d t1 = x1 + (x2-x1).normalized() * r1;
          Eigen::Vector3d t2 = x2 + (x1-x2).normalized() * r2;
          // center will be midpoint, radius will be distance between tips
          flp::Primitive p = col1[i];
          p.sphere.center = (t1+t2) / 2;
          p.sphere.radius = (t1-t2).norm() / 2;
          inter.push_back(p);
        }
      }
    }
  }
  // from our result, remove any spheres inscribed in other spheres
  std::vector<flp::Primitive> interClean;
  for (unsigned int i = 0; i < inter.size(); i++) {
    bool remove = false;
    for (unsigned int j = 0; j < inter.size(); j++) {
      if (i == j) continue;
      Eigen::Vector3d x1 = inter[i].sphere.center;
      Eigen::Vector3d x2 = inter[j].sphere.center;
      double r1 = inter[i].sphere.radius;
      double r2 = inter[j].sphere.radius;
      // if sphere i contained in j, remove
      if (r1 < r2 && (x1-x2).norm() <= r2 - r1 + 0.1 * std::max(r1,r2)) {
        remove = true;
        break;
      }
    }
    if (!remove)
      interClean.push_back(inter[i]);
  }
  // remove any repeated spheres
  std::vector<flp::Primitive> interClean2;
  for (unsigned int i = 0; i < interClean.size(); i++) {
    bool remove = false;
    for (unsigned int j = i+1; j < interClean.size(); j++) {
      Eigen::Vector3d x1 = interClean[i].sphere.center;
      Eigen::Vector3d x2 = interClean[j].sphere.center;
      double r1 = interClean[i].sphere.radius;
      double r2 = interClean[j].sphere.radius;
      // if sphere i approx. equal j, remove
      if (fabs(r1-r2)/std::max(r1,r2) < 0.1 && (x1-x2).norm()/std::max(r1,r2) < 0.1) {
        remove = true;
        break;
      }
    }
    if (!remove)
      interClean2.push_back(interClean[i]);
  }
  return interClean2;
}

std::pair<arch::Model, RobotModel::Ptr> getSubspaceCollisionModel(const StateSpace::Ptr& fullspace, const RobotModel::Ptr& fullmodel, const arch::VirtualStateSpace& vsubspace)
{
  std::cout << "Getting robot model for subspace " << vsubspace.to_string() << "\n";

  std::mt19937 gen;
  StateSpace::Ptr subspace = PLANNER_.getStateSpace(vsubspace.to_string());
  std::map<std::string, unsigned int> fullspaceDimNamesMap = fullspace->getDimNamesMap();
  std::map<std::string, unsigned int> subspaceDimNamesMap = subspace->getDimNamesMap();

  // build abstract model...
  arch::Model abstractModel;
  for (unsigned int u = 0; u < vsubspace.units.size(); u++)
    abstractModel.statesAccepted.push_back(vsubspace.units[u].getName());

  // special cases
  if (vsubspace.to_string() == "XYZ p y M ")
    return std::pair<arch::Model, RobotModel::Ptr>(abstractModel, fullmodel);

  // build collision model... initial estimate = full model
  State::Ptr state0 = fullspace->getStateZero();
  std::vector<flp::Primitive> col0 = fullmodel->getPrimitivesCollision(state0);
  std::vector<flp::Primitive> H = col0;
  std::cout << fullspace->toString(state0) << "\n";
  debug(H);

  // loop
  for (unsigned int iter = 0; iter < 50; iter++) {
    // random state
    State::Ptr newstate = fullspace->sampleUniform(gen);
    // set common dimensions equal to what is in state0 (because we only want to randomize S\Si)
    std::vector<double> vec = fullspace->toVector(state0);
    std::vector<double> newvec = fullspace->toVector(newstate);
    for (std::map<std::string, unsigned int>::const_iterator it = fullspaceDimNamesMap.begin(); it != fullspaceDimNamesMap.end(); ++it) {
      // all subspace dimensions = state0
      auto m = subspaceDimNamesMap.find(it->first);
      if (m != subspaceDimNamesMap.end()) {
        newvec[it->second] = vec[it->second];
      }
      // special case "pitch" dimension = state0 (because pitch range in anymal is narrow, not full 360deg)
      if (it->first == "p") {
        newvec[it->second] = vec[it->second];
      }
    }
    newstate = fullspace->fromVector(newvec);
    // intersect estimated geometry with new one
    std::vector<flp::Primitive> col = fullmodel->getPrimitivesCollision(newstate);
    H = intersect(H, col);
    //std::cout << fullspace->toString(newstate) << "\n";
    //debug(col);
    //debug(H);
  }
  for (unsigned int i = 0; i < H.size(); i++)
    std::cout << "  geom " << H[i].sphere.center.transpose() << " " << H[i].sphere.radius << "\n";
  debug(H);
  RobotModel::Ptr collisionModel( new RobotModelFixedPrimitives(StateSpace::Ptr(), H, std::vector<flp::Primitive>()));

  // finished
  return std::pair<arch::Model, RobotModel::Ptr>(abstractModel, collisionModel);
}


int main (int argc, char *argv[])
{
  // full state space
  arch::VirtualStateSpace space;
  space.units.push_back(arch::VirtualStateSpaceUnit("XYZ"));
  space.units.push_back(arch::VirtualStateSpaceUnit("p"));
  space.units.push_back(arch::VirtualStateSpaceUnit("y"));
  space.units.push_back(arch::VirtualStateSpaceUnit("M"));

  PLANNER_.lb_["XYZ"] = std::vector<double>(3,-2.0);
  PLANNER_.ub_["XYZ"] = std::vector<double>(3, 2.0);
  PLANNER_.nbins_["XYZ"] = std::vector<int>(3, 200);
  PLANNER_.nbins_["p"] = std::vector<int>(1, 36);
  PLANNER_.nbins_["y"] = std::vector<int>(1, 36);
  PLANNER_.nbins_["M"] = std::vector<int>(1, 3);

  StateSpace::Ptr fullspace = PLANNER_.getStateSpace(space.to_string());

  // full robot collision model
  std::vector<flp::Primitive> pmodelFeetBody;
  {
    flp::Primitive body;
    body.type = flp::PrimitiveTypeSphere;
    body.sphere.center << 0.2, 0.0, 0.05;
    body.sphere.radius = 0.30;
    pmodelFeetBody.push_back(body);
    body.sphere.center(0) = 0.0;
    pmodelFeetBody.push_back(body);
    body.sphere.center(0) =-0.2;
    pmodelFeetBody.push_back(body);
  }
  std::vector<flp::Primitive> pmodelFeetCont;
  {
    flp::Primitive c;
    c.type = flp::PrimitiveTypeSphere;
    c.sphere.radius = 0.15;
    c.sphere.center <<  0.33, 0.22, -0.54; pmodelFeetCont.push_back(c);
    c.sphere.center << -0.33, 0.22, -0.54; pmodelFeetCont.push_back(c);
    c.sphere.center <<  0.33,-0.22, -0.54; pmodelFeetCont.push_back(c);
    c.sphere.center << -0.33,-0.22, -0.54; pmodelFeetCont.push_back(c);
  }
  RobotModel::Ptr fullmodel( new RobotModelFixedPrimitives(StateSpace::Ptr(), pmodelFeetBody, pmodelFeetCont) );

  // get all combinations of a 0-1 vector
  std::vector<bool> vec(space.units.size(), false);
  std::vector< std::vector<bool> > combinations;
  arch::getCombinations(vec, 0, combinations);

  // get all possible subspaces
  std::vector<arch::VirtualStateSpace> subspaces;
  for (unsigned int i = 0; i < combinations.size(); i++) {
    arch::VirtualStateSpace newspace;
    // convert
    for (unsigned int j = 0; j < space.units.size(); j++) {
      if (combinations[i][j])
        newspace.units.push_back(space.units[j]);
    }
    if (newspace.units.size() > 0)
      subspaces.push_back(newspace);
  }

  // get robot collision model for each subspace
  std::vector< std::pair<arch::Model, RobotModel::Ptr> > models;
  for (unsigned int i = 0; i < subspaces.size(); i++) {
    std::pair<arch::Model, RobotModel::Ptr> modelpair = getSubspaceCollisionModel(fullspace, fullmodel, subspaces[i]);
    // discard if there is no geometry (useless as heuristic since all states would be feasible)
    if (boost::static_pointer_cast<RobotModelFixedPrimitives>(modelpair.second)->collision_.size() > 0)
      models.push_back(modelpair);
  }

  // discard some models to simplify graph?
  std::vector< std::pair<arch::Model, RobotModel::Ptr> > models2;
  if (true) {
    for (unsigned int i = 0; i < models.size(); i++) {
      if (models[i].first.to_string() == "XYZ p " || models[i].first.to_string() == "XYZ p M ")
        std::cout << "Discarding model [ " << models[i].first.to_string() << "] to simplify...\n";
      else
        models2.push_back(models[i]);
    }
  } else {
    models2 = models;
  }

  // show all models
  std::cout << "Finished. Now showing all non-empty robot models\n";
  std::vector<arch::Model> abstractModels;
  for (unsigned int i = 0; i < models2.size(); i++) {
    std::cout << "Showing model " << i << " ( ";
    for (unsigned int j = 0; j < models2[i].first.statesAccepted.size(); j++)
      std::cout << models2[i].first.statesAccepted[j] << " ";
    std::cout << ")...\n";
    debug(boost::static_pointer_cast<RobotModelFixedPrimitives>(models2[i].second)->collision_);
    abstractModels.push_back(models2[i].first);
  }

  // get architecture graph
  std::cout << "Creating and saving graph-representation of the hierarchical planner \n";
  flp::Graph graph = arch::getAllValidSubspaces(space, abstractModels, std::vector<double>());
  graph.savepdf("test-subspace-models.pdf");

  return 0;
}

