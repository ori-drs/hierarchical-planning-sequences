#include "ArchitectureSearch.h"
#include "StateSpaceModular.h"
#include "FeasibilityCheckerCostFinite.h"
#include "FeasibilityCheckerDistanceToStates.h"
#include "FeasibilityCheckerMaxDisplacement.h"
#include "PlannerSBPL.h"
#include "PlannerRecast.h"
#include "PlannerConnectReferencePath.h"
#include "EnvironmentConversions.h"
#include <grid_map_core/grid_map_core.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>

#ifndef FLPLIGHTWEIGHT
  #include "ViewerTrajopt.h"
#endif


//==========================================================================================================
// VirtualStateSpace

arch::VirtualStateSpace::VirtualStateSpace(const std::vector<std::string>& names) {
  for (unsigned int i = 0; i < names.size(); i++)
    units.push_back(VirtualStateSpaceUnit(names[i]));
}

bool arch::VirtualStateSpace::isContainedIn(const VirtualStateSpace& other) const {
  // check size of other is larger
  if (other.units.size() <= units.size())
    return false;
  // check all ours units are also in other
  for (unsigned int i = 0 ; i < units.size(); i++)
    if (std::find(other.units.begin(), other.units.end(), units[i]) == other.units.end())
      return false;
  return true;
}

std::string arch::VirtualStateSpace::to_string() const {
  std::stringstream ss;
  for (unsigned int i = 0; i < units.size(); i++)
    ss << units[i].getName() << " ";
  return ss.str();
}


//==========================================================================================================
// Model

bool arch::Model::isDefinedBy(const arch::VirtualStateSpace& space) const {
  // check we accept all state units in the space, and count those for which we dont have a default value
  unsigned int nondefaults = 0;
  for (unsigned int j = 0; j < space.units.size(); j++) {
    std::string s = space.units[j].getName();
    if (std::find(statesAccepted.begin(), statesAccepted.end(), s) == statesAccepted.end())
      return false;
    if (statesDefaultValue.find(s) == statesDefaultValue.end())
      nondefaults++;
  }
  // check all states for which we have no default value are set
  if (nondefaults == statesAccepted.size() - statesDefaultValue.size())
    return true;
  else
    return false;
}

bool arch::Model::isPartialOf(const Model& model) const {
  for (unsigned int j = 0; j < statesAccepted.size(); j++) {
    std::string s = statesAccepted[j];
    if (std::find(model.statesAccepted.begin(), model.statesAccepted.end(), s) == model.statesAccepted.end())
      return false;
    if (statesDefaultValue.find(s) == statesDefaultValue.end() && model.statesDefaultValue.find(s) != model.statesDefaultValue.end())
      return false;
    if (statesDefaultValue.find(s) != statesDefaultValue.end() && model.statesDefaultValue.find(s) == model.statesDefaultValue.end())
      return false;
    if (statesDefaultValue.find(s) != statesDefaultValue.end() && model.statesDefaultValue.find(s) != model.statesDefaultValue.end())
      if (statesDefaultValue.at(s) != model.statesDefaultValue.at(s))
        return false;
  }
  if (statesAccepted.size() == model.statesAccepted.size())
    return false;
  return true;
}

std::vector<std::string> arch::Model::minus(const Model& model) const {
  std::vector<std::string> diff;
  for (unsigned int j = 0; j < statesAccepted.size(); j++) {
    std::string s = statesAccepted[j];
    if (std::find(model.statesAccepted.begin(), model.statesAccepted.end(), s) == model.statesAccepted.end())
      diff.push_back(s);
  }
  return diff;
}

std::string arch::Model::to_string() const {
  std::stringstream ss;
  for (unsigned int i = 0; i < statesAccepted.size(); i++)
    ss << statesAccepted[i] << " ";
  return ss.str();
}


//==========================================================================================================
// Trajectory

std::vector<State::Ptr> arch::Trajectory::toStateVector(const StateSpace::Ptr& space) const
{
  std::vector<State::Ptr> states;
  for (unsigned int i = 0; i < vectors.size(); i++) {
    states.push_back( space->fromVector(vectors[i]) );
  }
  return states;
}


//==========================================================================================================
// Planner

arch::Planner::Planner()
  : useRecastHeuristic_(false)
  , verbose_(false)
  , lastCost_(0) 
  , plannerName_("SBPL")
  , plannerCosts_("distance")
  , plannerConstraints_("traversability")
{

}

std::vector<std::string> arch::Planner::explode(const std::string& s, char delim) const {
  std::vector<std::string> result;
  std::istringstream iss(s);
  for (std::string token; std::getline(iss, token, delim); ) {
    result.push_back(std::move(token));
  }
  return result;
}

StateSpace::Ptr arch::Planner::getStateSpace(const std::string& spaceName) const {
  std::vector<StateSpace::Ptr> subspaces;
  std::vector<std::string> dimNames;
  std::vector<double> weights;
  // go through each subspace name
  std::vector<unsigned int> subspaceIndices;
  std::vector<std::string> subspaceNames = explode(spaceName, ' ');
  for (unsigned int i = 0; i < subspaceNames.size(); i++) {
    // add corresponding subspace if we know the name
    if (subspaceNames[i] == "XY") {
      subspaces.push_back(StateSpace::Ptr( new StateSpaceReal(lb_.at("XY"), ub_.at("XY"), nbins_.at("XY")) ));
      subspaceIndices.push_back(subspaces.size()-1);
      subspaceIndices.push_back(subspaces.size()-1);
      dimNames.push_back("X");
      dimNames.push_back("Y");
      weights.push_back(1);
    } else if (subspaceNames[i] == "XYZ") {
      subspaces.push_back(StateSpace::Ptr( new StateSpaceReal(lb_.at("XYZ"), ub_.at("XYZ"), nbins_.at("XYZ")) ));
      subspaceIndices.push_back(subspaces.size()-1);
      subspaceIndices.push_back(subspaces.size()-1);
      subspaceIndices.push_back(subspaces.size()-1);
      dimNames.push_back("X");
      dimNames.push_back("Y");
      dimNames.push_back("Z");
      weights.push_back(1);
    } else if (subspaceNames[i] == "r") {
      subspaces.push_back(StateSpace::Ptr( new StateSpaceSO2(nbins_.at("r")[0]) ));
      subspaceIndices.push_back(subspaces.size()-1);
      dimNames.push_back("r");
      weights.push_back(0.4); // this should be the robot radius
    } else if (subspaceNames[i] == "p") {
      subspaces.push_back(StateSpace::Ptr( new StateSpaceSO2(nbins_.at("p")[0]) ));
      subspaceIndices.push_back(subspaces.size()-1);
      dimNames.push_back("p");
      weights.push_back(0.4); // this should be the robot radius
    } else if (subspaceNames[i] == "y") {
      subspaces.push_back(StateSpace::Ptr( new StateSpaceSO2(nbins_.at("y")[0]) ));
      subspaceIndices.push_back(subspaces.size()-1);
      dimNames.push_back("y");
      weights.push_back(0.4); // this should be the robot radius
    } else if (subspaceNames[i] == "M") {
      subspaces.push_back(StateSpace::Ptr( new StateSpaceDiscreteCircular(nbins_.at("M")[0]) ));
      subspaceIndices.push_back(subspaces.size()-1);
      dimNames.push_back("M");
      weights.push_back(0.01);
    }
  }
  if (subspaces.size() == 0)
    return StateSpace::Ptr();
  StateSpace::Ptr space( new StateSpaceModular(subspaces, weights) );
  space->setDimNames(dimNames, subspaceIndices);
  return space;
}

RobotModel::Ptr arch::Planner::getRobotModel(const std::string& spaceName, const StateSpace::Ptr& space, const std::vector<double>& plannerParameters) const {
  std::vector<std::string> subspaceNames = explode(spaceName, ' ');
  subspaceNames.erase(subspaceNames.begin());
  VirtualStateSpace vspace(subspaceNames);
  for (unsigned int i = 0; i < models_.size(); i++) {
    if (models_[i].first.isDefinedBy(vspace)) {
      models_[i].second->setStateSpace(space);
      return models_[i].second;
    }
  }
  return RobotModel::Ptr();
}

std::vector<FeasibilityChecker::Ptr> arch::Planner::getFeasibilityCheckers(const RobotModel::Ptr& model, const StateSpace::Ptr& space, const Trajectory& refTrajectory, const std::vector<double>& plannerParameters) const {
  std::vector<FeasibilityChecker::Ptr> checkers;
  // traversability
  if (plannerConstraints_ == "traversability") {
    StateTransitionCost::Ptr costTrav( new StateTransitionCostTraversability(model, false) );
    checkers.push_back(FeasibilityChecker::Ptr( new FeasibilityCheckerCostFinite(costTrav) ));
  }
  // traversability
  if (plannerConstraints_ == "traversabilityETH") {
    StateTransitionCost::Ptr costTrav( new StateTransitionCostTraversabilityETH(model, false) );
    checkers.push_back(FeasibilityChecker::Ptr( new FeasibilityCheckerCostFinite(costTrav) ));
  }
  // distance to reference trajectory
  if (false && refTrajectory.vectors.size() > 0) {
    if (verbose_) std::cout << "   Using reference trajectory as a constraint\n";
    double maxDistance = 0.5;
    StateSpace::Ptr refSpace = getStateSpace(refTrajectory.spaceName);
    std::vector<State::Ptr> states = refTrajectory.toStateVector(refSpace);
    std::vector<double> weights = states[0]->space_->getSubSpaceWeights();
    std::vector<double> weightsEqual(weights.size(), 1); // equal weights so that mode is followed exactly (0.3 < 1)
    checkers.push_back(FeasibilityChecker::Ptr( new FeasibilityCheckerDistanceToStates(states, maxDistance, weightsEqual) ));
  }
  // max displacement
  if (false) {
    checkers.push_back(FeasibilityChecker::Ptr( new FeasibilityCheckerMaxDisplacement(0.5) ));
  }
  return checkers;
}

StateTransitionCost::Ptr arch::Planner::getCost(const RobotModel::Ptr& model, const StateSpace::Ptr& space) const {
  if (plannerCosts_ == "distance") {
    return StateTransitionCost::Ptr( new StateTransitionCostDistance(model, false) );
  }
  if (plannerCosts_ == "energy") {
    return StateTransitionCost::Ptr( new StateTransitionCostExpectedEnergy(model, false, space) );
  }
  if (plannerCosts_ == "success") {
    return StateTransitionCost::Ptr( new StateTransitionCostProbabilitySuccess(model, false) );
  }
  if (plannerCosts_ == "traversabilityETH") {
    return StateTransitionCost::Ptr( new StateTransitionCostTraversabilityETH(model, false) );
  }
  if (plannerCosts_ == "utopianEnergySuccess") {
    std::vector<StateTransitionCost::Ptr> costs;
    costs.push_back(StateTransitionCost::Ptr( new StateTransitionCostProbabilitySuccess(model, false) ));
    costs.push_back(StateTransitionCost::Ptr( new StateTransitionCostExpectedEnergy(model, false, space) ));
    return StateTransitionCost::Ptr( new StateTransitionCostUtopia(model, false, costs, std::vector<double>(costs.size(), 0.5)) );
  }
}

std::vector< std::vector<double> > arch::Planner::plan(const std::string& spaceName, const std::vector<double>& plannerParameters, const Trajectory& refTrajectory) {
  // create state space for planning
  StateSpace::Ptr space = getStateSpace(spaceName);
  // create robot model
  RobotModel::Ptr model = getRobotModel(spaceName, space, plannerParameters);
  RobotModelFixedPrimitives::Ptr primModel = boost::dynamic_pointer_cast<RobotModelFixedPrimitives>(model);
  // robot model radius as an optimization parameter
  for (unsigned int i = 0; i < plannerParameterNames_.size(); i++) {
    if (plannerParameterNames_[i] == "radius") {
      if (primModel) {
        for (unsigned int c = 0; c < primModel->collision_.size(); c++) {
          primModel->collision_[c].sphere.radius = plannerParameters[i] * plannerParameterScales_[i];
          primModel->collision_[c].cylinder.radius = plannerParameters[i] * plannerParameterScales_[i];
        }
        for (unsigned int c = 0; c < primModel->contact_.size(); c++) {
          primModel->contact_[c].sphere.radius = plannerParameters[i] * plannerParameterScales_[i];
          primModel->contact_[c].cylinder.radius = plannerParameters[i] * plannerParameterScales_[i];
        }
      } else {
        std::cout << "WARNING: robot model does not allow to set radius\n";
      }
      break;
    }
  }
  // create feasibility checkers
  std::vector<FeasibilityChecker::Ptr> checkers = getFeasibilityCheckers(model, space, refTrajectory, plannerParameters);
  // create cost
  StateTransitionCost::Ptr cost = getCost(model, space);
  // create planner
  ::Planner::Ptr planner;
  if (plannerName_ == "SBPL")
    planner.reset( new PlannerSBPL(space, environment_, cost, checkers, model) );
  planner->maxTime_ = 2.0;
  // planning time as an optimization parameter
  for (unsigned int i = 0; i < plannerParameterNames_.size(); i++) {
    if (plannerParameterNames_[i] == "maxTime") {
      planner->maxTime_ = plannerParameters[i] * plannerParameterScales_[i];
      break;
    }
  }
  planner->optimizeSolution_ = true;
  planner->projectToEnvironment_ = true;
  if (viewer_) {
    viewer_->updateEnvironment(environment_);
    planner->attachViewer(viewer_, model);
  }
  // set heuristic cost-to-goal
  if (refTrajectory.vectors.size() > 0) {
    // reference trajectory
    if (verbose_) std::cout << "   Using reference trajectory as a heuristic distance-to-goal\n";
    StateSpace::Ptr refSpace = getStateSpace(refTrajectory.spaceName);
    std::vector<State::Ptr> states = refTrajectory.toStateVector(refSpace);
    ::Planner::Ptr heurPathLength( new PlannerConnectReferencePath(states) );
    planner->setHeuristicPathLength(heurPathLength);
  } else if (useRecastHeuristic_) {
    // recast
    ::Planner::Ptr heurPathLength( new PlannerRecast(envRecast_, model) );
    planner->setHeuristicPathLength(heurPathLength);
  }
  // create start/goal states
  State::Ptr startState = space->getStateZero();
  State::Ptr goalState  = space->getStateZero();
  space->setTransform(startState, Tstart_);
  space->setTransform(goalState,  Tgoal_);
  State::Ptr projStart = model->project(startState, environment_);
  State::Ptr projGoal = model->project(goalState, environment_);
  // debug
  if (verbose_) {
    std::cout << ">> Planning on space: " << spaceName << "\n";
    std::cout << "   Time budget: " << planner->maxTime_ << "\n";
    for (unsigned int i = 0; i < plannerParameterNames_.size(); i++)
      if (plannerParameterNames_[i] == "radius")
        std::cout << "   Robot model radius: " << plannerParameters[i] * plannerParameterScales_[i] << "\n";
  }
  if (viewer_) {
    // check feasibility
    bool feasibleStart = true;
    for (unsigned int c = 0; c < checkers.size() && feasibleStart; c++) {
      checkers[c]->updateEnvironment(environment_);
      if (!checkers[c]->feasible(projStart)) { printf("checker[%d] failed\n", c); feasibleStart = false; }
    }
    if (!feasibleStart) {
      StateStats stats;
      stats.compute(environment_, projStart, model);
      std::cout << "Start not feasible!!\n" << stats << "\n";
    }
    bool feasibleGoal = true;
    for (unsigned int c = 0; c < checkers.size() && feasibleGoal; c++) {
      checkers[c]->updateEnvironment(environment_);
      if (!checkers[c]->feasible(projGoal)) { printf("checker[%d] failed\n", c); feasibleGoal = false; }
    }
    if (!feasibleStart) {
      StateStats stats;
      stats.compute(environment_, projGoal, model);
      std::cout << "Goal not feasible!!\n" << stats << "\n";
    }
    // show
    printf("showing start: %s\n", space->toString(startState).c_str());
    viewer_->clearPrimitives();
    viewer_->addPrimitives(model->getPrimitives(startState));
    viewer_->drawAndIdle();
    printf("showing start proj: %s\n", space->toString(projStart).c_str());
    viewer_->clearPrimitives();
    viewer_->addPrimitives(model->getPrimitives(projStart));
    viewer_->drawAndIdle();
    printf("showing goal: %s\n", space->toString(goalState).c_str());
    viewer_->clearPrimitives();
    viewer_->addPrimitives(model->getPrimitives(goalState));
    viewer_->drawAndIdle();
    printf("showing goal proj: %s\n", space->toString(projGoal).c_str());
    viewer_->clearPrimitives();
    viewer_->addPrimitives(model->getPrimitives(projGoal));
    viewer_->drawAndIdle();
    viewer_->clearPrimitives();
  }
  // plan
  std::vector<StateTransition> transitions;
  planner->plan(projStart, projGoal, transitions);
  // return result
  std::vector< std::vector<double> > traj;
  if (transitions.size() > 0)
    traj.push_back(transitions[0].state->space_->toVector(transitions[0].state));
  for (unsigned int i = 0; i < transitions.size(); i++)
    traj.push_back(transitions[i].newState->space_->toVector(transitions[i].newState));
  // save cost
  lastCost_ = 0;
  for (unsigned int i = 0; i < transitions.size(); i++)
    lastCost_ += transitions[i].cost;
  // debug
  if (verbose_)
    std::cout << "   Planner " << (transitions.size() > 0 ? "FOUND" : "did NOT find") << " a trajectory on this space!\n";
  return traj;
}

void arch::Planner::setRecast(bool useRecast) {
  useRecastHeuristic_ = useRecast;
  if (!useRecastHeuristic_) {
    envRecast_.reset();
    return;
  }
  // create recast heuristic
  pcl::PolygonMesh mesh;
  if (boost::dynamic_pointer_cast<EnvironmentGridMap>(environment_) && convert(boost::dynamic_pointer_cast<EnvironmentGridMap>(environment_)->map_, mesh)) {
    // convert to recast
    envRecast_.reset(new EnvironmentRecast());
    envRecast_->build(mesh, recastParams_);
    // display
    if (viewer_) {
      printf("visualizing recast...\n");
      std::vector<Eigen::Vector3d> lines;
      envRecast_->recast_.getNavMesh(lines);
      viewer_->updateEnvironment(environment_);
      viewer_->addLineList(lines, Eigen::Vector4d(0,0,0,1));
      viewer_->drawAndIdle();
    }
  } else {
    printf("Could not convert to recast environment\n");
  }
}


//==========================================================================================================
// Planner Evaluator

arch::PlannerEvaluator::PlannerEvaluator() : dimension_(0), seedEnvironmentSampling_(5489u)
{

}

void arch::PlannerEvaluator::save(const std::vector<double>& vector, const std::string& filename, const std::string& extension)
{
  flp::Graph g = graph_;
  g.setEdgesVector(vector);

  std::vector<int> path = g.getShortestPath(0, 1);
  g.setColorPath(path, "green");

  if (extension == "png")
    g.savepng(filename);
  else if (extension == "pdf")
    g.savepdf(filename);
  else
    std::cout << "Unhandled file extension '" << extension << "'.\n";
}

void arch::PlannerEvaluator::saveLast(const std::string& filename, const std::string& extension)
{
  for (unsigned int i = 0; i < lastGraphDrawable_.size(); i++) {
    char num[100];
    sprintf(num, "prob%03d", (int)i);
    std::string filenamei = filename + std::string(num);
    if (extension == "png")
      lastGraphDrawable_[i].savepng(filenamei);
    else if (extension == "pdf")
      lastGraphDrawable_[i].savepdf(filenamei);
    else
      std::cout << "Unhandled file extension '" << extension << "'.\n";
  }
}

void arch::PlannerEvaluator::setDebug(bool debug)
{
  if (debug) {
    planner_.verbose_ = true;
    #ifndef FLPLIGHTWEIGHT
    planner_.viewer_.reset( new ViewerTrajopt() );
    planner_.viewer_->updateEnvironment(planner_.environment_);
    #else
    std::cout << "WARNING: library was compiled lightweight, viewer will not be used\n";
    #endif
  } else {
    planner_.verbose_ = false;
    planner_.viewer_.reset();
  }
}

void arch::PlannerEvaluator::setEnvironment(const std::string& strEnvironment)
{
  // FSC environment
  if (strEnvironment == "fsc") {
    // load PCL file
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud( new pcl::PointCloud<pcl::PointXYZRGB>() );
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB> ("fsc-oil-rig-map-full-1cm-clean.pcd", *cloud) == -1) {
      PCL_ERROR ("Couldn't read .pcd file \n");
      printf("Couldn't read .pcd file \n");
      return;
    }
    // chop off top floor
    std::cout << "Cropping Z...\n";
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered( new pcl::PointCloud<pcl::PointXYZRGB>() );
    //pcl::PassThrough<pcl::PointXYZRGB> pass;
    //pass.setInputCloud (cloud);
    //pass.setFilterFieldName ("z");
    //pass.setFilterLimits (-10.0, 1.0);
    //pass.filter (*cloud_filtered);
    for (unsigned int i = 0; i < cloud->points.size(); i++) {
      const pcl::PointXYZRGB &pt = cloud->points[i];
      if (pt.x >=   0.0 && pt.x <= 40.0 &&
          pt.y >=   0.0 && pt.y <= 80.0 &&
          pt.z >= -10.0 && pt.z <=  1.0) {
        cloud_filtered->points.push_back(pt);
      }
    }
    // convert to gridmap
    std::cout << "Converting to grid_map...\n";
    grid_map::GridMap gridmap({"elevation"});
    convert(cloud_filtered, gridmap, 0.05);
    // fix some issues with the map
    for (grid_map::SpiralIterator iterator(gridmap, grid_map::Position(9.65,22.3), 0.70); !iterator.isPastEnd(); ++iterator) { // close to stairs
      gridmap.at("elevation", *iterator) = -1.28;
    }
    for (grid_map::SpiralIterator iterator(gridmap, grid_map::Position(10.6,32.1), 0.30); !iterator.isPastEnd(); ++iterator) { // close to stairs bottom
      gridmap.at("elevation", *iterator) = -1.38;
    }
    for (grid_map::SpiralIterator iterator(gridmap, grid_map::Position(10,36.0), 0.80); !iterator.isPastEnd(); ++iterator) { // close to slab
      gridmap.at("elevation", *iterator) = -1.40;
    }
    // environment and start/goal poses
    EnvironmentGridMap::Ptr environment( new EnvironmentGridMap() );
    environment->map_ = gridmap;
    planner_.environment_ = environment;
    planner_.Tstart_ = flp::Transform(8.65,22.5,-1.3, 0.0, 0.0, 1.6); // stair entrance to room
    planner_.Tgoal_ =  flp::Transform(8.2, 28.0,-1.25,0.0, 0.0, 1.6);
    evaluatedStates_.clear();
    evaluatedStates_.push_back(planner_.Tstart_);
    evaluatedStates_.push_back(planner_.Tgoal_);
    // recast
    EnvironmentRecast::Params recastParams_fsc;
    recastParams_fsc.cellSize = 0.1f;
    recastParams_fsc.cellHeight = 0.1f;
    recastParams_fsc.agentHeight = 1.0f;
    recastParams_fsc.agentRadius = 0.1f;
    recastParams_fsc.agentMaxClimb = 0.3f;
    recastParams_fsc.agentMaxSlopeDegrees = 60.0f;
    planner_.recastParams_ = recastParams_fsc;
    // set bounds and discretization
    std::vector<double> xyzlb, xyzub;
    environment->getSpatialBounds(xyzlb, xyzub); xyzub[2] += 1.0;
    std::vector<int> nbins(3);
    nbins[0] = (xyzub[0] - xyzlb[0]) / 0.05;
    nbins[1] = (xyzub[1] - xyzlb[1]) / 0.05;
    nbins[2] = (xyzub[2] - xyzlb[2]) / 0.05;
    planner_.lb_["XYZ"] = xyzlb;
    planner_.ub_["XYZ"] = xyzub;
    planner_.nbins_["XYZ"] = nbins;
    // bounds for sampling random poses (custom around building)
    lbEnvironmentSampling_ = xyzlb;
    ubEnvironmentSampling_ = xyzub;
    lbEnvironmentSampling_[0] = 7.5;
    ubEnvironmentSampling_[0] = 14.0;
    lbEnvironmentSampling_[1] = 22.0;
    ubEnvironmentSampling_[1] = 33.0;
  } else {
    planner_.recastParams_ = EnvironmentRecast::Params();
    printf("Unknown map: %s \n", strEnvironment.c_str());
  }
  setRecast(planner_.useRecastHeuristic_);
}

void arch::PlannerEvaluator::setRecast(bool useRecast)
{
  planner_.setRecast(useRecast);
}

void arch::PlannerEvaluator::generateRandomProblems(int numProblems)
{
  // make sure we have an even number of poses
  if (numProblems <= 0) return;
  int numPoses = numProblems * 2;
  // get all spaces
  std::vector<std::string> spaceNames;
  std::vector<StateSpace::Ptr> spaces;
  for (int i = 0; i < graph_.getNumberOfNodes(); i++) {
    StateSpace::Ptr space = planner_.getStateSpace(graph_.getNodeName(i));
    if (space) {
      spaceNames.push_back(graph_.getNodeName(i));
      spaces.push_back(space);
    }
  }
  // generate random poses
  std::mt19937 gen(seedEnvironmentSampling_);
  evaluatedStates_.clear();
  while (evaluatedStates_.size() < numPoses) {
    // debug
    if (planner_.viewer_) {
      printf("Sampling new pose...\n");
    }
    // random point
    flp::Point pt = planner_.environment_->sampleUniform(gen);
    // check within sampling bounds
    bool withinSamplingBounds = true;
    for (unsigned int i = 0; i < lbEnvironmentSampling_.size() && withinSamplingBounds; i++) {
      if (pt.p(i) < lbEnvironmentSampling_[i] || pt.p(i) > ubEnvironmentSampling_[i])
        withinSamplingBounds = false;
    }
    if (!withinSamplingBounds)
      continue;
    // let's check feasibility for a few different angles
    for (unsigned int attemptAngle = 0; attemptAngle < 10; attemptAngle++) {
      // random angle
      std::uniform_real_distribution<double> angledist(-M_PI, M_PI);
      flp::Transform T(pt.p(0), pt.p(1), pt.p(2), 0, 0, angledist(gen));
      // check if state is feasible in all spaces
      bool feasible = true;
      for (unsigned int s = 0; s < spaces.size() && feasible; s++) {
        // space, model, feasibility
        StateSpace::Ptr space = spaces[s];
        std::vector<double> dummyP;
        Trajectory dummyT( std::vector< std::vector<double> >(), spaceNames[s] );
        RobotModel::Ptr model = planner_.getRobotModel(spaceNames[s], space, dummyP);
        std::vector<FeasibilityChecker::Ptr> checkers = planner_.getFeasibilityCheckers(model, space, dummyT, dummyP);
        // add one more feasibility check (large contact ratio)
        StateTransitionCost::Ptr costTrav( new StateTransitionCostTraversability(model, false, 0.6) );
        checkers.push_back(FeasibilityChecker::Ptr( new FeasibilityCheckerCostFinite(costTrav) ));
        // state
        State::Ptr state = space->getStateZero();
        space->setTransform(state, T);
        State::Ptr proj = model->project(state, planner_.environment_);
        // check feasibility
        for (unsigned int c = 0; c < checkers.size() && feasible; c++) {
          checkers[c]->updateEnvironment(planner_.environment_);
          if (!checkers[c]->feasible(proj)) feasible = false;
        }
        // debug
        if (planner_.viewer_) {
          printf("  showing sampled pose in space %s: %s\n", spaceNames[s].c_str(), feasible ? "feasible" : "NOT feasible");
          if (!feasible) {
            StateStats stats;
            stats.compute(planner_.environment_, proj, model);
            std::cout << stats << "\n";
          }
          planner_.viewer_->clearPrimitives();
          planner_.viewer_->addPrimitives(model->getPrimitives(proj));
          planner_.viewer_->draw();
        }
      }
      if (feasible) {
        evaluatedStates_.push_back(T);
        break;
      }
    }
  }
  // show all
  if (planner_.viewer_) {
    // show all together
    std::vector<Eigen::Vector3d> lineList;
    std::vector<Eigen::Vector4d> lineColors;
    planner_.viewer_->clearPrimitives();
    for (unsigned int i = 0; i < evaluatedStates_.size(); i++) {
      // space, model, feasibility
      StateSpace::Ptr space = spaces[0];
      std::vector<double> dummyP;
      Trajectory dummyT( std::vector< std::vector<double> >(), spaceNames[0] );
      RobotModel::Ptr model = planner_.getRobotModel(spaceNames[0], space, dummyP);
      // state
      State::Ptr state = space->getStateZero();
      space->setTransform(state, evaluatedStates_[i]);
      State::Ptr proj = model->project(state, planner_.environment_);
      // debug
      planner_.viewer_->addPrimitives(model->getPrimitives(proj));
      // line list
      flp::Transform Tproj;
      space->getTransform(proj, Tproj);
      lineList.push_back(Eigen::Vector3d(Tproj.x, Tproj.y, Tproj.z));
      if (i % 2 == 0) {
        double r = (double)rand() / (double)RAND_MAX;
        double g = (double)rand() / (double)RAND_MAX;
        double b = (double)rand() / (double)RAND_MAX;
        lineColors.push_back(Eigen::Vector4d(r,g,b,1));
      }
    }
    printf("Showing all poses...\n");
    planner_.viewer_->drawAndIdle();
    // show all together, as well as lines showing the pairs
    printf("Showing all poses plus lines connecting start-goal...\n");
    planner_.viewer_->addLineList(lineList, lineColors);
    planner_.viewer_->drawAndIdle();
    planner_.viewer_->clearLineList();
    // now show one by one
    for (unsigned int i = 0; i < evaluatedStates_.size(); i++) {
      const flp::Transform& T = evaluatedStates_[i];
      printf("Pose number %d: (%f, %f, %f) \n", (int)i, T.x, T.y, T.z);
      // space, model, feasibility
      StateSpace::Ptr space = spaces[0];
      std::vector<double> dummyP;
      Trajectory dummyT( std::vector< std::vector<double> >(), spaceNames[0] );
      RobotModel::Ptr model = planner_.getRobotModel(spaceNames[0], space, dummyP);
      // state
      State::Ptr state = space->getStateZero();
      space->setTransform(state, T);
      State::Ptr proj = model->project(state, planner_.environment_);
      // debug
      planner_.viewer_->clearPrimitives();
      planner_.viewer_->addPrimitives(model->getPrimitives(proj));
      planner_.viewer_->drawAndIdle();
    }
  }
}


//==========================================================================================================
// Architecure Search Functions

void arch::getCombinations(const std::vector<bool>& vec, unsigned int idx, std::vector< std::vector<bool> >& out) {
  if (vec.size() == 0) {
    return;
  }
  if (idx == vec.size()) {
    out.push_back(vec);
    return;
  }
  std::vector<bool> newvec = vec;
  newvec[idx] = true;
  getCombinations(newvec, idx+1, out);
  newvec[idx] = false;
  getCombinations(newvec, idx+1, out);
}

flp::Graph arch::getAllValidSubspaces(const arch::VirtualStateSpace& space, const std::vector<arch::Model>& models, const std::vector<double>& defaultEdgeParameters)
{
  // get all combinations of a 0-1 vector
  std::vector<bool> vec(space.units.size(), false);
  std::vector< std::vector<bool> > combinations;
  getCombinations(vec, 0, combinations);

  // get set of possible spaces
  std::vector<VirtualStateSpace> subspaces;
  subspaces.push_back(VirtualStateSpace()); // null space
  for (unsigned int i = 0; i < combinations.size(); i++) {
    VirtualStateSpace newspace;
    // convert
    for (unsigned int j = 0; j < space.units.size(); j++) {
      if (combinations[i][j])
        newspace.units.push_back(space.units[j]);
    }
    // check if space is plannable (there is a model that can be fully defined using this space)
    for (unsigned int m = 0; m < models.size(); m++) {
      if (models[m].isDefinedBy(newspace)) {
        subspaces.push_back(newspace);
        break;
      }
    }
  }

  // build a graph of transitions between spaces (directed edge a->b  if  a \in b)
  flp::Graph graph;
  for (unsigned int i = 0; i < subspaces.size(); i++) {
    for (unsigned int j = 0; j < subspaces.size(); j++) {
      if (subspaces[i].isContainedIn(subspaces[j])) {
        graph.addEdge(i, j, 1, defaultEdgeParameters);
        graph.setNodeName(i, std::to_string(i) + std::string(": ") + subspaces[i].to_string());
        graph.setNodeName(j, std::to_string(j) + std::string(": ") + subspaces[j].to_string());
      }
    }
  }
  graph.update();
  return graph;
}

std::vector< std::vector<double> > arch::executeEdge(const flp::Graph& graph, int i, int j, arch::Planner& planner)
{
  // plan on space j starting from a reference solution obtained on space i
  std::string statespace = graph.getNodeName(j);
  std::vector<double> plannerParameters = graph.getEdgeParameters(i, j);
  arch::Trajectory referenceTrajectory(graph.getNodeData(i), graph.getNodeName(i));
  return planner.plan(statespace, plannerParameters, referenceTrajectory);
}

int arch::executePath(flp::Graph& graph, const std::vector<int>& path, arch::Planner& planner)
{
  if (path.size() < 2) return 0;
  int executedEdges = 0;
  // try and execute path, storing edge observations in the graph, avoiding re-execution of edges for which observations already exist
  for (unsigned int i = 0; i < path.size()-1; i++) {
    // check if we have already observed edge i already
    std::vector< std::vector<double> > data_stored = graph.getNodeData(path[i+1]);
    if (data_stored.size() > 0) {
      executedEdges++;
      continue;
    }
    // get observation for edge i
    std::vector< std::vector<double> > data1 = graph.getNodeData(path[i]);
    std::vector< std::vector<double> > data2 = executeEdge(graph, path[i], path[i+1], planner); // execute edge (i.e. get data for node i+1)
    // check if execution was successful
    if (data2.size() > 0)
      executedEdges++;
    else
      break;
    // store observation for late use
    graph.setNodeData(path[i+1], data2);
  }
  return executedEdges;
}

bool arch::findShortestPathOnline(const flp::Graph& graph, int start, int goal, arch::Planner& planner, flp::Graph& graphUpdated, flp::Graph& graphDraw, std::vector<int>& graphPath)
{
  graphUpdated = graph;
  graphDraw = graph;
  int num_attempts = 0;
  while(true) {
    // debug: save current graph
    //char filename[100];
    //sprintf(filename, "arch-graph-rand-find%03d.png", num_attempts);
    //graphUpdated.savepng(filename);
    // find shortest path to the goal
    graphPath = graphUpdated.getShortestPath(start, goal);
    num_attempts++;
    if (planner.verbose_) {
      std::cout << "> Next hierarchical planning sequence: ";
      for (unsigned int i = 0; i < graphPath.size(); i++) std::cout << graphPath[i] << " ";
      std::cout << "(";
      for (unsigned int i = 0; i < graphPath.size(); i++) std::cout << graphUpdated.getNodeName(graphPath[i]) << "--> ";
      std::cout << ")\n";
    }
    // failure if none found
    if (graphPath.size() == 0) return false;
    // try and execute the path (reusing all reusable information from previous iterations)
    int executedEdges = executePath(graphUpdated, graphPath, planner);
    // color visited edges
    for (int i = 0; i < executedEdges; i++) {
      int node1 = graphPath[i];
      int node2 = graphPath[i+1];
      graphDraw.setColorEdge(node1, node2, "blue");
    }
    // exit if the execution was successful
    if (executedEdges == graphPath.size()-1)
      break;
    // if failed at some point then update graph
    if (executedEdges < graphPath.size()-1) {
      int node1 = graphPath[executedEdges];
      int node2 = graphPath[executedEdges+1];
      graphUpdated.removeEdge(node1, node2);
      graphUpdated.update();
      graphDraw.setColorEdge(node1, node2, "red");
    }
  }
  graphDraw.setColorPath(graphPath, "green");
  return true;
}

