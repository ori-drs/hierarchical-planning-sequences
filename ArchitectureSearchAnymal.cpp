#include "ArchitectureSearchAnymal.h"
#include "RobotModelFixedPrimitives.h"
#include "EnvironmentGridMap.h"
#include "EnvironmentConversions.h"
#include <grid_map_core/grid_map_core.hpp>
#include <boost/timer/timer.hpp>

arch::PlannerEvaluatorAnymal::PlannerEvaluatorAnymal()
{
  // state space units
  arch::VirtualStateSpace space;
  space.units.push_back(arch::VirtualStateSpaceUnit("XYZ"));
//space.units.push_back(arch::VirtualStateSpaceUnit("r"));
  space.units.push_back(arch::VirtualStateSpaceUnit("p"));
  space.units.push_back(arch::VirtualStateSpaceUnit("y"));
  space.units.push_back(arch::VirtualStateSpaceUnit("M"));

  // robot models
  arch::Model modelCyli;
  modelCyli.statesAccepted.push_back("XYZ");
  modelCyli.statesAccepted.push_back("M");
  modelCyli.statesDefaultValue["M"] = 0;

  arch::Model modelCyl2;
  modelCyl2.statesAccepted.push_back("XYZ");
  modelCyl2.statesAccepted.push_back("y");

  arch::Model modelFeet;
  modelFeet.statesAccepted.push_back("XYZ");
//modelFeet.statesAccepted.push_back("r");
  modelFeet.statesAccepted.push_back("p");
  modelFeet.statesAccepted.push_back("y");
  modelFeet.statesAccepted.push_back("M");
//modelFeet.statesDefaultValue["r"] = 0;
  modelFeet.statesDefaultValue["p"] = 0;
  modelFeet.statesDefaultValue["M"] = 0;

  std::vector<arch::Model> models;
  models.push_back(modelCyli);
  models.push_back(modelCyl2);
  models.push_back(modelFeet);

  // planner robot models
  std::vector<flp::Primitive> pmodelCyliBody;
  {
    flp::Primitive body;
    body.type = flp::PrimitiveTypeSphere;
    body.sphere.center << 0.0, 0.0, 0.05;
    body.sphere.radius = 0.30;
    pmodelCyliBody.push_back(body);
  }
  std::vector<flp::Primitive> pmodelCyliCont;
  {
    flp::Primitive c;
    c.type = flp::PrimitiveTypeSphere;
    c.sphere.radius = 0.30;
    c.sphere.center <<  0.0, 0.0, -0.54; pmodelCyliCont.push_back(c);
  }
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
  RobotModel::Ptr pmodelCyli( new RobotModelFixedPrimitives(StateSpace::Ptr(), pmodelCyliBody, pmodelCyliCont) );
  RobotModel::Ptr pmodelFeet( new RobotModelFixedPrimitives(StateSpace::Ptr(), pmodelFeetBody, pmodelFeetCont) );
  pmodelCyli->setContactProjectionOffset(Eigen::Vector3d(0,0,0.15));
  pmodelFeet->setContactProjectionOffset(Eigen::Vector3d(0,0,0.05));
  std::vector< std::pair<arch::Model, RobotModel::Ptr> > pmodels;
  pmodels.push_back( std::pair<arch::Model, RobotModel::Ptr>(modelCyli, pmodelCyli) );
  pmodels.push_back( std::pair<arch::Model, RobotModel::Ptr>(modelCyl2, pmodelCyli) );
  pmodels.push_back( std::pair<arch::Model, RobotModel::Ptr>(modelFeet, pmodelFeet) );

  // planner setup
  planner_.lb_["XYZ"] = std::vector<double>(3,-2.0);
  planner_.ub_["XYZ"] = std::vector<double>(3, 2.0);
  planner_.nbins_["XYZ"] = std::vector<int>(3, 200);
//planner_.nbins_["r"] = std::vector<int>(1, 36);
  planner_.nbins_["p"] = std::vector<int>(1, 36);
  planner_.nbins_["y"] = std::vector<int>(1, 36);
  planner_.nbins_["M"] = std::vector<int>(1, 3);
  planner_.models_ = pmodels;

  // waves environment
  grid_map::GridMap map({"elevation"});
  map.setFrameId("map");
  map.setGeometry(grid_map::Length(4.0, 4.0), 0.03);
  double t = 100;
  for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
    grid_map::Position position;
    map.getPosition(*it, position);
    map.at("elevation", *it) = -0.6 + 0.05 * std::sin(3.0 * t + 5.0 * position.y());
  }
  EnvironmentGridMap::Ptr environment( new EnvironmentGridMap() );
  environment->map_ = map;

  // environment and start/goal poses
  planner_.environment_ = environment;
  planner_.Tstart_ = flp::Transform(0,0,0,0,0,0);
  planner_.Tgoal_ = flp::Transform(1.5,1.5,0,0,-0.15,3.14/2);

  evaluatedStates_.clear();
  evaluatedStates_.push_back(planner_.Tstart_);
  evaluatedStates_.push_back(planner_.Tgoal_);

  // architecture graph
  std::vector<double> defaultEdgeParameters(1,0);
  graph_ = arch::getAllValidSubspaces(space, models, defaultEdgeParameters);

  // planner parameters to be optimized
  planner_.plannerParameterNames_ .push_back("maxTime");
  planner_.plannerParameterScales_.push_back(50./100.);

  // planner costs and constraints
  planner_.plannerConstraints_ = "traversability";
  planner_.plannerCosts_ = "utopianEnergySuccess";

  // architecture properties
  std::vector<double> vec = graph_.getEdgesVector();
  dimension_ = vec.size();
  lb_ = std::vector<double>(dimension_, 1);
  ub_ = std::vector<double>(dimension_, 100);
}

void arch::PlannerEvaluatorAnymal::setParamsBaseline(const std::string& baseline)
{
  if (baseline == "fullspace") {

    // plan directly on the full space (nodes 0 and 1)
    std::vector<double> params = graph_.getEdgeParameters(0,1);
    flp::Graph graph;
    graph.addEdge(0, 1, 1, params);
    graph.setNodeName(0, graph_.getNodeName(0));
    graph.setNodeName(1, graph_.getNodeName(1));
    graph.update();
    graph_ = graph;
    // limits
    std::vector<double> vec = graph_.getEdgesVector();
    dimension_ = vec.size();
    lb_ = std::vector<double>(dimension_, 1);
    ub_ = std::vector<double>(dimension_, 100);

  } else if (baseline == "hierarchy" || baseline == "hierarchyXYZ") {

    // single hierarchy architecture: XYZ-then-fullspace
    std::vector<double> params = graph_.getEdgeParameters(0,1);
    flp::Graph graph;
    graph.addEdge(0, 2, 1, params);
    graph.addEdge(2, 1, 1, params);
    graph.setNodeName(0, graph_.getNodeName(0));
    graph.setNodeName(1, graph_.getNodeName(1));
    graph.setNodeName(2, "2: XYZ ");
    graph.update();
    graph_ = graph;
    // limits
    std::vector<double> vec = graph_.getEdgesVector();
    dimension_ = vec.size();
    lb_ = std::vector<double>(dimension_, 1);
    ub_ = std::vector<double>(dimension_, 100);

  } else if (baseline == "hierarchyXYZM") {

    // single hierarchy architecture: XYZ-then-fullspace
    std::vector<double> params = graph_.getEdgeParameters(0,1);
    flp::Graph graph;
    graph.addEdge(0, 2, 1, params);
    graph.addEdge(2, 1, 1, params);
    graph.setNodeName(0, graph_.getNodeName(0));
    graph.setNodeName(1, graph_.getNodeName(1));
    graph.setNodeName(2, "2: XYZ M ");
    graph.update();
    graph_ = graph;
    // limits
    std::vector<double> vec = graph_.getEdgesVector();
    dimension_ = vec.size();
    lb_ = std::vector<double>(dimension_, 1);
    ub_ = std::vector<double>(dimension_, 100);

  } else if (baseline == "hierarchyXYZy") {

    // single hierarchy architecture: XYZ-then-fullspace
    std::vector<double> params = graph_.getEdgeParameters(0,1);
    flp::Graph graph;
    graph.addEdge(0, 2, 1, params);
    graph.addEdge(2, 1, 1, params);
    graph.setNodeName(0, graph_.getNodeName(0));
    graph.setNodeName(1, graph_.getNodeName(1));
    graph.setNodeName(2, "2: XYZ y ");
    graph.update();
    graph_ = graph;
    // limits
    std::vector<double> vec = graph_.getEdgesVector();
    dimension_ = vec.size();
    lb_ = std::vector<double>(dimension_, 1);
    ub_ = std::vector<double>(dimension_, 100);

  } else if (baseline == "BFS") {

    // remove edges from 0 to all-except-XYZ
    std::vector<int> nei = graph_.getNeighbors(0);
    for (unsigned int i = 0; i < nei.size(); i++) {
      std::string spaceName = graph_.getNodeName(nei[i]);
      std::vector<std::string> subspaceNames = planner_.explode(spaceName, ' ');
      subspaceNames.erase(subspaceNames.begin());
      if (subspaceNames.size() > 1)
        graph_.removeEdge(0, nei[i]);
    }

    // do a transitive reduction (Styler 2017)
    graph_.transitiveReduction();

    // get new limits
    graph_.update();
    std::vector<double> vec = graph_.getEdgesVector();
    dimension_ = vec.size();
    lb_ = std::vector<double>(dimension_, 1);
    ub_ = std::vector<double>(dimension_, 100);

    // fix costs
    for (unsigned int i = 0; i < lb_.size(); i+=2)
      ub_[i] = lb_[i];

  } else {

    std::cout << "WARNING: baseline '" << baseline << "' does not exist. Ignoring...\n";

  }
}

void arch::PlannerEvaluatorAnymal::evaluate(const std::vector<double>& vector)
{
  // set architecture parameters
  graph_.setEdgesVector(vector);

  // stats
  std::vector<double> allTimes;
  std::vector<double> allTimesMeasured;
  std::vector<double> allCosts;
  std::vector<bool> allSuccess;

  lastGraphDrawable_.clear();

  // for each planning problem (two states)
  int Nproblems = (int)(evaluatedStates_.size()) / 2;
  for (int p = 0; p < Nproblems; p++) {

    planner_.Tstart_ = evaluatedStates_[2*p];
    planner_.Tgoal_  = evaluatedStates_[2*p+1];

    // solve planning problem using our graph-based architecture
    flp::Graph graphUpdated;
    std::vector<int> graphPath;
    lastGraphDrawable_.push_back(graphUpdated);

    boost::timer::cpu_timer timer;
    bool found = arch::findShortestPathOnline(graph_, 0, 1, planner_, graphUpdated, lastGraphDrawable_[p], graphPath);
    boost::timer::cpu_times const elapsedTimes(timer.elapsed());
    double timeSec = (elapsedTimes.user + elapsedTimes.system) / 1e9;

    if (found && graphPath.size() > 0) {
      // time (sum of the expected execution time over all visited edges - those that we tried to execute)
      lastEvaluationTime_ = 0;
      std::vector<int> visitedEdges = lastGraphDrawable_[p].getEdgesWithColor("blue");
      for (unsigned int i = 0; i < visitedEdges.size(); i+=2) {
        std::vector<double> params = lastGraphDrawable_[p].getEdgeParameters(visitedEdges[i], visitedEdges[i+1]);
        lastEvaluationTime_ += params[0] * planner_.plannerParameterScales_[0];
      }
      visitedEdges = lastGraphDrawable_[p].getEdgesWithColor("red");
      for (unsigned int i = 0; i < visitedEdges.size(); i+=2) {
        std::vector<double> params = lastGraphDrawable_[p].getEdgeParameters(visitedEdges[i], visitedEdges[i+1]);
        lastEvaluationTime_ += params[0] * planner_.plannerParameterScales_[0];
      }
      visitedEdges = lastGraphDrawable_[p].getEdgesWithColor("green");
      for (unsigned int i = 0; i < visitedEdges.size(); i+=2) {
        std::vector<double> params = lastGraphDrawable_[p].getEdgeParameters(visitedEdges[i], visitedEdges[i+1]);
        lastEvaluationTime_ += params[0] * planner_.plannerParameterScales_[0];
      }
      allTimes.push_back(lastEvaluationTime_);
      // time v2
      allTimesMeasured.push_back(timeSec);
      // cost
      double dist = sqrt( pow(planner_.Tgoal_.x - planner_.Tstart_.x, 2.0) + pow(planner_.Tgoal_.y - planner_.Tstart_.y, 2.0) + pow(planner_.Tgoal_.z - planner_.Tstart_.z, 2.0) );
      allCosts.push_back(planner_.lastCost_ / dist);
      // success
      allSuccess.push_back(true);
    } else {
      allTimes.push_back(1e10);
      allTimesMeasured.push_back(1e10);
      allCosts.push_back(1e10);
      allSuccess.push_back(false);
    }
  }

  // merge stats
  lastEvaluationTime_ = 0;
  lastEvaluationTimeMeasured_ = 0;
  lastEvaluationCost_ = 0;
  lastEvaluationSuccessRate_ = 0;
  unsigned int N = allTimes.size();
  for (unsigned int i = 0; i < N; i++) {
    lastEvaluationTime_ += allTimes[i] / (double)N;
    lastEvaluationTimeMeasured_ += allTimesMeasured[i] / (double)N;
    lastEvaluationCost_ += allCosts[i] / (double)N;
    lastEvaluationSuccessRate_ += (allSuccess[i] ? 1.0 : 0.0) / (double)N;
  }
}

