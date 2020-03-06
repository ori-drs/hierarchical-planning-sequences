#pragma once
#include "flp.h"
#include "StateSpace.h"
#include "RobotModel.h"
#include "FeasibilityChecker.h"
#include "StateTransitionCost.h"
#include "Environment.h"
#include "EnvironmentRecast.h"
#include "Viewer.h"

namespace arch
{


class VirtualStateSpaceUnit
{
public:
  VirtualStateSpaceUnit(const std::string& name) : name_(name) {}
  std::string getName() const { return name_; }
  bool operator==(const VirtualStateSpaceUnit& other) const { return name_ == other.getName(); }
  bool operator!=(const VirtualStateSpaceUnit& other) const { return !(*this == other); }
private:
  std::string name_;
};

struct VirtualStateSpace
{
  VirtualStateSpace() {}
  VirtualStateSpace(const std::vector<std::string>& names);
  std::vector<VirtualStateSpaceUnit> units;
  bool isContainedIn(const VirtualStateSpace& other) const;
  std::string to_string() const;
};

struct Model
{
  std::vector<std::string> statesAccepted;
  std::map<std::string, double> statesDefaultValue;
  bool isDefinedBy(const VirtualStateSpace& space) const;
  bool isPartialOf(const Model& model) const;
  std::vector<std::string> minus(const Model& model) const;
  std::string to_string() const;
};

struct Trajectory
{
  Trajectory(const std::vector< std::vector<double> >& v, const std::string& s) : vectors(v), spaceName(s) {}
  std::vector<State::Ptr> toStateVector(const StateSpace::Ptr& space) const;
  std::vector< std::vector<double> > vectors;
  std::string spaceName;
};

struct Planner
{
  Planner();
  std::vector<std::string> explode(const std::string& s, char delim) const;
  StateSpace::Ptr getStateSpace(const std::string& spaceName) const;
  RobotModel::Ptr getRobotModel(const std::string& spaceName, const StateSpace::Ptr& space, const std::vector<double>& plannerParameters) const;
  std::vector<FeasibilityChecker::Ptr> getFeasibilityCheckers(const RobotModel::Ptr& model, const StateSpace::Ptr& space, const Trajectory& refTrajectory, const std::vector<double>& plannerParameters) const;
  StateTransitionCost::Ptr getCost(const RobotModel::Ptr& model, const StateSpace::Ptr& space) const;
  std::vector< std::vector<double> > plan(const std::string& spaceName, const std::vector<double>& plannerParameters, const Trajectory& refTrajectory);
  void setRecast(bool useRecast);
  // properties
  std::map< std::string, std::vector<double> > lb_;
  std::map< std::string, std::vector<double> > ub_;
  std::map< std::string, std::vector<int> > nbins_;
  std::vector< std::pair<Model, RobotModel::Ptr> > models_;
  std::string plannerName_;
  std::string plannerCosts_;
  std::string plannerConstraints_;
  std::vector<std::string> plannerParameterNames_;
  std::vector<double> plannerParameterScales_;
  Environment::Ptr environment_;
  EnvironmentRecast::Ptr envRecast_;
  EnvironmentRecast::Params recastParams_;
  bool useRecastHeuristic_;
  flp::Transform Tstart_;
  flp::Transform Tgoal_;
  Viewer::Ptr viewer_;
  bool verbose_;
  double lastCost_;
};

class PlannerEvaluator
{
public:
  PlannerEvaluator();
  virtual void evaluate(const std::vector<double>& vector) = 0;
  virtual void save(const std::vector<double>& vector, const std::string& filename, const std::string& extension);
  virtual void saveLast(const std::string& filename, const std::string& extension);
  virtual void setDebug(bool debug);
  virtual void setEnvironment(const std::string& strEnvironment);
  virtual void setRecast(bool useRecast);
  virtual void setSeed(unsigned int seed) { seedEnvironmentSampling_ = seed; }
  virtual void generateRandomProblems(int numProblems);
  virtual std::vector<double> getLowerBounds() const { return lb_; }
  virtual std::vector<double> getUpperBounds() const { return ub_; }
  virtual unsigned int getDimension() const { return dimension_; }
  virtual double getLastEvaluationTime() const { return lastEvaluationTime_; }
  virtual double getLastEvaluationTimeMeasured() const { return lastEvaluationTimeMeasured_; }
  virtual double getLastEvaluationCost() const { return lastEvaluationCost_; }
  virtual double getLastEvaluationSuccessRate() const { return lastEvaluationSuccessRate_; }
  flp::Graph getGraph() const { return graph_; }
protected:
  flp::Graph graph_; // graph representation of a sequential planning architecture
  std::vector<flp::Graph> lastGraphDrawable_;
  arch::Planner planner_;
  unsigned int dimension_;
  std::vector<double> lb_;
  std::vector<double> ub_;
  double lastEvaluationTime_;
  double lastEvaluationTimeMeasured_;
  double lastEvaluationCost_;
  double lastEvaluationSuccessRate_;
  std::vector<flp::Transform> evaluatedStates_;
  std::vector<double> lbEnvironmentSampling_;
  std::vector<double> ubEnvironmentSampling_;
  unsigned int seedEnvironmentSampling_;
};

void getCombinations(const std::vector<bool>& vec, unsigned int idx, std::vector< std::vector<bool> >& out);
flp::Graph getAllValidSubspaces(const VirtualStateSpace& space, const std::vector<Model>& models, const std::vector<double>& defaultEdgeParameters);
std::vector< std::vector<double> > executeEdge(const flp::Graph& graph, int i, int j, Planner& planner);
int executePath(flp::Graph& graph, const std::vector<int>& path, Planner& planner);
bool findShortestPathOnline(const flp::Graph& graph, int start, int goal, Planner& planner, flp::Graph& graphUpdated, flp::Graph& graphDraw, std::vector<int>& graphPath);


}
