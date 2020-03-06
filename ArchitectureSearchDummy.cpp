#include "ArchitectureSearchDummy.h"
#include "RobotModelFixedPrimitives.h"
#include "EnvironmentGridMap.h"
#include <grid_map_core/grid_map_core.hpp>
#include <boost/timer/timer.hpp>

arch::PlannerEvaluatorDummy::PlannerEvaluatorDummy()
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

  // architecture graph: two edge parameters (one for each optimization objective)
  std::vector<double> defaultEdgeParameters(2,0);
  graph_ = arch::getAllValidSubspaces(space, models, defaultEdgeParameters);

  // only optimize edge costs, not parameters
  std::vector<double> vec = graph_.getEdgesVector();
  dimension_ = vec.size();
  lb_ = std::vector<double>(dimension_, 0);
  ub_ = std::vector<double>(dimension_, 100);

  // set random parameters in the beginning. the dummy problem is to find edge costs that lead to pareto-optimal sum/min of parameters over shortest-cost-path
  std::mt19937 generator;
  std::uniform_real_distribution<double> dis(0, 100);
  for (unsigned int i = 0; i < vec.size(); i++)
    vec[i] = dis(generator);
  graph_.setEdgesVector(vec);
}

void arch::PlannerEvaluatorDummy::evaluate(const std::vector<double>& vector)
{
  // only optimize edge costs, not parameters
  //std::vector<double> newvector = graph_.getEdgesVector();
  //for (unsigned int i = 0; i < vector.size(); i++) {
  //  unsigned int idx = i * 3;
  //  newvector[idx] = vector[i];
  //}
  std::vector<double> newvector = vector;
  // solve planning problem using our graph-based architecture
  lastGraphDrawable_.clear();
  lastGraphDrawable_.push_back(graph_);
  lastGraphDrawable_[0].setEdgesVector(newvector);
  std::vector<int> graphPath = lastGraphDrawable_[0].getShortestPath(0, 1);
  if (graphPath.size() == 0) {
    lastEvaluationSuccessRate_ = 0;
    lastEvaluationTime_ = 1e10;
    lastEvaluationCost_ = 1e10;
    return;
  }
  lastEvaluationSuccessRate_ = 1;
  // option 1: sum of params
  //lastEvaluationTime_ = 0;
  //lastEvaluationCost_ = 0;
  // option 2: min of params
  lastEvaluationTime_ = 1e10;
  lastEvaluationCost_ = 1e10;
  int iii = 0;
  int jjj = 0;
  for (unsigned int i = 0; i < graphPath.size()-1; i++) {
    std::vector<double> params = lastGraphDrawable_[0].getEdgeParameters(graphPath[i], graphPath[i+1]);
    // option 1: sum of params
    //lastEvaluationTime_ += params[0];
    //lastEvaluationCost_ += params[1];
    // option 2: min of params
    lastEvaluationTime_ = std::min(lastEvaluationTime_, params[0]);
    lastEvaluationCost_ = std::min(lastEvaluationCost_, params[1]);
    iii += i;
    jjj += graphPath[i];
  }
  //lastEvaluationTime_ = pow(50-lastEvaluationTime_, 2.0) * lastEvaluationCost_ + 10 + sin(lastEvaluationCost_) * 1000 * fabs(iii-1);
  //lastEvaluationCost_ = pow((lastEvaluationCost_-30), 2.0) + 50 + cos(lastEvaluationCost_) * 1000 * fabs(iii-1);
  lastEvaluationTime_ = pow(50-lastEvaluationTime_, 2.0) * lastEvaluationCost_ + 10 + sin(lastEvaluationCost_) * 1000 * fabs(-jjj+4);
  lastEvaluationCost_ = pow((lastEvaluationCost_-30), 2.0) + 50 + cos(lastEvaluationCost_) * 1000 * fabs(iii-1);
  lastGraphDrawable_[0].setColorPath(graphPath, "green");
}

void arch::PlannerEvaluatorDummy::save(const std::vector<double>& vector, const std::string& filename, const std::string& extension)
{
  // only optimize edge costs, not parameters
  std::vector<double> newvector = graph_.getEdgesVector();
  for (unsigned int i = 0; i < vector.size(); i++) {
    unsigned int idx = i * 3;
    newvector[idx] = vector[i];
  }
  PlannerEvaluator::save(newvector, filename, extension);
}

