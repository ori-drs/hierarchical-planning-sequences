#pragma once
#include "Planner.h"
#include "RobotModel.h"
#include "EnvironmentRecast.h"

class PlannerRecast : public Planner
{
public:
  typedef boost::shared_ptr<PlannerRecast> Ptr;
  PlannerRecast(const EnvironmentRecast::Ptr& environment, const RobotModel::Ptr& model);

  virtual void setCost(const StateTransitionCost::Ptr& cost);
  virtual void setSubPlanner(const Planner::Ptr& subPlanner);
  virtual void plan(const State::Ptr& start, const State::Ptr& goal, std::vector<StateTransition>& transitions);
  virtual void getLastPlanVisitedStates(std::vector<State::Ptr>& states) const;
  virtual void getLastPlanVisitedGraph(std::vector<State::Ptr>& lineList) const;
  virtual void debugLastPlanVisitedGraph();

protected:
  EnvironmentRecast::Ptr environment_;
  RobotModel::Ptr model_;
};

