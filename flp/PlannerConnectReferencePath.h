#pragma once
#include "Planner.h"
#include "RobotModel.h"

class PlannerConnectReferencePath : public Planner
{
public:
  typedef boost::shared_ptr<PlannerConnectReferencePath> Ptr;
  PlannerConnectReferencePath(const std::vector<State::Ptr>& states);

  virtual void setCost(const StateTransitionCost::Ptr& cost);
  virtual void setSubPlanner(const Planner::Ptr& subPlanner);
  virtual void plan(const State::Ptr& start, const State::Ptr& goal, std::vector<StateTransition>& transitions);
  virtual void getLastPlanVisitedStates(std::vector<State::Ptr>& states) const;
  virtual void getLastPlanVisitedGraph(std::vector<State::Ptr>& lineList) const;
  virtual void debugLastPlanVisitedGraph();

protected:
  std::vector<State::Ptr> states_;
};

