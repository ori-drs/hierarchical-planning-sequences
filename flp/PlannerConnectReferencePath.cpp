#include "PlannerConnectReferencePath.h"

PlannerConnectReferencePath::PlannerConnectReferencePath(const std::vector<State::Ptr>& states)
  : states_(states)
{

}

void PlannerConnectReferencePath::setCost(const StateTransitionCost::Ptr& cost)
{

}

void PlannerConnectReferencePath::setSubPlanner(const Planner::Ptr& subPlanner)
{

}

void PlannerConnectReferencePath::plan(const State::Ptr& start, const State::Ptr& goal, std::vector<StateTransition>& transitions)
{
  // connect start to reference-path (1)
  int sIdx = -1;
  double sDist = std::numeric_limits<double>::infinity();
  State::Ptr sproj = states_[0]->space_->projectFrom(start, start->space_); // TODO: check whether all states[i] are in the same space
  if (!sproj) return;
  for (unsigned int i = 0; i < states_.size(); i++) {
    double dist = sproj->space_->distance(sproj, states_[i]);
    if (dist < sDist) {
      sDist = dist;
      sIdx = i;
    }
  }
  if (sIdx < 0) return;

  // connect goal to reference-path (2)
  int gIdx = -1;
  double gDist = std::numeric_limits<double>::infinity();
  State::Ptr gproj = states_[0]->space_->projectFrom(goal, goal->space_); // TODO: check whether all states[i] are in the same space
  if (!gproj) return;
  for (unsigned int i = 0; i < states_.size(); i++) {
    double dist = gproj->space_->distance(gproj, states_[i]);
    if (dist < gDist) {
      gDist = dist;
      gIdx = i;
    }
  }
  if (gIdx < 0) return;

  // use reference path to connect (1) to (2)
  State::Ptr prevS = sproj;
  if (sIdx <= gIdx) {
    for (unsigned int i = sIdx; i <= gIdx; i++) {
      StateTransition t;
      t.state = prevS->clone();
      t.newState = states_[i];
      t.cost = states_[i]->space_->distance(t.state, t.newState); // TODO: check whether all states[i] are in the same space
      transitions.push_back(t);
      prevS = t.newState;
    }
  } else {
    for (unsigned int i = sIdx; i >= gIdx; i--) {
      StateTransition t;
      t.state = prevS->clone();
      t.newState = states_[i];
      t.cost = states_[i]->space_->distance(t.state, t.newState); // TODO: check whether all states[i] are in the same space
      transitions.push_back(t);
      prevS = t.newState;
    }
  }
  // add goal state
  if (transitions.size() > 0) {
    StateTransition t;
    t.state = prevS->clone();
    t.newState = goal;
    t.cost = gDist;
    transitions.push_back(t);
  }
}

void PlannerConnectReferencePath::getLastPlanVisitedStates(std::vector<State::Ptr>& states) const
{

}

void PlannerConnectReferencePath::getLastPlanVisitedGraph(std::vector<State::Ptr>& lineList) const
{

}

void PlannerConnectReferencePath::debugLastPlanVisitedGraph()
{

}

