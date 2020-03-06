#include "PlannerRecast.h"

PlannerRecast::PlannerRecast(const EnvironmentRecast::Ptr& environment, const RobotModel::Ptr& model)
  : environment_(environment)
  , model_(model)
{

}

void PlannerRecast::setCost(const StateTransitionCost::Ptr& cost)
{

}

void PlannerRecast::setSubPlanner(const Planner::Ptr& subPlanner)
{

}

void PlannerRecast::plan(const State::Ptr& start, const State::Ptr& goal, std::vector<StateTransition>& transitions)
{
  // convert to points
  flp::Transform T1, T2;
  if (!start->space_->getTransform(start, T1)) return;
  if (!start->space_->getTransform(goal, T2)) return;
  pcl::PointXYZ p1; p1.x = T1.x; p1.y = T1.y; p1.z = T1.z;
  pcl::PointXYZ p2; p2.x = T2.x; p2.y = T2.y; p2.z = T2.z;

  // query recast
  std::vector<pcl::PointXYZ> traj;
  if (!environment_->recast_.query(p1, p2, traj)) return;

  // convert to states and transitions
  State::Ptr prevS = start;
  flp::Transform prevT = T1;
  for (unsigned int i = 1; i < traj.size(); i++) {
    flp::Transform T = prevT;
    T.x = traj[i].x;
    T.y = traj[i].y;
    T.z = traj[i].z;
    State::Ptr s = start->clone();
    if (!s->space_->setTransform(s, T)) {
      transitions.clear();
      printf("can't set transform on this state space...\n");
      return;
    }
    // test
    //flp::Transform test;
    //s->space_->getTransform(s, test);
    //printf("target: %f %f %f \n", T.x, T.y, T.z);
    //printf("real  : %f %f %f \n", test.x, test.y, test.z);
    //printf("-------\n");
    StateTransition t;
    t.state = prevS;
    t.newState = s;
    t.cost = s->space_->distance(prevS, s);
    transitions.push_back(t);
    prevS = s;
    prevT = T;
  }
}

void PlannerRecast::getLastPlanVisitedStates(std::vector<State::Ptr>& states) const
{

}

void PlannerRecast::getLastPlanVisitedGraph(std::vector<State::Ptr>& lineList) const
{

}

void PlannerRecast::debugLastPlanVisitedGraph()
{

}

