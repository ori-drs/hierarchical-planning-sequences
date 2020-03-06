#pragma once
#include "State.h"
#include "StateTransition.h"
#include "StateTransitionCost.h"
#include "Viewer.h"
#include "RobotModel.h"
#include <boost/shared_ptr.hpp>

class Planner
{
public:
  typedef boost::shared_ptr<Planner> Ptr;
  Planner() : maxTime_(30.0), optimizeSolution_(true), projectToEnvironment_(false) {}
  virtual void setCost(const StateTransitionCost::Ptr& cost) = 0;
  virtual void setHeuristicPathLength(const Planner::Ptr& heurPathLengthPlanner) { heurPathLengthPlanner_ = heurPathLengthPlanner; }
  virtual void setSubPlanner(const Planner::Ptr& subPlanner) = 0;
  virtual void plan(const State::Ptr& start, const State::Ptr& goal, std::vector<StateTransition>& transitions) = 0;
  virtual void getLastPlanVisitedStates(std::vector<State::Ptr>& states) const = 0;
  virtual void getLastPlanVisitedGraph(std::vector<State::Ptr>& lineList) const = 0;
  virtual void debugLastPlanVisitedGraph() = 0;

  virtual void attachViewer(const Viewer::Ptr& viewer, const RobotModel::Ptr& model) {
    viewer_ = viewer;
    viewerModel_ = model;
  }
  virtual void detachViewer() {
    viewer_.reset();
    viewerModel_.reset();
  }
  virtual Viewer::Ptr getViewer() {
    return viewer_;
  }
  virtual RobotModel::Ptr getViewerModel() {
    return viewerModel_;
  }

  double maxTime_;
  bool optimizeSolution_;
  bool projectToEnvironment_;

protected:
  Viewer::Ptr viewer_;
  RobotModel::Ptr viewerModel_;
  Planner::Ptr heurPathLengthPlanner_;
};

