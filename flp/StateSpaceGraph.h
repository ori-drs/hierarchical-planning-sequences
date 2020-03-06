#pragma once
#include "flp.h"
#include "State.h"
#include "StateSpace.h"

struct StateGraph : public State
{
  typedef boost::shared_ptr<StateGraph> Ptr;
  StateGraph(int value) : value_(value) {
  }
  StateGraph(int value, const StateSpaceConstPtr& space) : State(space), value_(value) {
  }
  virtual State::Ptr clone() const {
    return StateGraph::Ptr( new StateGraph(value_, space_) );
  }
  int value_;
};

class StateSpaceGraph : public StateSpace
{
public:
  typedef boost::shared_ptr<StateSpaceGraph> Ptr;
  typedef boost::shared_ptr<StateSpaceGraph const> ConstPtr;
  StateSpaceGraph(const flp::Graph& graph) : StateSpace(std::vector<StateSpace::Ptr>(), std::vector<double>()), graph_(graph) {
    // compute distances etc
    graph_.update();
    // get (and store) embedding by multi-dimensional scaling (check isomap code) TODO
    // get lb, ub, nbins
    int V = graph_.getNumberOfNodes();
    lb_ = std::vector<double>(1, 1.0);
    ub_ = std::vector<double>(1, V+0.9999);
    nbins_ = std::vector<int>(1, V);
    dim_ = 1;
  }
  virtual State::Ptr applyNormalizedChange(const State::Ptr& state, const std::vector<double>& control) const {
    // control will be in [0,1]
    std::vector<State::Ptr> nei = getNeighbors(state);
    int idx = nei.size() * control[0];
    if (idx == nei.size())
      idx = nei.size() - 1;
    return nei[idx];
  }
  virtual std::vector<double> getNormalizedChange(const State::Ptr& state1, const State::Ptr& state2) const {
    // get neighbor which is closest to state2
    std::vector<State::Ptr> nei = getNeighbors(state1);
    int imin = 0;
    double mindist = std::numeric_limits<double>::infinity();
    for (unsigned int i = 0; i < nei.size(); i++) {
      const StateGraph& s1 = *boost::static_pointer_cast<StateGraph>(nei[i]);
      const StateGraph& s2 = *boost::static_pointer_cast<StateGraph>(state2);
      double dist = graph_.getDistance(s1.value_, s2.value_);
      if (dist < mindist) {
        mindist = dist;
        imin = i;
      }
    }
    // control will be in [0,1]
    std::vector<double> control(1);
    control[0] = (double)imin / (double)nei.size();
    assert((int)(control[0] * nei.size()) == imin);
    return control;
  }
  virtual State::Ptr interpolate(const State::Ptr& state1, const State::Ptr& state2, double fraction) const {
    const StateGraph& s1 = *boost::static_pointer_cast<StateGraph>(state1);
    const StateGraph& s2 = *boost::static_pointer_cast<StateGraph>(state2);
    std::vector<int> path = graph_.getShortestPath(s1.value_, s2.value_);
    int idx = fraction * path.size();
    if (idx >= path.size())
      idx = path.size() - 1;
    return State::Ptr( new StateGraph(path[idx], shared_from_this()) );
  }
  virtual std::vector<State::Ptr> getNeighbors(const State::Ptr& state) const {
    const StateGraph& s = *boost::static_pointer_cast<StateGraph>(state);
    std::vector<int> nodes = graph_.getNeighbors(s.value_);
    std::vector<State::Ptr> nei(nodes.size());
    for (unsigned int i = 0; i < nodes.size(); i++)
      nei[i] = State::Ptr( new StateGraph(nodes[i], shared_from_this()) );
    return nei;
  }
  virtual State::Ptr getStateZero() const {
    return State::Ptr( new StateGraph(lb_[0], shared_from_this()) );
  }
  virtual State::Ptr sampleUniform(std::mt19937& generator) const {
    std::uniform_int_distribution<int> dis((int)lb_[0], (int)ub_[0]);
    return StateGraph::Ptr( new StateGraph(dis(generator), shared_from_this()) );
  }
  virtual State::Ptr projectToFeasible(const State::Ptr& state) const {
    const StateGraph::Ptr& s = boost::static_pointer_cast<StateGraph>(state);
    int newvalue = std::max(std::min(s->value_,(int)ub_[0]),(int)lb_[0]);
    return StateGraph::Ptr( new StateGraph(newvalue, shared_from_this()) );
  }
  virtual State::Ptr fromVector(const std::vector<double>& vec) const {
    return State::Ptr( new StateGraph(vec[0], shared_from_this()) );
  }
  virtual std::vector<double> toVector(const State::Ptr& state) const {
    return std::vector<double>(1, boost::static_pointer_cast<StateGraph>(state)->value_);
  }
  virtual double distance(const State::Ptr& state1, const State::Ptr& state2) const {
    const StateGraph& s1 = *boost::static_pointer_cast<StateGraph>(state1);
    const StateGraph& s2 = *boost::static_pointer_cast<StateGraph>(state2);
    return graph_.getDistance(s1.value_, s2.value_);
  }
protected:
  flp::Graph graph_;
};

