#include "ActionSpace.h"
#include <sstream>

//----------------------------------------------------------------------------------
// action space

void ActionSpace::getBounds(std::vector<double>& lb, std::vector<double>& ub) const {
  unsigned int dim = getDimension();
  lb.clear(); lb.resize(dim, 0.0);
  ub.clear(); ub.resize(dim, 1.0);
}

Action::Ptr ActionSpace::getControlZero() const {
  std::vector<double> vec(getDimension(),0.0);
  return fromNormalizedVector(vec);
}

Action::Ptr ActionSpace::sampleControlUniform(std::mt19937& generator) const {
  std::uniform_real_distribution<double> dis(0, 1);
  std::vector<double> vec(getDimension());
  for (unsigned int i = 0; i < vec.size(); i++)
    vec[i] = dis(generator);
  return fromNormalizedVector(vec);
}

std::vector<Action::Ptr> ActionSpace::sampleControlUniform(unsigned int N, std::mt19937& generator) const {
  std::vector<Action::Ptr> actions;
  for (unsigned int i = 0; i < N; i++) {
    actions.push_back(sampleControlUniform(generator));
  }
  return actions;
}

std::string ActionSpace::toString(const Action::Ptr& action) const {
  std::vector<double> vec = toNormalizedVector(action);
  std::stringstream ss;
  for (unsigned int i = 0; i < vec.size(); i++)
    ss << vec[i] << " ";
  return ss.str();
}


//----------------------------------------------------------------------------------
// action space option

unsigned int ActionSpaceOption::getDimension() const { // TODO precompute once and then return result
  unsigned int maxDim = 0;
  for (unsigned int i = 0; i < options_.size(); i++) {
    unsigned int dim = options_[i]->getDimension();
    if (dim > maxDim)
      maxDim = dim;
  }
  return 1 + maxDim; // 1 dimension to choose between options, plus maxDim to represent the rest of the action
}

std::vector<ActionSpace::Ptr> ActionSpaceOption::getControlSpaces() const {
  std::vector<ActionSpace::Ptr> spaces;
  for (unsigned int i = 0; i < options_.size(); i++) {
    if (options_[i]->isOption()) {
      std::vector<ActionSpace::Ptr> s = options_[i]->getControlSpaces();
      spaces.insert(spaces.end(), s.begin(), s.end());
    } else {
      spaces.push_back(options_[i]);
    }
  }
  return spaces;
}

ActionSpace::Ptr ActionSpaceOption::sampleControlSpacesUniform(std::mt19937& generator) const {
  std::vector<ActionSpace::Ptr> spaces = getControlSpaces();
  std::uniform_int_distribution<unsigned int> dis(0, spaces.size()-1);
  return spaces[dis(generator)];
}

std::vector<ActionSpace::Ptr> ActionSpaceOption::sampleControlSpacesUniform(unsigned int N, std::mt19937& generator) const {
  std::vector<ActionSpace::Ptr> spaces = getControlSpaces();
  if (N >= spaces.size())
    return spaces;
  std::shuffle(spaces.begin(), spaces.end(), generator);
  spaces.resize(N);
  return spaces;
}

Action::Ptr ActionSpaceOption::fromNormalizedVector(const std::vector<double>& vec) const {
  unsigned int optionId = vec[0] * options_.size();
  std::vector<double> optionVec = vec;
  optionVec.erase(optionVec.begin());
  Action::Ptr option = options_[optionId]->fromNormalizedVector(optionVec);
  return ActionOption::Ptr(new ActionOption(name_, option, optionId));
}

std::vector<double> ActionSpaceOption::toNormalizedVector(const Action::Ptr& action) const {
  return toNormalizedVector(boost::static_pointer_cast<ActionOption>(action));
}

std::vector<double> ActionSpaceOption::toNormalizedVector(const ActionOption::Ptr& action) const {
  std::vector<double> vec = options_[action->optionId_]->toNormalizedVector(action->option_);
  vec.insert(vec.begin(), (double)action->optionId_ / (double)options_.size());
}


//----------------------------------------------------------------------------------
// action space control

Action::Ptr ActionSpaceControl::fromNormalizedVector(const std::vector<double>& vec) const {
  std::vector<double> control(getDimension());
  for (unsigned int i = 0; i < vec.size(); i++)
    control[i] = vec[i] * (ub_[i] - lb_[i]) + lb_[i];
  return ActionControl::Ptr(new ActionControl(name_, control));
}

std::vector<double> ActionSpaceControl::toNormalizedVector(const Action::Ptr& action) const {
  return toNormalizedVector(boost::static_pointer_cast<ActionControl>(action));
}

std::vector<double> ActionSpaceControl::toNormalizedVector(const ActionControl::Ptr& action) const {
  std::vector<double> vec(getDimension());
  for (unsigned int i = 0; i < vec.size(); i++)
    vec[i] = (action->control_[i] - lb_[i]) / (ub_[i] - lb_[i]);
  return vec;
}

