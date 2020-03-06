#pragma once
#include <boost/shared_ptr.hpp>
#include <random>
#include <vector>
#include <string>

struct Action
{
  typedef boost::shared_ptr<Action> Ptr;
  Action(const std::string& name) : name_(name) {}

  virtual bool isOption() const = 0;
  std::string name_;
};


struct ActionOption : public Action
{
  typedef boost::shared_ptr<ActionOption> Ptr;
  ActionOption(const std::string& name, const Action::Ptr& option, int optionId)
    : Action(name), option_(option), optionId_(optionId) {}

  virtual bool isOption() const { return true; }

  Action::Ptr option_;
  int optionId_;
};


struct ActionControl : public Action
{
  typedef boost::shared_ptr<ActionControl> Ptr;
  ActionControl(const std::string& name, const std::vector<double>& control)
    : Action(name), control_(control) {}

  virtual bool isOption() const { return false; }

  std::vector<double> control_;
};

