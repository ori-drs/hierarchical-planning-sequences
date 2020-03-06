#pragma once
#include "Action.h"

struct ActionSpace
{
  typedef boost::shared_ptr<ActionSpace> Ptr;
  ActionSpace(const std::string& name) : name_(name) {}

  virtual bool isOption() const = 0;
  virtual unsigned int getDimension() const = 0;
  virtual void getBounds(std::vector<double>& lb, std::vector<double>& ub) const;
  virtual std::vector<ActionSpace::Ptr> getControlSpaces() const = 0;
  virtual Action::Ptr getControlZero() const;
  virtual Action::Ptr sampleControlUniform(std::mt19937& generator) const;
  virtual std::vector<Action::Ptr> sampleControlUniform(unsigned int N, std::mt19937& generator) const;
  virtual Action::Ptr fromNormalizedVector(const std::vector<double>& vec) const = 0;
  virtual std::vector<double> toNormalizedVector(const Action::Ptr& action) const = 0;
  virtual std::string toString(const Action::Ptr& action) const;

  std::string name_;
};


struct ActionSpaceOption : public ActionSpace
{
  typedef boost::shared_ptr<ActionSpaceOption> Ptr;
  ActionSpaceOption(const std::string& name, const std::vector<ActionSpace::Ptr>& options)
    : ActionSpace(name), options_(options) { /*TODO throw exception if size zero*/ }

  virtual bool isOption() const { return true; }
  virtual unsigned int getDimension() const;
  virtual std::vector<ActionSpace::Ptr> getControlSpaces() const;
  virtual ActionSpace::Ptr sampleControlSpacesUniform(std::mt19937& generator) const;
  virtual std::vector<ActionSpace::Ptr> sampleControlSpacesUniform(unsigned int N, std::mt19937& generator) const;
  virtual Action::Ptr fromNormalizedVector(const std::vector<double>& vec) const;
  virtual std::vector<double> toNormalizedVector(const Action::Ptr& action) const;
  virtual std::vector<double> toNormalizedVector(const ActionOption::Ptr& action) const;

  std::vector<ActionSpace::Ptr> options_;
};


struct ActionSpaceControl : public ActionSpace
{
  typedef boost::shared_ptr<ActionSpaceControl> Ptr;
  ActionSpaceControl(const std::string& name, const std::vector<double>& lb, const std::vector<double>& ub, const std::vector<int>& nbins)
    : ActionSpace(name), lb_(lb), ub_(ub), nbins_(nbins) {}

  virtual bool isOption() const { return false; }
  virtual unsigned int getDimension() const { return lb_.size(); }
  virtual std::vector<ActionSpace::Ptr> getControlSpaces() const { return std::vector<ActionSpace::Ptr>(); }
  virtual Action::Ptr fromNormalizedVector(const std::vector<double>& vec) const;
  virtual std::vector<double> toNormalizedVector(const Action::Ptr& action) const;
  virtual std::vector<double> toNormalizedVector(const ActionControl::Ptr& action) const;

  std::vector<double> lb_;
  std::vector<double> ub_;
  std::vector<int> nbins_;
};

