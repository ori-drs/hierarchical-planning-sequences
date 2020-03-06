#pragma once
#include "ArchitectureSearch.h"

namespace arch
{


class PlannerEvaluatorStarleth : public PlannerEvaluator
{
public:
  PlannerEvaluatorStarleth();
  virtual void evaluate(const std::vector<double>& vector);
  virtual void setParamsBaseline(const std::string& baseline);
protected:
  std::string baseline_;
  double inscribedRadius_;
};


}
