#pragma once
#include "ArchitectureSearch.h"

namespace arch
{


class PlannerEvaluatorAnymal : public PlannerEvaluator
{
public:
  PlannerEvaluatorAnymal();
  virtual void evaluate(const std::vector<double>& vector);
  virtual void setParamsBaseline(const std::string& baseline);
};


}
