#pragma once
#include "ArchitectureSearch.h"

namespace arch
{


class PlannerEvaluatorDummy : public PlannerEvaluator
{
public:
  PlannerEvaluatorDummy();
  virtual void evaluate(const std::vector<double>& vector);
  virtual void save(const std::vector<double>& vector, const std::string& filename, const std::string& extension);
};


}
