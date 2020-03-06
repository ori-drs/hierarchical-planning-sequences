#include "ArchitectureSearchStarleth.h"

int main(int argc, char *argv[])
{
  // starleth
  arch::PlannerEvaluatorStarleth arch;
  arch.setDebug(true);

  // my architecture
  arch.save(arch.getUpperBounds(), "test-starleth-arch.png", "png");

  // eth paper architecture
  arch.setParamsBaseline("ethSingleMaxtimePerSpace");
  arch.save(arch.getUpperBounds(), "test-starleth-arch-eth.png", "png");

  // evaluate
  arch.evaluate(arch.getUpperBounds());
  return 0;
}

