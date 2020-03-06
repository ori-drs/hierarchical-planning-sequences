#include "StateTransitionCost.h"
#include "StateSpaceXYZTM.h"
#include "StateSpaceSwitchContacts.h"
#include <sstream>

//---------------------------------------------------------------------------
// state statistics collector

void StateStats::compute(const Environment::Ptr& environment, const State::Ptr& statein, const RobotModel::Ptr& model)
{
  state = statein;

  // check mode
  state->space_->getMode(state, mode);
  state->space_->getMode(state, modeVec);

  // check collision
  std::vector<flp::Primitive> primitivesCollision = model->getPrimitivesCollision(state);
  if (environment->getCollision(primitivesCollision))
    baseDistanceToCollision = 0.0;
  else
    baseDistanceToCollision = 1.0; // TODO

  // check distance to nearest cliff
  baseDistanceToCliff = 0.0; // TODO

  // check contact and traversability
  std::vector<flp::Primitive> primitivesContact = model->getPrimitivesContact(state);
  legCollision.reserve(primitivesContact.size());
  legRoughness.reserve(primitivesContact.size());
  legTraversability.reserve(primitivesContact.size());
  legSupportRatio.reserve(primitivesContact.size());
  legSupportRatioOfPointsWithinRealisticSlope.reserve(primitivesContact.size());

  for (unsigned int i = 0; i < primitivesContact.size(); i++) {
    const double centerZ = primitivesContact[i].sphere.center(2);
    double maxPtDZ = primitivesContact[i].sphere.radius;
    if (boost::dynamic_pointer_cast<StateSpaceSwitchContacts const>(state->space_))
      maxPtDZ *= 0.1; //TODO: use slope property
    bool collision = environment->getCollision(primitivesContact[i], Environment::CollisionTypeAbove);
    std::vector<flp::Point> pts = environment->getPointsInside(primitivesContact[i]);
    double npts = 0;
    double nptsWithinRealisticSlope = 0;
    flp::Stats statsTra, statsRou;
    for (unsigned int j = 0; j < pts.size(); j++) {
      const flp::Point& pt = pts[j];
      statsRou.update(pt.roughness < 0 ? 0 : pt.roughness); //TODO: remove cap. tmp check
      statsTra.update(pt.traversability < 0 ? 1 : pt.traversability); //TODO: remove cap. tmp check
      if (fabs(pt.p(2) - centerZ) <= maxPtDZ) nptsWithinRealisticSlope++; //TODO: use slope property
      npts++;
    }
    double expectedPts = primitivesContact[i].expectedNumberOfPoints(environment->getSpatialResolution());
    double ratio = npts / expectedPts;
    double ratioOfPointsWithinRealisticSlope = nptsWithinRealisticSlope / expectedPts;
    ratio = std::max(std::min(ratio, 1.0), 0.0);
    ratioOfPointsWithinRealisticSlope = std::max(std::min(ratioOfPointsWithinRealisticSlope, 1.0), 0.0);
    legCollision.push_back(collision);
    legSupportRatio.push_back(ratio);
    legSupportRatioOfPointsWithinRealisticSlope.push_back(ratioOfPointsWithinRealisticSlope);
    legRoughness.push_back(statsRou);
    legTraversability.push_back(statsTra);
  }

  // check limb distances: LF, LH, RF, RH
  contacts.clear();
  if (state->space_->getContacts(state, contacts) && contacts.size() == 4) {
    feetDiagonal1 = (contacts[0]-contacts[3]).norm();
    feetDiagonal2 = (contacts[1]-contacts[2]).norm();
    feetForeHind1 = (contacts[0]-contacts[1]).norm();
    feetForeHind2 = (contacts[2]-contacts[3]).norm();
    feetLeftRight1 =(contacts[0]-contacts[2]).norm();
    feetLeftRight2 =(contacts[1]-contacts[3]).norm();
    feetForeHind1z = fabs(contacts[0](2) - contacts[1](2));
    feetForeHind2z = fabs(contacts[2](2) - contacts[3](2));
  }
}

std::ostream& operator<<(std::ostream& ss, const StateStats& stats)
{
  ss << stats.state->space_->toString(stats.state) << "\n";
  ss << "  mode: " << stats.mode << "\n";
  ss << "  modeVec: " << stats.modeVec.transpose() << "\n";
  ss << "  baseDistanceToCollision: " << stats.baseDistanceToCollision << "\n";
  ss << "  baseDistanceToCliff: " << stats.baseDistanceToCliff << "\n";

  ss << "  legCollision: \n";
  for (unsigned int i = 0; i < stats.legCollision.size(); i++)
    ss << "    legCollision[" << i << "]: " << stats.legCollision[i] << "\n";

  ss << "  legRoughness: \n";
  for (unsigned int i = 0; i < stats.legRoughness.size(); i++)
    ss << "    legRoughness[" << i << "]: " << stats.legRoughness[i] << "\n";

  ss << "  legTraversability: \n";
  for (unsigned int i = 0; i < stats.legTraversability.size(); i++)
    ss << "    legTraversability[" << i << "]: " << stats.legTraversability[i] << "\n";

  ss << "  legSupportRatio: \n";
  for (unsigned int i = 0; i < stats.legSupportRatio.size(); i++)
    ss << "    legSupportRatio[" << i << "]: " << stats.legSupportRatio[i] << "\n";

  ss << "  legSupportRatioOfPointsWithinRealisticSlope: \n";
  for (unsigned int i = 0; i < stats.legSupportRatioOfPointsWithinRealisticSlope.size(); i++)
    ss << "    legSupportRatioOfPointsWithinRealisticSlope[" << i << "]: " << stats.legSupportRatioOfPointsWithinRealisticSlope[i] << "\n";

  ss << "  leg distances: \n";
  ss << "    feetDiagonal1:  " << stats.feetDiagonal1 << "\n";
  ss << "    feetDiagonal2:  " << stats.feetDiagonal2 << "\n";
  ss << "    feetForeHind1:  " << stats.feetForeHind1 << "\n";
  ss << "    feetForeHind2:  " << stats.feetForeHind2 << "\n";
  ss << "    feetLeftRight1: " << stats.feetLeftRight1 << "\n";
  ss << "    feetLeftRight2: " << stats.feetLeftRight2 << "\n";
  ss << "    feetForeHind1z: " << stats.feetForeHind1z << "\n";
  ss << "    feetForeHind2z: " << stats.feetForeHind2z << "\n";

  return ss;
}


//---------------------------------------------------------------------------
// general cost

StateTransitionCost::StateTransitionCost(const RobotModel::Ptr& model, bool projectToEnvironmentBeforeComputingCost)
  : failure_(std::numeric_limits<double>::infinity())
  , model_(model)
  , projectToEnvironment_(projectToEnvironmentBeforeComputingCost)
{

}

double StateTransitionCost::evaluate(const Environment::Ptr& environment, const State::Ptr& state) const
{
  if (!state) return failure_;
  // TODO: environment projection
  StateStats stats;
  stats.compute(environment, state, model_);
  return evaluate(environment, stats);
}

double StateTransitionCost::evaluate(const Environment::Ptr& environment, StateTransition& transition) const
{
  double costPerDistance;
  return evaluate(environment, transition, costPerDistance);
}

double StateTransitionCost::evaluate(const Environment::Ptr& environment, StateTransition& transition, double& costPerDistance) const
{
  StateStats stats1, stats2;
  stats1.compute(environment, transition.state, model_);
  stats2.compute(environment, transition.newState, model_);
  return evaluate(environment, stats1, stats2, costPerDistance);
}

double StateTransitionCost::evaluate(const Environment::Ptr& environment, const StateStats& stats1, const StateStats& stats2) const
{
  double costPerDistance;
  return evaluate(environment, stats1, stats2, costPerDistance);
}

double StateTransitionCost::evaluate(const Environment::Ptr& environment, const StateStats& stats1, const StateStats& stats2, double& costPerDistance) const
{
  double cost1 = evaluate(environment, stats1);
  double cost2 = evaluate(environment, stats2);
  costPerDistance = (cost1 + cost2)/2; // state cost integral (as in OMPL)
  return costPerDistance * stats1.state->space_->distance(stats1.state, stats2.state);
}


//---------------------------------------------------------------------------
// multi-objective methods

double StateTransitionCostMultiObjective::evaluate(const Environment::Ptr& environment, const StateStats& stats1, const StateStats& stats2, double& costPerDistance) const
{
  printf("WARNING: evaluate(state1, state2) not implemented on this cost function.\n");
  return failure_;
}


//---------------------------------------------------------------------------
// lexicographic method for multiobjective optimization (least to most important)

StateTransitionCostLexicographic::StateTransitionCostLexicographic(const RobotModel::Ptr& model, bool projectToEnvironmentBeforeComputingCost, const std::vector<StateTransitionCost::Ptr>& costs, const std::vector<double>& epsilon)
  : StateTransitionCostMultiObjective(model, projectToEnvironmentBeforeComputingCost, costs)
  , epsilon_(epsilon) // eps > 0 tolerance/resolution of each cost, i.e. value of the cost at which we become indifferent because of noise etc (used to convert costs to int)
  , lowerBoundStateCost_(0)
  , lowerBoundCostPerDistance_(0)
  , upperBoundStateCost_(0)
  , upperBoundCostPerDistance_(0)
{
  // scaling for defacto lexicographic optimization...
  precedingDigits_.push_back(0);
  for (unsigned int i = 0; i < costs_.size(); i++) {
    double maxCost = costs_[i]->getUpperBoundStateCost();
    int maxDigits = (int)log10( 1 + maxCost / epsilon_[i] ) + 1;
    if (i < costs_.size() - 1) {
      precedingDigits_.push_back(precedingDigits_.back() + maxDigits);
    }
  }

  //std::cout << "Pre-computing lexicographic cost bounds:\n";
  //for (unsigned int i = 0; i < precedingDigits_.size(); i++) {
  //  printf("  preceding digits[%d] = %d \n", i, precedingDigits_[i]);
  //}

  // pre-compute bounds
  double c;
  for (unsigned int i = 0; i < costs_.size(); i++) {
    //printf("  costs[%d].epsilon = %f \n", i, epsilon_[i]);
    //printf("  costs[%d].weight  = %f \n", i, 1.0 / epsilon_[i] * pow(10.0, precedingDigits_[i]));

    c = costs_[i]->getLowerBoundStateCost();
    lowerBoundStateCost_ += (double)((int)(1 + c / epsilon_[i])) * pow(10.0, precedingDigits_[i]);
    //printf("  costs[%d].lower = %f \n", i, c);
    //printf("  costs[%d].lowerC= %f \n", i, (double)((int)(1 + c / epsilon_[i])) * pow(10.0, precedingDigits_[i]));

    c = costs_[i]->getLowerBoundCostPerDistance();
    lowerBoundCostPerDistance_ += (double)((int)(1 + c / epsilon_[i])) * pow(10.0, precedingDigits_[i]);

    c = costs_[i]->getUpperBoundStateCost();
    upperBoundStateCost_ += (double)((int)(1 + c / epsilon_[i])) * pow(10.0, precedingDigits_[i]);
    //printf("  costs[%d].upper = %f \n", i, c);
    //printf("  costs[%d].upperC= %f \n", i, (double)((int)(1 + c / epsilon_[i])) * pow(10.0, precedingDigits_[i]));

    c = costs_[i]->getUpperBoundCostPerDistance();
    upperBoundCostPerDistance_ += (double)((int)(1 + c / epsilon_[i])) * pow(10.0, precedingDigits_[i]);
  }

  // display bounds...
  //std::cout << "Cost bounds:\n";
  //std::cout << "  costs[:].lower = " << lowerBoundStateCost_ << "\n";
  //std::cout << "  costs[:].upper = " << upperBoundStateCost_ << "\n";
  //std::cout << "  costs[:].lowerd= " << lowerBoundCostPerDistance_ << "\n";
  //std::cout << "  costs[:].upperd= " << upperBoundCostPerDistance_ << "\n";
}

double StateTransitionCostLexicographic::evaluate(const Environment::Ptr& environment, const StateStats& stats) const
{
  double cost = 0;
  for (unsigned int i = 0; i < costs_.size(); i++) {
    double c = costs_[i]->evaluate(environment, stats);
    if (std::isnan(c) || c == failure_) return failure_;
    cost += (double)((int)(1 + c / epsilon_[i])) * pow(10.0, precedingDigits_[i]);
  }
  return cost;
}

double StateTransitionCostLexicographic::evaluate(const Environment::Ptr& environment, const StateStats& stats1, const StateStats& stats2, double& costPerDistance) const
{
  double cPerDistance = 0;
  costPerDistance = 0;
  for (unsigned int i = 0; i < costs_.size(); i++) {
    double c = costs_[i]->evaluate(environment, stats1, stats2, cPerDistance);
    if (std::isnan(c) || c == failure_) return failure_;
    costPerDistance += (double)((int)(1 + cPerDistance / epsilon_[i])) * pow(10.0, precedingDigits_[i]);
  }
  return costPerDistance * stats1.state->space_->distance(stats1.state, stats2.state);
}

double StateTransitionCostLexicographic::getLowerBoundStateCost() const
{
  return lowerBoundStateCost_;
}

double StateTransitionCostLexicographic::getLowerBoundCostPerDistance() const
{
  return lowerBoundCostPerDistance_;
}

double StateTransitionCostLexicographic::getUpperBoundStateCost() const
{
  return upperBoundStateCost_;
}

double StateTransitionCostLexicographic::getUpperBoundCostPerDistance() const
{
  return upperBoundCostPerDistance_;
}


//---------------------------------------------------------------------------
// soft lexicographic method for multiobjective optimization (least to most important)

StateTransitionCostLexicographicSoft::StateTransitionCostLexicographicSoft(const RobotModel::Ptr& model, bool projectToEnvironmentBeforeComputingCost, const std::vector<StateTransitionCost::Ptr>& costs)
  : StateTransitionCostMultiObjective(model, projectToEnvironmentBeforeComputingCost, costs)
  , lowerBoundStateCost_(0)
  , lowerBoundCostPerDistance_(0)
  , upperBoundStateCost_(0)
  , upperBoundCostPerDistance_(0)
{
  // scaling for defacto lexicographic optimization...
  for (unsigned int i = 0; i < costs_.size(); i++) {
    weights_.push_back(pow(10.0,(double)i));
  }

  // pre-compute bounds
  for (unsigned int i = 0; i < costs_.size(); i++) {
    // bounds of normalized costs will be [0.1, 1.0]
    lowerBoundStateCost_ += weights_[i] * 0.1;
    upperBoundStateCost_ += weights_[i] * 1.0;
    lowerBoundCostPerDistance_ += weights_[i] * 0.1;
    upperBoundCostPerDistance_ += weights_[i] * 1.0;
  }

  // display bounds...
  //std::cout << "Cost bounds:\n";
  //std::cout << "  costs[:].lower = " << lowerBoundStateCost_ << "\n";
  //std::cout << "  costs[:].upper = " << upperBoundStateCost_ << "\n";
  //std::cout << "  costs[:].lowerd= " << lowerBoundCostPerDistance_ << "\n";
  //std::cout << "  costs[:].upperd= " << upperBoundCostPerDistance_ << "\n";
}

double StateTransitionCostLexicographicSoft::evaluate(const Environment::Ptr& environment, const StateStats& stats) const
{
  double cost = 0;
  for (unsigned int i = 0; i < costs_.size(); i++) {
    double c = costs_[i]->evaluate(environment, stats);
    if (std::isnan(c) || c == failure_) return failure_;
    double lb = costs_[i]->getLowerBoundStateCost();
    double ub = costs_[i]->getUpperBoundStateCost();
    if (lb == ub) printf("WARNING: lower bound equal to upper bound!! \n");
    if (c+1e-3 < lb || c-1e-3 > ub) printf("WARNING: cost %d out of bounds!!  (%f not in [%f, %f]) \n", (int)i, c, lb, ub);
    c = std::max(std::min(c, ub), lb);
    double c_normalized = (c-lb) / (ub-lb) * 0.9 + 0.1; // scale so that cost is between 0.1 and 1.0
    cost += c_normalized * weights_[i];
  }
  return cost;
}

double StateTransitionCostLexicographicSoft::evaluate(const Environment::Ptr& environment, const StateStats& stats1, const StateStats& stats2, double& costPerDistance) const
{
  double cPerDistance = 0;
  costPerDistance = 0;
  for (unsigned int i = 0; i < costs_.size(); i++) {
    double c = costs_[i]->evaluate(environment, stats1, stats2, cPerDistance);
    if (std::isnan(c) || c == failure_) return failure_;
    double lb = costs_[i]->getLowerBoundCostPerDistance();
    double ub = costs_[i]->getUpperBoundCostPerDistance();
    if (lb == ub) printf("WARNING: lower bound equal to upper bound!! \n");
    if (cPerDistance+1e-3 < lb || cPerDistance-1e-3 > ub) printf("WARNING: cost %d out of bounds!!  (%f not in [%f, %f]) \n", (int)i, cPerDistance, lb, ub);
    cPerDistance = std::max(std::min(cPerDistance, ub), lb);
    double c_normalized = (cPerDistance-lb) / (ub-lb) * 0.9 + 0.1; // scale so that cost is between 0.1 and 1.0
    costPerDistance += c_normalized * weights_[i];
  }
  return costPerDistance * stats1.state->space_->distance(stats1.state, stats2.state);
}

double StateTransitionCostLexicographicSoft::getLowerBoundStateCost() const
{
  return lowerBoundStateCost_;
}

double StateTransitionCostLexicographicSoft::getLowerBoundCostPerDistance() const
{
  return lowerBoundCostPerDistance_;
}

double StateTransitionCostLexicographicSoft::getUpperBoundStateCost() const
{
  return upperBoundStateCost_;
}

double StateTransitionCostLexicographicSoft::getUpperBoundCostPerDistance() const
{
  return upperBoundCostPerDistance_;
}


//---------------------------------------------------------------------------
// utopia distance (see Miettinen 1998 Nonlinear Multiobjective Optimization; 
//                   or Mausser 2006 Normalization and Other Topics in Multi­Objective Optimization;
//                   or Miettinen 2008 Introduction to Multiobjective Optimization: Noninteractive Approaches)

StateTransitionCostUtopia::StateTransitionCostUtopia(const RobotModel::Ptr& model, bool projectToEnvironmentBeforeComputingCost, const std::vector<StateTransitionCost::Ptr>& costs)
  : StateTransitionCostMultiObjective(model, projectToEnvironmentBeforeComputingCost, costs)
  , weights_(costs.size(), 1.0)
{
  maxCost_ = 0;
  for (unsigned int i = 0; i < weights_.size(); i++)
    maxCost_ += weights_[i] * 1.0; // approximation: manhattan-distance is 1.0 per cost function so w||.|| ~ wN
}

StateTransitionCostUtopia::StateTransitionCostUtopia(const RobotModel::Ptr& model, bool projectToEnvironmentBeforeComputingCost, const std::vector<StateTransitionCost::Ptr>& costs, const std::vector<double>& weights)
  : StateTransitionCostMultiObjective(model, projectToEnvironmentBeforeComputingCost, costs)
  , weights_(weights)
{
  assert(costs.size() == weights.size());
  maxCost_ = 0;
  for (unsigned int i = 0; i < weights_.size(); i++)
    maxCost_ += weights_[i] * 1.0; // approximation: manhattan-distance is 1.0 per cost function so w||.|| ~ wN
}

double StateTransitionCostUtopia::evaluate(const Environment::Ptr& environment, const StateStats& stats) const
{
  Eigen::VectorXd costs = Eigen::VectorXd::Zero(costs_.size());
  for (unsigned int i = 0; i < costs_.size(); i++) {
    double c = costs_[i]->evaluate(environment, stats);
    if (std::isnan(c) || c == failure_) return failure_;
    double lb = costs_[i]->getLowerBoundStateCost();
    double ub = costs_[i]->getUpperBoundStateCost();
    if (lb == ub) printf("WARNING: lower bound equal to upper bound!! \n");
    if (c+1e-3 < lb || c-1e-3 > ub) printf("WARNING: cost %d out of bounds!!  (%f not in [%f, %f]) \n", (int)i, c, lb, ub);
    c = std::max(std::min(c, ub), lb);
    double c_normalized = (c-lb) / (ub-lb);
    costs[i] = weights_[i] * c_normalized;
    //if (std::isnan(costs[i])){
    //  printf("normalized cost %d is bad (c=%f, lb=%f, ub=%f, w=%f)\n", i, c, lb, ub, weights_[i]);
    //}
  }
  return 1 + 10 * costs.norm(); // return distance from cost to (0,0,0,...) i.e. the utopia point
}

double StateTransitionCostUtopia::evaluate(const Environment::Ptr& environment, const StateStats& stats1, const StateStats& stats2, double& costPerDistance) const
{
  Eigen::VectorXd costs = Eigen::VectorXd::Zero(costs_.size());
  double cPerDistance = 0;
  for (unsigned int i = 0; i < costs_.size(); i++) {
    double c = costs_[i]->evaluate(environment, stats1, stats2, cPerDistance);
    if (std::isnan(c) || c == failure_) return failure_;
    double lb = costs_[i]->getLowerBoundCostPerDistance();
    double ub = costs_[i]->getUpperBoundCostPerDistance();
    if (lb == ub) printf("WARNING: lower bound equal to upper bound!! \n");
    if (cPerDistance+1e-3 < lb || cPerDistance-1e-3 > ub) printf("WARNING: cost %d out of bounds!!  (%f not in [%f, %f]) \n", (int)i, cPerDistance, lb, ub);
    cPerDistance = std::max(std::min(cPerDistance, ub), lb);
    double c_normalized = (cPerDistance-lb) / (ub-lb);
    costs[i] = weights_[i] * c_normalized;
  }
  //std::cout << "  costs = " << costs.transpose() << "\n";
  costPerDistance = 1 + 10 * costs.norm(); // return distance from cost to (0,0,0,...) i.e. the utopia point
  return costPerDistance * stats1.state->space_->distance(stats1.state, stats2.state);
}

double StateTransitionCostUtopia::getLowerBoundStateCost() const
{
  return 1;
}

double StateTransitionCostUtopia::getLowerBoundCostPerDistance() const
{
  return 1;
}

double StateTransitionCostUtopia::getUpperBoundStateCost() const
{
  return 1 + 10 * maxCost_;
}

double StateTransitionCostUtopia::getUpperBoundCostPerDistance() const
{
  return 1 + 10 * maxCost_;
}


//---------------------------------------------------------------------------
// tchebycheff distance (see Yi 2015 MRRF* paper)

StateTransitionCostTchebycheff::StateTransitionCostTchebycheff(const RobotModel::Ptr& model, bool projectToEnvironmentBeforeComputingCost, const std::vector<StateTransitionCost::Ptr>& costs)
  : StateTransitionCostMultiObjective(model, projectToEnvironmentBeforeComputingCost, costs)
  , weights_(costs.size(), 1.0)
{
  maxCost_ = 0;
  for (unsigned int i = 0; i < weights_.size(); i++)
    if (weights_[i] > maxCost_)
      maxCost_ = weights_[i];
}

StateTransitionCostTchebycheff::StateTransitionCostTchebycheff(const RobotModel::Ptr& model, bool projectToEnvironmentBeforeComputingCost, const std::vector<StateTransitionCost::Ptr>& costs, const std::vector<double>& weights)
  : StateTransitionCostMultiObjective(model, projectToEnvironmentBeforeComputingCost, costs)
  , weights_(weights)
{
  assert(costs.size() == weights.size());
  maxCost_ = 0;
  for (unsigned int i = 0; i < weights_.size(); i++)
    if (weights_[i] > maxCost_)
      maxCost_ = weights_[i];
}

double StateTransitionCostTchebycheff::evaluate(const Environment::Ptr& environment, const StateStats& stats) const
{
  Eigen::VectorXd costs = Eigen::VectorXd::Zero(costs_.size());
  double cost = 0;
  for (unsigned int i = 0; i < costs_.size(); i++) {
    double c = costs_[i]->evaluate(environment, stats);
    if (std::isnan(c) || c == failure_) return failure_;
    double lb = costs_[i]->getLowerBoundStateCost();
    double ub = costs_[i]->getUpperBoundStateCost();
    if (lb == ub) printf("WARNING: lower bound equal to upper bound!! \n");
    if (c+1e-3 < lb || c-1e-3 > ub) printf("WARNING: cost %d out of bounds!!  (%f not in [%f, %f]) \n", (int)i, c, lb, ub);
    c = std::max(std::min(c, ub), lb);
    double c_normalized = (c-lb) / (ub-lb);
    double c_normalized_to_utopia = (c_normalized-0) * weights_[i];
    if (c_normalized_to_utopia > cost)
      cost = c_normalized_to_utopia;
  }
  return 1 + 10 * cost; // return max w|c-c_utopia|
}

double StateTransitionCostTchebycheff::evaluate(const Environment::Ptr& environment, const StateStats& stats1, const StateStats& stats2, double& costPerDistance) const
{
  Eigen::VectorXd costs = Eigen::VectorXd::Zero(costs_.size());
  double cPerDistance = 0;
  double cost = 0;
  for (unsigned int i = 0; i < costs_.size(); i++) {
    double c = costs_[i]->evaluate(environment, stats1, stats2, cPerDistance);
    if (std::isnan(c) || c == failure_) return failure_;
    double lb = costs_[i]->getLowerBoundCostPerDistance();
    double ub = costs_[i]->getUpperBoundCostPerDistance();
    if (lb == ub) printf("WARNING: lower bound equal to upper bound!! \n");
    if (cPerDistance+1e-3 < lb || cPerDistance-1e-3 > ub) printf("WARNING: cost %d out of bounds!!  (%f not in [%f, %f]) \n", (int)i, cPerDistance, lb, ub);
    cPerDistance = std::max(std::min(cPerDistance, ub), lb);
    double c_normalized = (cPerDistance-lb) / (ub-lb);
    double c_normalized_to_utopia = (c_normalized-0) * weights_[i];
    if (c_normalized_to_utopia > cost)
      cost = c_normalized_to_utopia;
  }
  costPerDistance = 1 + 10 * cost; // return max w|c-c_utopia|
  return costPerDistance * stats1.state->space_->distance(stats1.state, stats2.state);
}

double StateTransitionCostTchebycheff::getLowerBoundStateCost() const
{
  return 1;
}

double StateTransitionCostTchebycheff::getLowerBoundCostPerDistance() const
{
  return 1;
}

double StateTransitionCostTchebycheff::getUpperBoundStateCost() const
{
  return 1 + 10 * maxCost_;
}

double StateTransitionCostTchebycheff::getUpperBoundCostPerDistance() const
{
  return 1 + 10 * maxCost_;
}


//---------------------------------------------------------------------------
// normalized sum

StateTransitionCostNormalized::StateTransitionCostNormalized(const RobotModel::Ptr& model, bool projectToEnvironmentBeforeComputingCost, const std::vector<StateTransitionCost::Ptr>& costs)
  : StateTransitionCostMultiObjective(model, projectToEnvironmentBeforeComputingCost, costs)
  , weights_(costs.size(), 1.0)
  , lowerBoundStateCost_(0)
  , lowerBoundCostPerDistance_(0)
  , upperBoundStateCost_(0)
  , upperBoundCostPerDistance_(0)
{
  for (unsigned int i = 0; i < costs_.size(); i++) {
    lowerBoundStateCost_ += weights_[i] * 1.0;
    upperBoundStateCost_ += weights_[i] * costs_[i]->getUpperBoundStateCost() / costs_[i]->getLowerBoundStateCost();
    lowerBoundCostPerDistance_ += weights_[i] * 1.0;
    upperBoundCostPerDistance_ += weights_[i] * costs_[i]->getUpperBoundCostPerDistance() / costs_[i]->getLowerBoundCostPerDistance();
  }
}

StateTransitionCostNormalized::StateTransitionCostNormalized(const RobotModel::Ptr& model, bool projectToEnvironmentBeforeComputingCost, const std::vector<StateTransitionCost::Ptr>& costs, const std::vector<double>& weights)
  : StateTransitionCostMultiObjective(model, projectToEnvironmentBeforeComputingCost, costs)
  , weights_(weights)
  , lowerBoundStateCost_(0)
  , lowerBoundCostPerDistance_(0)
  , upperBoundStateCost_(0)
  , upperBoundCostPerDistance_(0)
{
  assert(costs.size() == weights.size());
  for (unsigned int i = 0; i < costs_.size(); i++) {
    lowerBoundStateCost_ += weights_[i] * 1.0;
    upperBoundStateCost_ += weights_[i] * costs_[i]->getUpperBoundStateCost() / costs_[i]->getLowerBoundStateCost();
    lowerBoundCostPerDistance_ += weights_[i] * 1.0;
    upperBoundCostPerDistance_ += weights_[i] * costs_[i]->getUpperBoundCostPerDistance() / costs_[i]->getLowerBoundCostPerDistance();
  }
}

double StateTransitionCostNormalized::evaluate(const Environment::Ptr& environment, const StateStats& stats) const
{
  double cost = 0;
  for (unsigned int i = 0; i < costs_.size(); i++) {
    double c = costs_[i]->evaluate(environment, stats);
    if (std::isnan(c) || c == failure_) return failure_;
    double lb = costs_[i]->getLowerBoundStateCost();
    if (c+1e-3 < lb) printf("WARNING: cost %d out of bounds!!  (%f not > %f) \n", (int)i, c, lb);
    c = std::max(c, lb);
    double c_normalized = c / lb;
    cost += weights_[i] * c_normalized;
  }
  return 1 + 10 * cost;
}

double StateTransitionCostNormalized::evaluate(const Environment::Ptr& environment, const StateStats& stats1, const StateStats& stats2, double& costPerDistance) const
{
  double cPerDistance = 0;
  double cost = 0;
  for (unsigned int i = 0; i < costs_.size(); i++) {
    double c = costs_[i]->evaluate(environment, stats1, stats2, cPerDistance);
    if (std::isnan(c) || c == failure_) return failure_;
    double lb = costs_[i]->getLowerBoundCostPerDistance();
    if (cPerDistance+1e-3 < lb) printf("WARNING: cost %d out of bounds!!  (%f not > %f) \n", (int)i, cPerDistance, lb);
    cPerDistance = std::max(cPerDistance, lb);
    double c_normalized = cPerDistance / lb;
    cost += weights_[i] * c_normalized;
  }
  costPerDistance = 1 + 10 * cost;
  return costPerDistance * stats1.state->space_->distance(stats1.state, stats2.state);
}

double StateTransitionCostNormalized::getLowerBoundStateCost() const
{
  return 1 + 10 * lowerBoundStateCost_;
}

double StateTransitionCostNormalized::getLowerBoundCostPerDistance() const
{
  return 1 + 10 * lowerBoundCostPerDistance_;
}

double StateTransitionCostNormalized::getUpperBoundStateCost() const
{
  return 1 + 10 * upperBoundStateCost_;
}

double StateTransitionCostNormalized::getUpperBoundCostPerDistance() const
{
  return 1 + 10 * upperBoundCostPerDistance_;
}


//---------------------------------------------------------------------------
// weighted-sum method for multiobjective optimization

StateTransitionCostWeighted::StateTransitionCostWeighted(const RobotModel::Ptr& model, bool projectToEnvironmentBeforeComputingCost, const std::vector<StateTransitionCost::Ptr>& costs, const std::vector<double>& weights)
  : StateTransitionCostMultiObjective(model, projectToEnvironmentBeforeComputingCost, costs)
  , weights_(weights)
  , lowerBoundStateCost_(0)
  , lowerBoundCostPerDistance_(0)
  , upperBoundStateCost_(0)
  , upperBoundCostPerDistance_(0)
{
  // pre-compute bounds
  double c;
  for (unsigned int i = 0; i < costs_.size(); i++) {
    lowerBoundStateCost_ += weights_[i] * costs_[i]->getLowerBoundStateCost();
    lowerBoundCostPerDistance_ += weights_[i] * costs_[i]->getLowerBoundCostPerDistance();
    upperBoundStateCost_ += weights_[i] * costs_[i]->getUpperBoundStateCost();
    upperBoundCostPerDistance_ += weights_[i] * costs_[i]->getUpperBoundCostPerDistance();
  }
}

double StateTransitionCostWeighted::evaluate(const Environment::Ptr& environment, const StateStats& stats) const
{
  double cost = 0;
  for (unsigned int i = 0; i < costs_.size(); i++) {
    double c = costs_[i]->evaluate(environment, stats);
    if (std::isnan(c) || c == failure_) return failure_;
    cost += weights_[i] * c;
  }
  return cost;
}

double StateTransitionCostWeighted::evaluate(const Environment::Ptr& environment, const StateStats& stats1, const StateStats& stats2, double& costPerDistance) const
{
  double cost = 0;
  for (unsigned int i = 0; i < costs_.size(); i++) {
    double c = costs_[i]->evaluate(environment, stats1, stats2);
    if (std::isnan(c) || c == failure_) return failure_;
    cost += weights_[i] * c;
  }
  double d = stats1.state->space_->distance(stats1.state, stats2.state);
  if (d == 0)
    costPerDistance = 0;
  else
    costPerDistance = cost / d;
  return cost;
}

double StateTransitionCostWeighted::getLowerBoundStateCost() const
{
  return lowerBoundStateCost_;
}

double StateTransitionCostWeighted::getLowerBoundCostPerDistance() const
{
  return lowerBoundCostPerDistance_;
}

double StateTransitionCostWeighted::getUpperBoundStateCost() const
{
  return upperBoundStateCost_;
}

double StateTransitionCostWeighted::getUpperBoundCostPerDistance() const
{
  return upperBoundCostPerDistance_;
}


//---------------------------------------------------------------------------
// distance

StateTransitionCostDistance::StateTransitionCostDistance(const RobotModel::Ptr& model, bool projectToEnvironmentBeforeComputingCost)
  : StateTransitionCost(model, projectToEnvironmentBeforeComputingCost)
{

}

double StateTransitionCostDistance::evaluate(const Environment::Ptr& environment, const StateStats& stats) const
{
  // return cost per distance distance
  return 1.0;
}

double StateTransitionCostDistance::getLowerBoundStateCost() const
{
  return 1.0;
}

double StateTransitionCostDistance::getLowerBoundCostPerDistance() const
{
  return 1.0;
}

double StateTransitionCostDistance::getUpperBoundStateCost() const
{
  return 1.0;
}

double StateTransitionCostDistance::getUpperBoundCostPerDistance() const
{
  return 1.0;
}


//---------------------------------------------------------------------------
// reachability

StateTransitionCostReachability::StateTransitionCostReachability(const RobotModel::Ptr& model, bool projectToEnvironmentBeforeComputingCost)
  : StateTransitionCost(model, projectToEnvironmentBeforeComputingCost)
{

}

double StateTransitionCostReachability::evaluate(const Environment::Ptr& environment, const StateStats& stats) const
{
  // check collision
  if (stats.baseDistanceToCollision <= 0.01)
    return failure_;

  // check contact
  unsigned int primitivesInContact = 0;
  for (unsigned int i = 0; i < stats.legSupportRatio.size(); i++) {
    if (stats.legSupportRatio[i] > 0.5)
      primitivesInContact++;
  }

  // state cost
  if (primitivesInContact == stats.legSupportRatio.size())
    return 1.0;
  else
    return failure_;
}

double StateTransitionCostReachability::getLowerBoundStateCost() const
{
  return 1.0;
}

double StateTransitionCostReachability::getLowerBoundCostPerDistance() const
{
  return 1.0;
}

double StateTransitionCostReachability::getUpperBoundStateCost() const
{
  return 1.0;
}

double StateTransitionCostReachability::getUpperBoundCostPerDistance() const
{
  return 1.0;
}


//---------------------------------------------------------------------------
// traversability

StateTransitionCostTraversability::StateTransitionCostTraversability(const RobotModel::Ptr& model, bool projectToEnvironmentBeforeComputingCost)
  : StateTransitionCost(model, projectToEnvironmentBeforeComputingCost)
  , defaultMinSupportRatio_(0.1)
{

}

StateTransitionCostTraversability::StateTransitionCostTraversability(const RobotModel::Ptr& model, bool projectToEnvironmentBeforeComputingCost, double minSupRatio)
  : StateTransitionCost(model, projectToEnvironmentBeforeComputingCost)
  , defaultMinSupportRatio_(minSupRatio)
{

}

StateTransitionCostTraversability::StateTransitionCostTraversability(const RobotModel::Ptr& model, bool projectToEnvironmentBeforeComputingCost, const std::vector<Eigen::Vector3d>& protectedContacts)
  : StateTransitionCost(model, projectToEnvironmentBeforeComputingCost)
  , protectedContacts_(protectedContacts)
  , defaultMinSupportRatio_(0.1)
{

}

StateTransitionCostTraversability::StateTransitionCostTraversability(const RobotModel::Ptr& model, bool projectToEnvironmentBeforeComputingCost, const std::vector<Eigen::Vector3d>& protectedContacts, double minSupRatio)
  : StateTransitionCost(model, projectToEnvironmentBeforeComputingCost)
  , protectedContacts_(protectedContacts)
  , defaultMinSupportRatio_(minSupRatio)
{

}

double StateTransitionCostTraversability::evaluate(const Environment::Ptr& environment, const StateStats& stats) const
{
  // check body collision
  if (stats.baseDistanceToCollision <= 0.01)
    return failure_;

  //------------------------------------
  // mode: walking or trotting controller

  if (stats.mode == 0 || stats.mode == 1) {
    // contact collision
    for (unsigned int i = 0; i < stats.legCollision.size(); i++) {
      if (stats.legCollision[i]) {
        // return failure if the state is not protected
        if (stats.contacts.size() != stats.legCollision.size() || stats.contacts.size() != protectedContacts_.size() || stats.contacts[i] != protectedContacts_[i])
          return failure_;
      }
    }
    // points on realistic slope
    unsigned int primitivesInContact = 0;
    for (unsigned int i = 0; i < stats.legSupportRatioOfPointsWithinRealisticSlope.size(); i++) {
      if (stats.legSupportRatioOfPointsWithinRealisticSlope.size() == protectedContacts_.size() && stats.contacts[i] == protectedContacts_[i]) {
        primitivesInContact++;
      } else if (stats.legSupportRatioOfPointsWithinRealisticSlope[i] > 0.5) {
        primitivesInContact++;
      }
    }
    if (primitivesInContact < stats.legSupportRatioOfPointsWithinRealisticSlope.size()) // assume we need all contacts
      return failure_;
  }

  //------------------------------------
  // mode: abstract stepPlan controller

  else if (stats.mode == 2 && !boost::dynamic_pointer_cast<StateSpaceSwitchContacts const>(stats.state->space_)) {
    // point density
    unsigned int primitivesInContact = 0;
    for (unsigned int i = 0; i < stats.legSupportRatio.size(); i++) {
      if (stats.legSupportRatio.size() == protectedContacts_.size() && stats.contacts[i] == protectedContacts_[i]) {
        primitivesInContact++;
      } else if (stats.legSupportRatio[i] > 0.1) {
        primitivesInContact++;
      }
    }
    if (primitivesInContact < stats.legSupportRatio.size()) // assume we need all contacts
      return failure_;
  }

  //------------------------------------
  // mode: actual step placements

  else if (stats.mode == 2 && !!boost::dynamic_pointer_cast<StateSpaceSwitchContacts const>(stats.state->space_)) {
    // contact collision
    for (unsigned int i = 0; i < stats.legCollision.size(); i++) {
      if (stats.legCollision[i]) {
        // return failure if the state is not protected
        if (stats.contacts.size() != stats.legCollision.size() || stats.contacts.size() != protectedContacts_.size() || stats.contacts[i] != protectedContacts_[i])
          return failure_;
      }
    }
    // point density
    unsigned int primitivesInContact = 0;
    for (unsigned int i = 0; i < stats.legSupportRatio.size(); i++) {
      if (stats.legSupportRatio.size() == protectedContacts_.size() && stats.contacts[i] == protectedContacts_[i]) {
        primitivesInContact++;
      } else if (stats.legSupportRatio[i] > 0.9) {
        primitivesInContact++;
      }
    }
    if (primitivesInContact < stats.legSupportRatio.size()) // assume we need all contacts
      return failure_;
  }

  //------------------------------------
  // mode: unknown

  else {
    // point density
    unsigned int primitivesInContact = 0;
    for (unsigned int i = 0; i < stats.legSupportRatio.size(); i++) {
      if (stats.legSupportRatio.size() == protectedContacts_.size() && stats.contacts[i] == protectedContacts_[i]) {
        primitivesInContact++;
      } else if (stats.legSupportRatio[i] > defaultMinSupportRatio_) {
        primitivesInContact++;
      }
    }
    if (primitivesInContact < stats.legSupportRatio.size()) // assume we need all contacts
      return failure_;
    //printf("StateTransitionCostTraversability(): unknown mode\n");
    //return failure_;
  }

  // check traversability
  double avgTra = 0;
  for (unsigned int i = 0; i < stats.legTraversability.size(); i++) {
    avgTra += stats.legTraversability[i].avg / (double)stats.legTraversability.size();
  }

  // state cost
  return 0.1 + 1-avgTra;
}

double StateTransitionCostTraversability::getLowerBoundStateCost() const
{
  return 0.1;
}

double StateTransitionCostTraversability::getLowerBoundCostPerDistance() const
{
  return 0.1;
}

double StateTransitionCostTraversability::getUpperBoundStateCost() const
{
  return 1.1;
}

double StateTransitionCostTraversability::getUpperBoundCostPerDistance() const
{
  return 1.1;
}


//---------------------------------------------------------------------------
// traversability (ETH)
// something similar to Wermelinger et al., "Navigation Planning for Legged Robots in Challenging Terrain", 2016

StateTransitionCostTraversabilityETH::StateTransitionCostTraversabilityETH(const RobotModel::Ptr& model, bool projectToEnvironmentBeforeComputingCost)
  : StateTransitionCost(model, projectToEnvironmentBeforeComputingCost)
  , w_(100) // original paper does not say what value they used...
  , p_(0.5) // original paper does not say what value they used...
  , minTraversability_(0.01) // original paper doesnt have this, so the cost could go to infinity...
{

}

double StateTransitionCostTraversabilityETH::evaluate(const Environment::Ptr& environment, const StateStats& stats) const
{
  // check body collision
  if (stats.baseDistanceToCollision <= 0.01)
    return failure_;

  // check contact
  unsigned int primitivesInContact = 0;
  for (unsigned int i = 0; i < stats.legSupportRatio.size(); i++) {
    if (stats.legSupportRatio[i] > 0.1)
      primitivesInContact++;
  }
  if (primitivesInContact < stats.legSupportRatio.size()) // assume we need all contacts
    return failure_;

  // check traversability
  double avgTra = 0;
  for (unsigned int i = 0; i < stats.legTraversability.size(); i++) {
    avgTra += stats.legTraversability[i].avg / (double)stats.legTraversability.size();
  }
  if (avgTra < minTraversability_) avgTra = minTraversability_; // original paper doesnt have this, so the cost could go to infinity...
  if (avgTra > 1.00) avgTra = 1.00; // for sanity

  return 1 + w_ * pow(1 / avgTra, p_);
}

double StateTransitionCostTraversabilityETH::getLowerBoundStateCost() const
{
  return 1 + w_ * pow(1 / 1.00, p_);
}

double StateTransitionCostTraversabilityETH::getLowerBoundCostPerDistance() const
{
  return 1 + w_ * pow(1 / 1.00, p_);
}

double StateTransitionCostTraversabilityETH::getUpperBoundStateCost() const
{
  return 1 + w_ * pow(1 / minTraversability_, p_);
}

double StateTransitionCostTraversabilityETH::getUpperBoundCostPerDistance() const
{
  return 1 + w_ * pow(1 / minTraversability_, p_);
}


//---------------------------------------------------------------------------
// traversability walk-trot

StateTransitionCostTraversabilityWalkTrot::StateTransitionCostTraversabilityWalkTrot(const RobotModel::Ptr& model, bool projectToEnvironmentBeforeComputingCost)
  : StateTransitionCost(model, projectToEnvironmentBeforeComputingCost)
{

}

double StateTransitionCostTraversabilityWalkTrot::evaluate(const Environment::Ptr& environment, const StateStats& stats) const
{
  // check collision
  if (stats.baseDistanceToCollision <= 0.01)
    return failure_;

  // check contact
  unsigned int primitivesInContact = 0;
  for (unsigned int i = 0; i < stats.legSupportRatioOfPointsWithinRealisticSlope.size(); i++) {
    if (stats.legSupportRatioOfPointsWithinRealisticSlope[i] > 0.5)
      primitivesInContact++;
  }
  if (primitivesInContact < stats.legSupportRatioOfPointsWithinRealisticSlope.size()) // assume we need all contacts
    return failure_;

  // check traversability
  double avgTra = 0;
  for (unsigned int i = 0; i < stats.legTraversability.size(); i++) {
    avgTra += stats.legTraversability[i].avg / (double)stats.legTraversability.size();
  }

  // get cost for specific mode
  if (stats.mode == 0) { // walk
    return 0.1 + avgTra; // lets say we prefer walking on high-cost terrain
  }
  if (stats.mode == 1) { // trot
    return 0.1 + 1-avgTra + 0.1; // lets say we prefer trotting on low-cost terrain
  }
  return failure_;
}

double StateTransitionCostTraversabilityWalkTrot::getLowerBoundStateCost() const
{
  return 0.1;
}

double StateTransitionCostTraversabilityWalkTrot::getLowerBoundCostPerDistance() const
{
  return 0.1;
}

double StateTransitionCostTraversabilityWalkTrot::getUpperBoundStateCost() const
{
  return 1.1;
}

double StateTransitionCostTraversabilityWalkTrot::getUpperBoundCostPerDistance() const
{
  return 1.1;
}


//---------------------------------------------------------------------------
// probability success

StateTransitionCostProbabilitySuccess::StateTransitionCostProbabilitySuccess(const RobotModel::Ptr& model, bool projectToEnvironmentBeforeComputingCost)
  : StateTransitionCost(model, projectToEnvironmentBeforeComputingCost)
{

}

double StateTransitionCostProbabilitySuccess::evaluate(const Environment::Ptr& environment, const StateStats& stats) const
{
  // check roughness
  double avgRou = 1;
  for (unsigned int i = 0; i < stats.legRoughness.size(); i++) {
    avgRou *= (1 - (stats.legRoughness[i].avg/0.1)); // 0.1 comes from filters_demo_filter_chain.yaml
  }
  double normalizedRoughness = 1 - avgRou;
  normalizedRoughness = std::max(std::min(normalizedRoughness, 1.0), 0.0);

  // check traversability
  double avgTra = 1;
  for (unsigned int i = 0; i < stats.legTraversability.size(); i++) {
    avgTra *= stats.legTraversability[i].avg;
  }
  double traversability = std::max(std::min(avgTra, 1.0), 0.0);

  // probability of having support
  double P_support = 0.0;
  for (unsigned int i = 0; i < stats.legSupportRatioOfPointsWithinRealisticSlope.size(); i++)
    P_support += stats.legSupportRatioOfPointsWithinRealisticSlope[i] / stats.legSupportRatioOfPointsWithinRealisticSlope.size();
  //double P_support = 1.0;
  //for (unsigned int i = 0; i < stats.legSupportRatioOfPointsWithinRealisticSlope.size(); i++)
  //  P_support *= stats.legSupportRatioOfPointsWithinRealisticSlope[i];

  // factors
  double roughnessFactor = (1-normalizedRoughness); // more roughness is lower success
  double traversabilityFactor = traversability; // more traversability is higher success
  double completenessFactor = P_support; // more holes is lower success

  // success rates of each controller
  double P_success_given_support = 0.0;
  double P_success_given_nosupport = 0.0;
  if (stats.mode == 0) {
    // walk
    P_success_given_support = 1.0 * pow(traversabilityFactor,0.5) * pow(roughnessFactor,1.0) * pow(completenessFactor,0.5); // more roughness is less P_success
    P_success_given_nosupport = 0.0;
  } else if (stats.mode == 1) {
    // trot
    P_success_given_support = 1.0 * pow(traversabilityFactor,2.0) * pow(roughnessFactor,2.0) * pow(completenessFactor,0.5); // trot has more trouble in tough conditions
    P_success_given_nosupport = 0.0;
  } else if (stats.mode == 2) {
    // step
    P_success_given_support = 1.0; // footstep planning "always" succeeds
    P_success_given_nosupport = 0.0;
  } else {
    //printf("WARNING: mode %d is not handled (StateTransitionCostProbabilitySuccess).\n", stats.mode);
    //return failure_;
    P_success_given_support = 1.0 * pow(traversabilityFactor,0.5) * pow(roughnessFactor,1.0) * pow(completenessFactor,0.5); // more roughness is less P_success
    P_success_given_nosupport = 0.0;
  }

  // probability
  double P = P_success_given_support * P_support + P_success_given_nosupport * (1-P_support);
  //printf("  mode=%d, sup=%.3f, trav=%.3f, rough=%.3f, compl=%.3f, Ps|s=%.3f, P=%.3f \n", stats.mode, P_support, traversabilityFactor, roughnessFactor, completenessFactor, P_success_given_support, P);

  // turn this into a cost
  double cost = 1 - P;
  cost = std::max(cost, getLowerBoundStateCost());
  cost = std::min(cost, getUpperBoundStateCost());
  return cost;
}

double StateTransitionCostProbabilitySuccess::getLowerBoundStateCost() const
{
  return 0.1;
}

double StateTransitionCostProbabilitySuccess::getLowerBoundCostPerDistance() const
{
  return 0.1;
}

double StateTransitionCostProbabilitySuccess::getUpperBoundStateCost() const
{
  return 1.0;
}

double StateTransitionCostProbabilitySuccess::getUpperBoundCostPerDistance() const
{
  return 1.0;
}


//---------------------------------------------------------------------------
// expected energy walk-trot

StateTransitionCostExpectedEnergyWalkTrot::StateTransitionCostExpectedEnergyWalkTrot(const RobotModel::Ptr& model, bool projectToEnvironmentBeforeComputingCost)
  : StateTransitionCost(model, projectToEnvironmentBeforeComputingCost)
{

}

double StateTransitionCostExpectedEnergyWalkTrot::evaluate(const Environment::Ptr& environment, const StateStats& stats) const
{
  // check mode
  if (stats.mode < 0 || stats.mode >= 2)
    return failure_;

  // state cost
  double costWalk = getUpperBoundStateCost(); // mode = 0
  double costTrot = getLowerBoundStateCost(); // mode = 1
  return stats.mode == 0 ? costWalk : costTrot;
//double dmode = stats.modeVec(0);
//return ( costTrot * (2-fabs(dmode-2)) + costWalk * (2-fabs(dmode-0)) ) / 2;
}

double StateTransitionCostExpectedEnergyWalkTrot::getLowerBoundStateCost() const
{
  return 10;
}

double StateTransitionCostExpectedEnergyWalkTrot::getLowerBoundCostPerDistance() const
{
  return 10;
}

double StateTransitionCostExpectedEnergyWalkTrot::getUpperBoundStateCost() const
{
  return 20;
}

double StateTransitionCostExpectedEnergyWalkTrot::getUpperBoundCostPerDistance() const
{
  return 20;
}


//---------------------------------------------------------------------------
// expected energy general

StateTransitionCostExpectedEnergy::StateTransitionCostExpectedEnergy(const RobotModel::Ptr& model, bool projectToEnvironmentBeforeComputingCost, const StateSpace::Ptr& stateSpace)
  : StateTransitionCost(model, projectToEnvironmentBeforeComputingCost)
  , powerIdle_(1000.0)
  , powerWalk_(1000.0)
  , powerTrot_(1000.0)
  , powerStep_(1000.0)
  , speedStraightWalk_(0.06) // 35s for 2m, 0.06m/s (measured with joystick)
  , speedSidewaysWalk_(0.02) //120s for 2m, 0.02m/s (measured with joystick)
  , speedStraightTrot_(0.40) //  5s for 2m, 0.40m/s (measured with joystick)
  , speedSidewaysTrot_(0.13) // 15s for 2m, 0.13m/s (measured with joystick)
  , speedStraightStep_(0.06) // 35s for 2m, 0.06m/s (assumed = walk)
  , speedSidewaysStep_(0.02) //120s for 2m, 0.02m/s (assumed = walk)
  , computationTimePerMeterStep_(1.00) // 2s for 1m ?
  , computationTimePerPlanStep_(10.00) // time budget given to an anytime algorithm doing footstep planning
{
  // pre-compute bounds
  minPower_ = std::numeric_limits<double>::infinity();
  maxPower_ = 0;
  minEnerPerMeter_ = std::numeric_limits<double>::infinity();
  maxEnerPerMeter_ = 0;
  std::vector<int> modes = stateSpace->getModes();
  if (modes.size() == 0) modes.push_back(0); // default mode is walking
  for (unsigned int i = 0; i < modes.size(); i++) {
    double timeStraight(0), timeSideways(0), timeComputation(0), timeComputationPerMeter(0), power(0);
    if (modes[i] == 0) {
      // walk
      timeStraight = 1/speedStraightWalk_; // 1 meter
      timeSideways = 1/speedSidewaysWalk_; // 1 meter
      power = powerWalk_;
    } else if (modes[i] == 1) {
      // trot
      timeStraight = 1/speedStraightTrot_; // 1 meter
      timeSideways = 1/speedSidewaysTrot_; // 1 meter
      power = powerTrot_;
    } else if (modes[i] == 2) {
      // step
      timeStraight = 1/speedStraightStep_ + computationTimePerMeterStep_; // 1 meter + computation
      timeSideways = 1/speedSidewaysStep_ + computationTimePerMeterStep_; // 1 meter + computation
      timeComputation = computationTimePerPlanStep_;
      timeComputationPerMeter = 1.0*computationTimePerMeterStep_;
      power = powerStep_;
    }
    if (power < minPower_)
      minPower_ = power;
    if (power > maxPower_)
      maxPower_ = power;
    double mintime = timeComputation +(std::min(timeStraight, timeSideways) + timeComputationPerMeter);
    double maxtime = timeComputation +(std::max(timeStraight, timeSideways) + timeComputationPerMeter) * (3/sqrt(3)); // inflate by manhattan-distance
    if (power * mintime < minEnerPerMeter_)
      minEnerPerMeter_ = power * mintime;
    if (power * maxtime > maxEnerPerMeter_)
      maxEnerPerMeter_ = power * maxtime;
  }
  if (powerIdle_ < minPower_)
    minPower_ = powerIdle_;
  if (minPower_ == maxPower_)
    maxPower_ = minPower_ * 1.1;
}

double StateTransitionCostExpectedEnergy::evaluate(const Environment::Ptr& environment, const StateStats& stats) const
{
  if (stats.mode == 0) {
    return powerWalk_;
  } else if (stats.mode == 1) {
    return powerTrot_;
  } else if (stats.mode == 2) {
    return powerStep_;
  } else {
    //printf("WARNING: mode %d is not handled (StateTransitionCostExpectedEnergy).\n", stats.mode);
    //return failure_;
    return powerWalk_;
  }
}

double StateTransitionCostExpectedEnergy::evaluate(const Environment::Ptr& environment, const StateStats& stats1, const StateStats& stats2, double& costPerDistance) const
{
  // get total, straight and side distances
  flp::Transform T1, T2;
  stats1.state->space_->getTransform(stats1.state, T1);
  stats2.state->space_->getTransform(stats2.state, T2);
  double r1,p1,y1,r2,p2,y2;
  T1.getRPY(r1,p1,y1);
  T2.getRPY(r2,p2,y2);
  Eigen::Vector2d heading(cos(y1), sin(y1));
  Eigen::Vector2d movement(T2.x-T1.x, T2.y-T1.y);
  double diffYaw = y2-y1;
  if (diffYaw > M_PI) diffYaw -= 2*M_PI;
  if (diffYaw <-M_PI) diffYaw += 2*M_PI;
  double distTotal = movement.norm();
  double distStraight = fabs(heading.dot(movement));
  double distSideways = distTotal - distStraight + fabs(diffYaw) * 0.40; //TODO. robot rotating radius (center to legs)
  double distHeight = fabs(T2.z-T1.z);
  // get power and time
  double speedStraight(0), speedSideways(0), timeComputation(0), power(0);
  if (stats2.mode == 0) {
    // walk
    power = powerWalk_;
    speedStraight = speedStraightWalk_;
    speedSideways = speedSidewaysWalk_;
  } else if (stats2.mode == 1) {
    // trot
    power = powerTrot_;
    speedStraight = speedStraightTrot_;
    speedSideways = speedSidewaysTrot_;
  } else if (stats2.mode == 2) {
    // step
    power = powerStep_;
    speedStraight = speedStraightStep_;
    speedSideways = speedSidewaysStep_;
    timeComputation = computationTimePerMeterStep_ * distTotal;
  } else {
    //printf("WARNING: mode %d is not handled (StateTransitionCostExpectedEnergy).\n", stats1.mode);
    //return failure_;
    power = powerWalk_;
    speedStraight = speedStraightWalk_;
    speedSideways = speedSidewaysWalk_;
  }
  if (stats1.mode != 2 && stats2.mode == 2) {
    // doing a "step plan" requires computation time (i.e. the time budget you allow for on an anytime algorithm)
    timeComputation += computationTimePerPlanStep_;
  }
  double timeExecution = (distStraight / speedStraight) + (distSideways / speedSideways) + (distHeight / speedStraight);
  double energy = power * (timeExecution + timeComputation);

  // energy per distance
  double d = stats1.state->space_->distance(stats1.state, stats2.state);
  if (d == 0)
    costPerDistance = minEnerPerMeter_;
  else
    costPerDistance = energy / d;
  //if (costPerDistance > maxEnerPerMeter_)
  //  printf("energy/d > UB !!! En=%f, Po=%f, Te=%f, Tc=%f, dist=%f, En/d=%f \n", energy, power, timeExecution, timeComputation, d, costPerDistance);
  costPerDistance = std::max(std::min(costPerDistance, maxEnerPerMeter_), minEnerPerMeter_);

  // minimum energy
  if (energy < costPerDistance * d)
    energy = costPerDistance * d;
  if (energy == 0)
    energy = powerIdle_ * 1.0; // 1 second to do nothing (e.g. change mode)

  // return energy
  return energy;
}

double StateTransitionCostExpectedEnergy::getLowerBoundStateCost() const
{
  return minPower_;
}

double StateTransitionCostExpectedEnergy::getLowerBoundCostPerDistance() const
{
  return minEnerPerMeter_;
}

double StateTransitionCostExpectedEnergy::getUpperBoundStateCost() const
{
  return maxPower_;
}

double StateTransitionCostExpectedEnergy::getUpperBoundCostPerDistance() const
{
  return maxEnerPerMeter_;
}


//---------------------------------------------------------------------------
// expected energy contact-planning

StateTransitionCostExpectedEnergyContacts::StateTransitionCostExpectedEnergyContacts(const RobotModel::Ptr& model, bool projectToEnvironmentBeforeComputingCost, const StateSpace::Ptr& stateSpace)
  : StateTransitionCost(model, projectToEnvironmentBeforeComputingCost)
  , power_(1000) // Watts
  , velocity_(0.25) // meters/sec
  , comTransferTime_(0.5) // sec
  , profileHeight_(0.1) // meters
  , minDistPerFootstep_(0.02) // meters
  , maxDistPerFootstep_(0.30) // meters
{

}

double StateTransitionCostExpectedEnergyContacts::evaluate(const Environment::Ptr& environment, const StateStats& stats) const
{
  return 1;
}

double StateTransitionCostExpectedEnergyContacts::evaluate(const Environment::Ptr& environment, const StateStats& stats1, const StateStats& stats2, double& costPerDistance) const
{
  // check if states are neighbors
  std::vector<Eigen::Vector3d> contacts1;
  std::vector<Eigen::Vector3d> contacts2;
  if (!stats1.state->space_->getContacts(stats1.state, contacts1) || !stats2.state->space_->getContacts(stats2.state, contacts2))
    return failure_;
  bool neighbors = true;
  for (unsigned int i = 0; i < contacts1.size(); i++) {
    if (contacts1[i] != contacts2[i]) {
      neighbors = false;
      break;
    }
  }
  // compute time to execute motion
  double dist = stats1.state->space_->distance(stats1.state, stats2.state);
  if (neighbors) {
    double time = comTransferTime_ + (2*profileHeight_/velocity_) + (dist/velocity_); // 0.5s to go up/down the rest at fixed velocity
    costPerDistance = power_ * time / dist;
    return power_ * time;
  } else {
    //double numFootsteps = contacts1.size() * dist / maxDistPerFootstep_; // assumes that distance is normalized by number of contacts
    //time = numFootsteps * (comTransferTime_ + (2*profileHeight_/velocity_) + (maxDistPerFootstep_/velocity_));
    //costPerDistance = power_ * time / dist;
    //return power_ * time;
    // which simplifies to:
    costPerDistance = power_ * contacts1.size() * (comTransferTime_ + (2*profileHeight_/velocity_) + (maxDistPerFootstep_/velocity_)) / maxDistPerFootstep_;
    return costPerDistance * dist;
  }
}

double StateTransitionCostExpectedEnergyContacts::getLowerBoundStateCost() const
{
  return 1;
}

double StateTransitionCostExpectedEnergyContacts::getLowerBoundCostPerDistance() const
{
  return power_ * (comTransferTime_ + (2*profileHeight_/velocity_) + (maxDistPerFootstep_/velocity_)) / maxDistPerFootstep_;
}

double StateTransitionCostExpectedEnergyContacts::getUpperBoundStateCost() const
{
  return 1.1;
}

double StateTransitionCostExpectedEnergyContacts::getUpperBoundCostPerDistance() const
{
  return power_ * (comTransferTime_ + (2*profileHeight_/velocity_) + (minDistPerFootstep_/velocity_)) / minDistPerFootstep_;
}


//---------------------------------------------------------------------------
// wifi path loss

StateTransitionCostWifiPathLoss::StateTransitionCostWifiPathLoss(const RobotModel::Ptr& model, const Eigen::Vector3d& transmitter, double frequencyMHz, bool projectToEnvironmentBeforeComputingCost)
  : StateTransitionCost(model, projectToEnvironmentBeforeComputingCost)
  , transmitter_(transmitter)
  , frequencyMHz_(frequencyMHz)
{

}

double StateTransitionCostWifiPathLoss::evaluate(const Environment::Ptr& environment, const StateStats& stats) const
{
  // Rath et al., "Realistic indoor path loss modeling for regular wifi operations", 2017
  //  pathloss = 20log10(f_MHz) + 30*log10(d_m) + obstacleLosses - 28
  //    obstacle         loss (dBm):
  //    wooden obstacle  2.67
  //    concrete wall    2.73
  //    pillar(0.6x0.6)  6.00
  //    glass            4.50
  //
  // Sommer et al., "A computationally inexpensive empirical model of IEEE 80211p"
  //  obstacleloss = beta * n_walls + gamma * obstacle_length
  //  beta = 9.2 dB/wall,  gamma = 0.32 dB/m  (warehouse)
  //  beta = 2.4 dB/wall,  gamma = 0.63 dB/m  (lightly built house)
  //

  // get distance to wifi transmitter
  flp::Transform T;
  bool ok = stats.state->space_->getTransform(stats.state, T);
  Eigen::Vector3d position(T.x,T.y,T.z);
  if (!ok) std::cout << "ERROR: cant compute state transform for cost\n";
  double distance = (position - transmitter_).norm();
  if (distance < 1.0)
    distance = 1.0;

  // get obstacle losses. TODO
  double numWalls = environment->getNumIntersectedPoints(position, transmitter_);
  double obstacleLosses = 2.5 * numWalls;

  //if (numWalls > 0)
  //  printf("robot behind %f wall(s)!!\n", numWalls);

  // return path loss
//double pathLoss = 20*log10(frequencyMHz_) + 30*log10(distance) + obstacleLosses - 28;
  double pathLoss = 1 + obstacleLosses;
  pathLoss = std::max(std::min(pathLoss, getUpperBoundStateCost()), getLowerBoundStateCost());
  return pathLoss;
}

double StateTransitionCostWifiPathLoss::getLowerBoundStateCost() const
{
  //return 20*log10(frequencyMHz_) + 30*log10(1.0) - 28;
  return 1;
}

double StateTransitionCostWifiPathLoss::getLowerBoundCostPerDistance() const
{
  //return 20*log10(frequencyMHz_) + 30*log10(1.0) - 28;
  return 1;
}

double StateTransitionCostWifiPathLoss::getUpperBoundStateCost() const
{
  //return 20*log10(frequencyMHz_) + 30*log10(10.0) + 2.5*10 - 28;
  return 1 + 2.5 * 5.0;
}

double StateTransitionCostWifiPathLoss::getUpperBoundCostPerDistance() const
{
  //return 20*log10(frequencyMHz_) + 30*log10(10.0) + 2.5*10 - 28;
  return 1 + 2.5 * 5.0;
}


//---------------------------------------------------------------------------
// distance to states

StateTransitionCostDistanceToStates::StateTransitionCostDistanceToStates(const RobotModel::Ptr& model, const std::vector<State::Ptr>& states, double maxDistance, bool projectToEnvironmentBeforeComputingCost)
  : StateTransitionCost(model, projectToEnvironmentBeforeComputingCost)
  , states_(states)
  , maxDistance_(maxDistance)
{

}

double StateTransitionCostDistanceToStates::evaluate(const Environment::Ptr& environment, const StateStats& stats) const
{
  if (states_.size() == 0)
    return failure_;
  double minDist = std::numeric_limits<double>::infinity();
  for (unsigned int i = 0; i < states_.size(); i++) {
    State::Ptr proj = states_[i]->space_->projectFrom(stats.state, stats.state->space_); // TODO: no need to project everytime, unless the space of states[i] is changing with i
    if (!proj) return failure_;
    double dist = proj->space_->distance(proj, states_[i]);
    if (dist < minDist) minDist = dist;
  }
  if (minDist > getUpperBoundStateCost())
    minDist = getUpperBoundStateCost();
  if (minDist < getLowerBoundStateCost())
    minDist = getLowerBoundStateCost();
  return minDist;
}

double StateTransitionCostDistanceToStates::getLowerBoundStateCost() const
{
  return 0.01;
}

double StateTransitionCostDistanceToStates::getLowerBoundCostPerDistance() const
{
  return getLowerBoundStateCost() / 0.05;
}

double StateTransitionCostDistanceToStates::getUpperBoundStateCost() const
{
  return maxDistance_;
}

double StateTransitionCostDistanceToStates::getUpperBoundCostPerDistance() const
{
  return getUpperBoundStateCost() / 0.01;
}

