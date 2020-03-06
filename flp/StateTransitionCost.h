#pragma once
#include "StateTransition.h"
#include "RobotModel.h"
#include "Environment.h"

struct StateStats
{
  StateStats()
  : mode(-1)
  , baseDistanceToCollision(0.0)
  , baseDistanceToCliff(0.0)
  , feetDiagonal1(0.0)
  , feetDiagonal2(0.0)
  , feetForeHind1(0.0)
  , feetForeHind2(0.0)
  , feetLeftRight1(0.0)
  , feetLeftRight2(0.0)
  , feetForeHind1z(0.0)
  , feetForeHind2z(0.0)
  {
  }
  void compute(const Environment::Ptr& environment, const State::Ptr& statein, const RobotModel::Ptr& model);
  friend std::ostream& operator<<(std::ostream& ss, const StateStats& stats);
  std::string toString();
  State::Ptr state;
  int mode;
  Eigen::VectorXd modeVec;
  double baseDistanceToCollision;
  double baseDistanceToCliff;
  std::vector<bool> legCollision;
  std::vector<flp::Stats> legRoughness;
  std::vector<flp::Stats> legTraversability;
  std::vector<double> legSupportRatio; // ratio of space where support is possible
  std::vector<double> legSupportRatioOfPointsWithinRealisticSlope;
  std::vector<Eigen::Vector3d> contacts;
  double feetDiagonal1;
  double feetDiagonal2;
  double feetForeHind1;
  double feetForeHind2;
  double feetLeftRight1;
  double feetLeftRight2;
  double feetForeHind1z;
  double feetForeHind2z;

};


class StateTransitionCost
{
public:
  typedef boost::shared_ptr<StateTransitionCost> Ptr;
  StateTransitionCost(const RobotModel::Ptr& model, bool projectToEnvironmentBeforeComputingCost);
  virtual double evaluate(const Environment::Ptr& environment, const State::Ptr& state) const;
  virtual double evaluate(const Environment::Ptr& environment, const StateStats& stats) const = 0;
          double evaluate(const Environment::Ptr& environment, StateTransition& transition) const;
  virtual double evaluate(const Environment::Ptr& environment, StateTransition& transition, double& costPerDistance) const;
          double evaluate(const Environment::Ptr& environment, const StateStats& stats1, const StateStats& stats2) const;
  virtual double evaluate(const Environment::Ptr& environment, const StateStats& stats1, const StateStats& stats2, double& costPerDistance) const;
  virtual double getLowerBoundStateCost() const = 0;
  virtual double getLowerBoundCostPerDistance() const = 0;
  virtual double getUpperBoundStateCost() const = 0;
  virtual double getUpperBoundCostPerDistance() const = 0;
  virtual bool isInteger() const = 0;
  virtual bool isMultiObjective() const { return false; }
  const double failure_;
protected:
  RobotModel::Ptr model_;
  bool projectToEnvironment_;
};


class StateTransitionCostMultiObjective : public StateTransitionCost
{
public:
  typedef boost::shared_ptr<StateTransitionCostMultiObjective> Ptr;
  StateTransitionCostMultiObjective(const RobotModel::Ptr& model, bool projectToEnvironmentBeforeComputingCost, const std::vector<StateTransitionCost::Ptr>& costs)
  : StateTransitionCost(model, projectToEnvironmentBeforeComputingCost), costs_(costs) {}
  virtual double evaluate(const Environment::Ptr& environment, const StateStats& stats1, const StateStats& stats2, double& costPerDistance) const;
  virtual bool isMultiObjective() const { return true; }
  std::vector<StateTransitionCost::Ptr> costs_;
};


class StateTransitionCostLexicographic : public StateTransitionCostMultiObjective
{
public:
  typedef boost::shared_ptr<StateTransitionCostLexicographic> Ptr;
  StateTransitionCostLexicographic(const RobotModel::Ptr& model, bool projectToEnvironmentBeforeComputingCost, const std::vector<StateTransitionCost::Ptr>& costs, const std::vector<double>& epsilon);
  virtual double evaluate(const Environment::Ptr& environment, const StateStats& stats) const;
  virtual double evaluate(const Environment::Ptr& environment, const StateStats& stats1, const StateStats& stats2, double& costPerDistance) const;
  virtual double getLowerBoundStateCost() const;
  virtual double getLowerBoundCostPerDistance() const;
  virtual double getUpperBoundStateCost() const;
  virtual double getUpperBoundCostPerDistance() const;
  virtual bool isInteger() const { return false; }
protected:
  std::vector<double> epsilon_;
  std::vector<int> precedingDigits_;
  double lowerBoundStateCost_;
  double lowerBoundCostPerDistance_;
  double upperBoundStateCost_;
  double upperBoundCostPerDistance_;
};


class StateTransitionCostLexicographicSoft : public StateTransitionCostMultiObjective
{
public:
  typedef boost::shared_ptr<StateTransitionCostLexicographicSoft> Ptr;
  StateTransitionCostLexicographicSoft(const RobotModel::Ptr& model, bool projectToEnvironmentBeforeComputingCost, const std::vector<StateTransitionCost::Ptr>& costs);
  virtual double evaluate(const Environment::Ptr& environment, const StateStats& stats) const;
  virtual double evaluate(const Environment::Ptr& environment, const StateStats& stats1, const StateStats& stats2, double& costPerDistance) const;
  virtual double getLowerBoundStateCost() const;
  virtual double getLowerBoundCostPerDistance() const;
  virtual double getUpperBoundStateCost() const;
  virtual double getUpperBoundCostPerDistance() const;
  virtual bool isInteger() const { return false; }
protected:
  std::vector<double> weights_;
  std::vector<int> precedingDigits_;
  double lowerBoundStateCost_;
  double lowerBoundCostPerDistance_;
  double upperBoundStateCost_;
  double upperBoundCostPerDistance_;
};


class StateTransitionCostUtopia : public StateTransitionCostMultiObjective
{
public:
  typedef boost::shared_ptr<StateTransitionCostUtopia> Ptr;
  StateTransitionCostUtopia(const RobotModel::Ptr& model, bool projectToEnvironmentBeforeComputingCost, const std::vector<StateTransitionCost::Ptr>& costs);
  StateTransitionCostUtopia(const RobotModel::Ptr& model, bool projectToEnvironmentBeforeComputingCost, const std::vector<StateTransitionCost::Ptr>& costs, const std::vector<double>& weights);
  virtual double evaluate(const Environment::Ptr& environment, const StateStats& stats) const;
  virtual double evaluate(const Environment::Ptr& environment, const StateStats& stats1, const StateStats& stats2, double& costPerDistance) const;
  virtual double getLowerBoundStateCost() const;
  virtual double getLowerBoundCostPerDistance() const;
  virtual double getUpperBoundStateCost() const;
  virtual double getUpperBoundCostPerDistance() const;
  virtual bool isInteger() const { return false; }
protected:
  std::vector<double> weights_;
  double maxCost_;
};


class StateTransitionCostTchebycheff : public StateTransitionCostMultiObjective
{
public:
  typedef boost::shared_ptr<StateTransitionCostTchebycheff> Ptr;
  StateTransitionCostTchebycheff(const RobotModel::Ptr& model, bool projectToEnvironmentBeforeComputingCost, const std::vector<StateTransitionCost::Ptr>& costs);
  StateTransitionCostTchebycheff(const RobotModel::Ptr& model, bool projectToEnvironmentBeforeComputingCost, const std::vector<StateTransitionCost::Ptr>& costs, const std::vector<double>& weights);
  virtual double evaluate(const Environment::Ptr& environment, const StateStats& stats) const;
  virtual double evaluate(const Environment::Ptr& environment, const StateStats& stats1, const StateStats& stats2, double& costPerDistance) const;
  virtual double getLowerBoundStateCost() const;
  virtual double getLowerBoundCostPerDistance() const;
  virtual double getUpperBoundStateCost() const;
  virtual double getUpperBoundCostPerDistance() const;
  virtual bool isInteger() const { return false; }
protected:
  std::vector<double> weights_;
  double maxCost_;
};


class StateTransitionCostNormalized : public StateTransitionCostMultiObjective
{
public:
  typedef boost::shared_ptr<StateTransitionCostNormalized> Ptr;
  StateTransitionCostNormalized(const RobotModel::Ptr& model, bool projectToEnvironmentBeforeComputingCost, const std::vector<StateTransitionCost::Ptr>& costs);
  StateTransitionCostNormalized(const RobotModel::Ptr& model, bool projectToEnvironmentBeforeComputingCost, const std::vector<StateTransitionCost::Ptr>& costs, const std::vector<double>& weights);
  virtual double evaluate(const Environment::Ptr& environment, const StateStats& stats) const;
  virtual double evaluate(const Environment::Ptr& environment, const StateStats& stats1, const StateStats& stats2, double& costPerDistance) const;
  virtual double getLowerBoundStateCost() const;
  virtual double getLowerBoundCostPerDistance() const;
  virtual double getUpperBoundStateCost() const;
  virtual double getUpperBoundCostPerDistance() const;
  virtual bool isInteger() const { return false; }
protected:
  std::vector<double> weights_;
  double lowerBoundStateCost_;
  double lowerBoundCostPerDistance_;
  double upperBoundStateCost_;
  double upperBoundCostPerDistance_;
};


class StateTransitionCostWeighted : public StateTransitionCostMultiObjective
{
public:
  typedef boost::shared_ptr<StateTransitionCostWeighted> Ptr;
  StateTransitionCostWeighted(const RobotModel::Ptr& model, bool projectToEnvironmentBeforeComputingCost, const std::vector<StateTransitionCost::Ptr>& costs, const std::vector<double>& weights);
  virtual double evaluate(const Environment::Ptr& environment, const StateStats& stats) const;
  virtual double evaluate(const Environment::Ptr& environment, const StateStats& stats1, const StateStats& stats2, double& costPerDistance) const;
  virtual double getLowerBoundStateCost() const;
  virtual double getLowerBoundCostPerDistance() const;
  virtual double getUpperBoundStateCost() const;
  virtual double getUpperBoundCostPerDistance() const;
  virtual bool isInteger() const { return false; }
protected:
  std::vector<double> weights_;
  double lowerBoundStateCost_;
  double lowerBoundCostPerDistance_;
  double upperBoundStateCost_;
  double upperBoundCostPerDistance_;
};


class StateTransitionCostDistance : public StateTransitionCost
{
public:
  typedef boost::shared_ptr<StateTransitionCostDistance> Ptr;
  StateTransitionCostDistance(const RobotModel::Ptr& model, bool projectToEnvironmentBeforeComputingCost);
  virtual double evaluate(const Environment::Ptr& environment, const StateStats& stats) const;
  virtual double getLowerBoundStateCost() const;
  virtual double getLowerBoundCostPerDistance() const;
  virtual double getUpperBoundStateCost() const;
  virtual double getUpperBoundCostPerDistance() const;
  virtual bool isInteger() const { return false; }
protected:
  RobotModel::Ptr model_;
  bool projectToEnvironment_;
};


class StateTransitionCostReachability : public StateTransitionCost
{
public:
  typedef boost::shared_ptr<StateTransitionCostReachability> Ptr;
  StateTransitionCostReachability(const RobotModel::Ptr& model, bool projectToEnvironmentBeforeComputingCost);
  virtual double evaluate(const Environment::Ptr& environment, const StateStats& stats) const;
  virtual double getLowerBoundStateCost() const;
  virtual double getLowerBoundCostPerDistance() const;
  virtual double getUpperBoundStateCost() const;
  virtual double getUpperBoundCostPerDistance() const;
  virtual bool isInteger() const { return false; }
};


class StateTransitionCostTraversability : public StateTransitionCost
{
public:
  typedef boost::shared_ptr<StateTransitionCostTraversability> Ptr;
  StateTransitionCostTraversability(const RobotModel::Ptr& model, bool projectToEnvironmentBeforeComputingCost);
  StateTransitionCostTraversability(const RobotModel::Ptr& model, bool projectToEnvironmentBeforeComputingCost, double minSupRatio);
  StateTransitionCostTraversability(const RobotModel::Ptr& model, bool projectToEnvironmentBeforeComputingCost, const std::vector<Eigen::Vector3d>& protectedContacts);
  StateTransitionCostTraversability(const RobotModel::Ptr& model, bool projectToEnvironmentBeforeComputingCost, const std::vector<Eigen::Vector3d>& protectedContacts, double minSupRatio);
  virtual double evaluate(const Environment::Ptr& environment, const StateStats& stats) const;
  virtual double getLowerBoundStateCost() const;
  virtual double getLowerBoundCostPerDistance() const;
  virtual double getUpperBoundStateCost() const;
  virtual double getUpperBoundCostPerDistance() const;
  virtual bool isInteger() const { return false; }
protected:
  RobotModel::Ptr model_;
  bool projectToEnvironment_;
  std::vector<Eigen::Vector3d> protectedContacts_;
  double defaultMinSupportRatio_;
};


class StateTransitionCostTraversabilityETH : public StateTransitionCost
{
public:
  typedef boost::shared_ptr<StateTransitionCostTraversabilityETH> Ptr;
  StateTransitionCostTraversabilityETH(const RobotModel::Ptr& model, bool projectToEnvironmentBeforeComputingCost);
  virtual double evaluate(const Environment::Ptr& environment, const StateStats& stats) const;
  virtual double getLowerBoundStateCost() const;
  virtual double getLowerBoundCostPerDistance() const;
  virtual double getUpperBoundStateCost() const;
  virtual double getUpperBoundCostPerDistance() const;
  virtual bool isInteger() const { return false; }
protected:
  RobotModel::Ptr model_;
  bool projectToEnvironment_;
  double w_;
  double p_;
  double minTraversability_;
};


class StateTransitionCostTraversabilityWalkTrot : public StateTransitionCost
{
public:
  typedef boost::shared_ptr<StateTransitionCostTraversabilityWalkTrot> Ptr;
  StateTransitionCostTraversabilityWalkTrot(const RobotModel::Ptr& model, bool projectToEnvironmentBeforeComputingCost);
  virtual double evaluate(const Environment::Ptr& environment, const StateStats& stats) const;
  virtual double getLowerBoundStateCost() const;
  virtual double getLowerBoundCostPerDistance() const;
  virtual double getUpperBoundStateCost() const;
  virtual double getUpperBoundCostPerDistance() const;
  virtual bool isInteger() const { return false; }
};


class StateTransitionCostProbabilitySuccess : public StateTransitionCost
{
public:
  typedef boost::shared_ptr<StateTransitionCostProbabilitySuccess> Ptr;
  StateTransitionCostProbabilitySuccess(const RobotModel::Ptr& model, bool projectToEnvironmentBeforeComputingCost);
  virtual double evaluate(const Environment::Ptr& environment, const StateStats& stats) const;
  virtual double getLowerBoundStateCost() const;
  virtual double getLowerBoundCostPerDistance() const;
  virtual double getUpperBoundStateCost() const;
  virtual double getUpperBoundCostPerDistance() const;
  virtual bool isInteger() const { return false; }
};


class StateTransitionCostExpectedEnergyWalkTrot : public StateTransitionCost
{
public:
  typedef boost::shared_ptr<StateTransitionCostTraversabilityWalkTrot> Ptr;
  StateTransitionCostExpectedEnergyWalkTrot(const RobotModel::Ptr& model, bool projectToEnvironmentBeforeComputingCost);
  virtual double evaluate(const Environment::Ptr& environment, const StateStats& stats) const;
  virtual double getLowerBoundStateCost() const;
  virtual double getLowerBoundCostPerDistance() const;
  virtual double getUpperBoundStateCost() const;
  virtual double getUpperBoundCostPerDistance() const;
  virtual bool isInteger() const { return false; }
};


class StateTransitionCostExpectedEnergy : public StateTransitionCost
{
public:
  typedef boost::shared_ptr<StateTransitionCostExpectedEnergy> Ptr;
  StateTransitionCostExpectedEnergy(const RobotModel::Ptr& model, bool projectToEnvironmentBeforeComputingCost, const StateSpace::Ptr& stateSpace);
  virtual double evaluate(const Environment::Ptr& environment, const StateStats& stats) const;
  virtual double evaluate(const Environment::Ptr& environment, const StateStats& stats1, const StateStats& stats2, double& costPerDistance) const;
  virtual double getLowerBoundStateCost() const;
  virtual double getLowerBoundCostPerDistance() const;
  virtual double getUpperBoundStateCost() const;
  virtual double getUpperBoundCostPerDistance() const;
  virtual bool isInteger() const { return false; }
protected:
  double powerIdle_;
  double powerWalk_;
  double powerTrot_;
  double powerStep_;
  double speedStraightWalk_;
  double speedSidewaysWalk_;
  double speedStraightTrot_;
  double speedSidewaysTrot_;
  double speedStraightStep_;
  double speedSidewaysStep_;
  double computationTimePerMeterStep_;
  double computationTimePerPlanStep_;
  double minPower_;
  double maxPower_;
  double minEnerPerMeter_;
  double maxEnerPerMeter_;
};


class StateTransitionCostExpectedEnergyContacts : public StateTransitionCost
{
public:
  typedef boost::shared_ptr<StateTransitionCostExpectedEnergyContacts> Ptr;
  StateTransitionCostExpectedEnergyContacts(const RobotModel::Ptr& model, bool projectToEnvironmentBeforeComputingCost, const StateSpace::Ptr& stateSpace);
  virtual double evaluate(const Environment::Ptr& environment, const StateStats& stats) const;
  virtual double evaluate(const Environment::Ptr& environment, const StateStats& stats1, const StateStats& stats2, double& costPerDistance) const;
  virtual double getLowerBoundStateCost() const;
  virtual double getLowerBoundCostPerDistance() const;
  virtual double getUpperBoundStateCost() const;
  virtual double getUpperBoundCostPerDistance() const;
  virtual bool isInteger() const { return false; }
protected:
  double power_;
  double velocity_;
  double comTransferTime_;
  double profileHeight_;
  double minDistPerFootstep_;
  double maxDistPerFootstep_;
};


class StateTransitionCostWifiPathLoss : public StateTransitionCost
{
public:
  typedef boost::shared_ptr<StateTransitionCostWifiPathLoss> Ptr;
  StateTransitionCostWifiPathLoss(const RobotModel::Ptr& model, const Eigen::Vector3d& transmitter, double frequencyMHz, bool projectToEnvironmentBeforeComputingCost);
  virtual double evaluate(const Environment::Ptr& environment, const StateStats& stats) const;
  virtual double getLowerBoundStateCost() const;
  virtual double getLowerBoundCostPerDistance() const;
  virtual double getUpperBoundStateCost() const;
  virtual double getUpperBoundCostPerDistance() const;
  virtual bool isInteger() const { return false; }
protected:
  Eigen::Vector3d transmitter_;
  double frequencyMHz_;
};


class StateTransitionCostDistanceToStates : public StateTransitionCost
{
public:
  typedef boost::shared_ptr<StateTransitionCostDistanceToStates> Ptr;
  StateTransitionCostDistanceToStates(const RobotModel::Ptr& model, const std::vector<State::Ptr>& states, double maxDistance, bool projectToEnvironmentBeforeComputingCost);
  virtual double evaluate(const Environment::Ptr& environment, const StateStats& stats) const;
  virtual double getLowerBoundStateCost() const;
  virtual double getLowerBoundCostPerDistance() const;
  virtual double getUpperBoundStateCost() const;
  virtual double getUpperBoundCostPerDistance() const;
  virtual bool isInteger() const { return false; }
protected:
  std::vector<State::Ptr> states_;
  double maxDistance_;
};

