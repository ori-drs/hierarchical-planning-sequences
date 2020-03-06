#pragma once
#include "Planner.h"
#include "StateSpace.h"
#include "StateTransitionCost.h"
#include "FeasibilityChecker.h"
#include <sbpl/discrete_space_information/environment.h>

class SBPLSpaceInformation;

class PlannerSBPL : public Planner
{
public:
  typedef boost::shared_ptr<PlannerSBPL> Ptr;
  PlannerSBPL(const StateSpace::Ptr& stateSpace, const Environment::Ptr& environment, const StateTransitionCost::Ptr& cost, const std::vector<FeasibilityChecker::Ptr>& checkers, const RobotModel::Ptr& model);
  ~PlannerSBPL();
  virtual void setCost(const StateTransitionCost::Ptr& cost);
  virtual void setSubPlanner(const Planner::Ptr& subPlanner);
  virtual void plan(const State::Ptr& start, const State::Ptr& goal, std::vector<StateTransition>& transitions);
  virtual void getLastPlanVisitedStates(std::vector<State::Ptr>& states) const;
  virtual void getLastPlanVisitedGraph(std::vector<State::Ptr>& lineList) const;
  virtual void debugLastPlanVisitedGraph();
  bool StateIsValid(const State::Ptr& state);
  bool TransitionIsValid(const State::Ptr& state1, const State::Ptr& state2, bool fastChecksOnly);
  int GetCost(const State::Ptr& state1, const State::Ptr& state2);
  int GetFromToHeuristic(const State::Ptr& state1, const State::Ptr& state2);
  std::vector<State::Ptr> GetNeighbors(const State::Ptr& state);

protected:
  StateSpace::Ptr stateSpace_;
  Environment::Ptr environment_;
  StateTransitionCost::Ptr cost_;
  std::vector<FeasibilityChecker::Ptr> checkers_;
  RobotModel::Ptr model_;
  double costToIntScale_;
  SBPLSpaceInformation* sbplSpaceInformation_;
};

//-----------------------------------------------------------------------------

struct SBPLHashEntry
{
  int stateID;
  std::vector<unsigned int> coord;
  State::Ptr state;
};

class SBPLSpaceInformation : public DiscreteSpaceInformation
{
public:
  SBPLSpaceInformation(PlannerSBPL* planner);
  ~SBPLSpaceInformation();
	void SetStartAndGoal(const State::Ptr& start, const State::Ptr& goal);
	void ReinitializeEnv();
  virtual bool InitializeEnv(const char* sEnvFile);
  virtual bool InitializeMDPCfg(MDPConfig *MDPCfg);
  virtual int GetFromToHeuristic(int FromStateID, int ToStateID);
  virtual int GetGoalHeuristic(int stateID);
  virtual int GetStartHeuristic(int stateID);
  virtual void SetAllActionsandAllOutcomes(CMDPSTATE* state);
  virtual void SetAllPreds(CMDPSTATE* state);
  virtual void GetSuccs(int SourceStateID, std::vector<int>* SuccIDV, std::vector<int>* CostV);
  virtual void GetPreds(int TargetStateID, std::vector<int>* PredIDV, std::vector<int>* CostV);
  virtual void GetRandomSuccsatDistance(int SourceStateID, std::vector<int>* SuccIDV, std::vector<int>* CLowV);
  virtual int SizeofCreatedEnv();
  virtual void PrintState(int stateID, bool bVerbose, FILE* fOut = NULL);
  virtual void PrintEnv_Config(FILE* fOut = NULL);
  virtual void PrintHashTableHist();
  virtual bool AreEquivalent(int StateID1, int StateID2);
  virtual std::vector<int> GetExpandedStates();
  virtual std::vector<int> GetExpandedGraph();
  std::vector<State::Ptr> GetStates(const std::vector<int>& stateIds);
  bool StateIsValid(int StateID);
  bool StateIsGoal(const std::vector<unsigned int>& coord);
  bool StateIsGoal(const State::Ptr& state);

protected:
  SBPLHashEntry* CreateNewHashEntry(std::vector<unsigned int>& coord);
  SBPLHashEntry* GetHashEntry(std::vector<unsigned int>& coord);
  unsigned int GETHASHBIN(std::vector<unsigned int>& coord);
  unsigned int inthash(unsigned int key);

  virtual void ComputeHeuristicValues();
  virtual void CreateStartandGoalStates();
  virtual void InitializeEnvConfig();

  bool initialized_;
  int HashTableSize_;
  std::vector<SBPLHashEntry*>* Coord2StateIDHashTable_;
  std::vector<SBPLHashEntry*> StateID2CoordTable_;
  SBPLHashEntry sbplStartState_;
  SBPLHashEntry sbplGoalState_;

  PlannerSBPL* planner_;
};

