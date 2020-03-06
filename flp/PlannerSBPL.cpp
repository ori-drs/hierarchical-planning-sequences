#include "PlannerSBPL.h"
#include <sbpl/headers.h>

PlannerSBPL::PlannerSBPL(const StateSpace::Ptr& stateSpace, const Environment::Ptr& environment, const StateTransitionCost::Ptr& cost, const std::vector<FeasibilityChecker::Ptr>& checkers, const RobotModel::Ptr& model)
  : stateSpace_(stateSpace)
  , environment_(environment)
  , checkers_(checkers)
  , model_(model)
  , sbplSpaceInformation_(NULL)
{
  // setup sbpl
  sbplSpaceInformation_ = new SBPLSpaceInformation(this);
  sbplSpaceInformation_->InitializeEnv(NULL);

  // setup cost
  setCost(cost);
}

PlannerSBPL::~PlannerSBPL()
{
  if (sbplSpaceInformation_)
    delete sbplSpaceInformation_;
}

void PlannerSBPL::setCost(const StateTransitionCost::Ptr& cost)
{
  cost_ = cost;
  if (cost->isInteger()) {
    costToIntScale_ = 1;
  } else {
    double bound = std::min(cost_->getLowerBoundStateCost(), cost_->getLowerBoundCostPerDistance()*0.001);
    costToIntScale_ = 10.0 / bound;
    //double bound = std::min(cost_->getUpperBoundStateCost(), cost_->getUpperBoundCostPerDistance()*0.001);
    //costToIntScale_ = 1.0 / bound;
  }
}

void PlannerSBPL::setSubPlanner(const Planner::Ptr& subPlanner)
{
  // TODO
}

void PlannerSBPL::plan(const State::Ptr& start, const State::Ptr& goal, std::vector<StateTransition>& transitions)
{
  transitions.clear();

  // make sure everyone has the right copy of the environment
  for (unsigned int i = 0; i < checkers_.size(); i++)
    checkers_[i]->updateEnvironment(environment_);

  // select epsilon
  double costBoundU = cost_->getUpperBoundCostPerDistance();
  double costBoundL = cost_->getLowerBoundCostPerDistance();
  double costBoundRatio = (costBoundU == costBoundL ? 1.0 : costBoundU / costBoundL);
  double epsilon = costBoundRatio + 2.0;

  // setup sbpl planner
  std::string sbplPlannerType = "ARAPlanner";
  ReplanParams sbplPlannerParams(maxTime_);
  sbplPlannerParams.max_time = maxTime_;
  sbplPlannerParams.repair_time = maxTime_;
  sbplPlannerParams.initial_eps = epsilon;
  sbplPlannerParams.final_eps = (optimizeSolution_ ? 1.0 : epsilon);
  sbplPlannerParams.dec_eps = (epsilon - 1.0) / 20; //0.2;
  sbplPlannerParams.return_first_solution = false; // set to true to run indefinitely until sol is found

  // some planners need to be destroyed and created again
  SBPLPlanner* sbplPlanner = NULL;
  if (sbplPlannerType == "RSTARPlanner" || sbplPlannerType == "ARAPlanner") {
    sbplSpaceInformation_->ReinitializeEnv();
    if (sbplPlanner)
      delete sbplPlanner;
    if (sbplPlannerType == "ARAPlanner") {
      sbplPlanner = new ARAPlanner(sbplSpaceInformation_, true);
    } else if (sbplPlannerType == "RSTARPlanner") {
      sbplPlanner = new RSTARPlanner(sbplSpaceInformation_, true);
    }
  }

  // set start and goal
  MDPConfig mdpConfig;
  sbplSpaceInformation_->SetStartAndGoal(start, goal);
  sbplSpaceInformation_->InitializeMDPCfg(&mdpConfig);

  // planner config
  if (!sbplPlanner->set_start(mdpConfig.startstateid) ||
      !sbplPlanner->set_goal(mdpConfig.goalstateid) ) {
    printf("couldnt set start/goal\n");
    return;
  }
  sbplPlanner->set_initialsolution_eps(sbplPlannerParams.initial_eps);
  sbplPlanner->set_search_mode(sbplPlannerParams.return_first_solution);

  // start equal to goal? (this is not detected by sbpl since a node will not be neighbor of itself)
  if (sbplSpaceInformation_->StateIsGoal(start)) {
    StateTransition t;
    t.state = start;
    t.newState = goal;
    t.cost = 0;
    transitions.push_back(t);
    delete sbplPlanner;
    return;
  }

  // search
  int res;
  std::vector<int> solution;
  try {
    if (sbplPlannerType == "ARAPlanner")
      res = sbplPlanner->replan(&solution, sbplPlannerParams);
    else
      res = sbplPlanner->replan(sbplPlannerParams.max_time, &solution);
  } catch (const SBPL_Exception& e) {
    printf("sbpl exception (%s)\n", e.what());
    delete sbplPlanner;
    return;
  }
  delete sbplPlanner;

  // check solution
  if (res && solution.size() > 0) {
    std::vector<State::Ptr> states = sbplSpaceInformation_->GetStates(solution);
    // fill transitions
    for (unsigned int i = 0; i < states.size(); i++) {
      if (transitions.size() > 0) {
        transitions.back().newState = states[i];
        transitions.back().cost = cost_->evaluate(environment_, transitions.back());
      }
      // don't make a transition starting on the last state (since we cant fill newstate)
      if (i == states.size()-1)
        break;
      // new transition
      StateTransition t;
      t.state = states[i];
      transitions.push_back(t);
    }
  }
}

void PlannerSBPL::getLastPlanVisitedStates(std::vector<State::Ptr>& states) const
{
  states.clear();
  if (!sbplSpaceInformation_)
    return;
  std::vector<int> expStates = sbplSpaceInformation_->GetExpandedStates();
  states = sbplSpaceInformation_->GetStates(expStates);
}

void PlannerSBPL::getLastPlanVisitedGraph(std::vector<State::Ptr>& lineList) const
{
  lineList.clear();
  if (!sbplSpaceInformation_)
    return;
  std::vector<int> graph = sbplSpaceInformation_->GetExpandedGraph();
  lineList = sbplSpaceInformation_->GetStates(graph);
}

void PlannerSBPL::debugLastPlanVisitedGraph()
{
  // TODO
}

bool PlannerSBPL::StateIsValid(const State::Ptr& state)
{
  // debug
  const bool debug = false;
  if (debug && viewer_) {
    StateStats stats;
    stats.compute(environment_, state, viewerModel_);
    std::cout << stats << "\n";
    viewer_->clearPrimitives();
    viewer_->addPrimitives(viewerModel_->getPrimitives(state));
    viewer_->drawAndIdle();
  }
  // first feasibility checkers
  bool feasible = state->space_->getFeasibility(state);
  if (debug && !feasible) std::cout << "state space says state NOT feasible\n";
  for (unsigned int i = 0; feasible && i < checkers_.size(); i++) {
    feasible = feasible && checkers_[i]->feasible(state);
    if (debug && !feasible) std::cout << "checker[" << i << "] says state NOT feasible\n";
  }
  if (!feasible)
    return false;
  // then check cost is not infinite
  //double cost = cost_->evaluate(environment_, state);
  //if (std::isnan(cost) || cost < 0 || cost == std::numeric_limits<double>::infinity())
  //  return false;
  //cost = cost_->evaluate(environment_, state);
  if (viewer_) {
    viewer_->clearPrimitives();
    viewer_->addPrimitives(viewerModel_->getPrimitives(state));
    viewer_->draw();
  }
  return feasible;
}

bool PlannerSBPL::TransitionIsValid(const State::Ptr& state1, const State::Ptr& state2, bool fastChecksOnly)
{
  StateTransition t;
  t.state = state1;
  t.newState = state2;
  bool feasible = true;
  for (unsigned int i = 0; feasible && i < checkers_.size(); i++) {
    if (fastChecksOnly && !checkers_[i]->isFast_) continue;
    feasible = feasible && checkers_[i]->feasible(t);
  }
  return feasible;
}

int PlannerSBPL::GetCost(const State::Ptr& state1, const State::Ptr& state2)
{
  StateTransition transition;
  transition.state = state1;
  transition.newState = state2;
  double cost = cost_->evaluate(environment_, transition);
  if (std::isnan(cost) || cost < 0 || cost == std::numeric_limits<double>::infinity())
    return INFINITECOST;
  int costInt = cost * costToIntScale_;
  if (costInt == 0 && cost > 0)
    costInt = 1;
  if (costInt <= 0 || costInt >= INFINITECOST) {
    std::cout << "WARNING: cost = " << costInt << " <= 0 or >= INF !!!\n";
    std::cout << "  state1 = " << state1->space_->toString(state1) << "\n";
    std::cout << "  state2 = " << state2->space_->toString(state2) << "\n";
    std::cout << "  distance = " << state1->space_->distance(state1, state2) << "\n";
    std::cout << "  cost = " << cost << "\n";
    std::cout << "  scale = " << costToIntScale_ << "\n";
    cost = cost_->evaluate(environment_, transition);
    if (viewer_) {
      StateStats stats1, stats2;
      stats1.compute(environment_, state1, viewerModel_);
      stats2.compute(environment_, state2, viewerModel_);
      std::cout << stats1 << "\n";
      std::cout << stats2 << "\n";
      viewer_->clearPrimitives();
      viewer_->addPrimitives(viewerModel_->getPrimitives(state1));
      viewer_->addPrimitives(viewerModel_->getPrimitives(state2));
      viewer_->drawAndIdle();
    }
  }
  return costInt;
}

int PlannerSBPL::GetFromToHeuristic(const State::Ptr& state1, const State::Ptr& state2)
{
  // TODO: check if state1 and state2 are neighbors. if so, return cost instead.
  // heuristic path length
  const int method = 0;
  double length = 0;
  if (method == 0 && heurPathLengthPlanner_) {
    // use heuristic path planner's path costs
    std::vector<StateTransition> transitions;
    heurPathLengthPlanner_->plan(state1, state2, transitions);
    for (unsigned int i = 0; i < transitions.size(); i++)
      length += transitions[i].cost;
    // show path
    if (viewer_) {
      std::vector<Eigen::Vector3d> lineList;
      Eigen::Vector4d color(0,1,0,1);
      for (unsigned int i = 0; i < transitions.size(); i++) {
        flp::Transform T1, T2;
        transitions[i].state->space_->getTransform(transitions[i].state, T1);
        transitions[i].newState->space_->getTransform(transitions[i].newState, T2);
        lineList.push_back(Eigen::Vector3d(T1.x, T1.y, T1.z));
        lineList.push_back(Eigen::Vector3d(T2.x, T2.y, T2.z));
      }
      viewer_->clearLineList();
      viewer_->addLineList(lineList, color);
    }
  } else
  if (method == 1 && heurPathLengthPlanner_) {
    // use heuristic path planner's path XYZ distances
    std::vector<StateTransition> transitions;
    heurPathLengthPlanner_->plan(state1, state2, transitions);
    flp::Transform T1, T2;
    stateSpace_->getTransform(state1, T1);
    stateSpace_->getTransform(state2, T2);
    double length_xyz_straight = sqrt( pow(T2.x-T1.x,2.0) + pow(T2.y-T1.y,2.0) + pow(T2.z-T1.z,2.0) );
    double length_xyz_heuristic = 0;
    flp::Transform Tprev = T1, Tnext;
    for (unsigned int i = 0; i < transitions.size(); i++) {
      stateSpace_->getTransform(transitions[i].newState, Tnext);
      length_xyz_heuristic += sqrt( pow(Tnext.x-Tprev.x,2.0) + pow(Tnext.y-Tprev.y,2.0) + pow(Tnext.z-Tprev.z,2.0) );
      Tprev = Tnext;
    }
    if (transitions.size() == 0 || length_xyz_heuristic < length_xyz_straight) { // make sure this is not lower than straight path
      length = stateSpace_->distance(state1, state2);
      //printf("WARNING: something's wrong the heuristic planner (%d transitions, pathLengthToGoal %f)\n", (int)transitions.size(), length_xyz_heuristic);
    } else {
      length = stateSpace_->distance(state1, state2) * length_xyz_heuristic / length_xyz_straight;
    }
  } else {
    length = stateSpace_->distance(state1, state2);
  }
  // admissibility sanity check
  double dist = stateSpace_->distance(state1, state2);
  //printf("heur_dist = %f,  straight_dist = %f \n", length, dist);
  if (length < dist) {
    length = dist;
  }
  // heuristic cost (minimium cost per distance)
  int h = cost_->getLowerBoundCostPerDistance() * length * costToIntScale_;
  //std::cout << "-------------------------------------------------\n";
  //std::cout << "heuristic s1 " << state1->space_->toString(state1) << "\n";
  //std::cout << "heuristic s2 " << state2->space_->toString(state2) << "\n";
  //printf("lb/d = %f, d = %f, scale = %f, h = %d \n", cost_->getLowerBoundCostPerDistance(), length, costToIntScale_, h);
  return h;
}

std::vector<State::Ptr> PlannerSBPL::GetNeighbors(const State::Ptr& state)
{
  // get successors
  std::vector<State::Ptr> states = state->space_->getNeighbors(state);

  // project to environment
  if (projectToEnvironment_) {
    std::vector<State::Ptr> projStates;
    projStates.reserve(states.size());
    for (unsigned int i = 0; i < states.size(); i++) {
      State::Ptr proj = model_->project(states[i], environment_);
      if (proj && state->space_->distance(state, proj) > 0)
        projStates.push_back(proj);
    }
    return projStates;
  }

  return states;
}


//-----------------------------------------------------------------------------

SBPLSpaceInformation::SBPLSpaceInformation(PlannerSBPL* planner) : initialized_(false), planner_(planner)
{

}

SBPLSpaceInformation::~SBPLSpaceInformation()
{
  ReinitializeEnv();
  delete[] Coord2StateIDHashTable_;
}

unsigned int SBPLSpaceInformation::inthash(unsigned int key)
{
  key += (key << 12);
  key ^= (key >> 22);
  key += (key << 4);
  key ^= (key >> 9);
  key += (key << 10);
  key ^= (key >> 2);
  key += (key << 7);
  key ^= (key >> 12);
  return key;
}

unsigned int SBPLSpaceInformation::GETHASHBIN(std::vector<unsigned int> &coord)
{
  int val = 0;
  for (int i = 0; i < coord.size(); i++)
    val += inthash(coord[i]) << i;
  return inthash(val) & (HashTableSize_ - 1);
}

bool SBPLSpaceInformation::InitializeEnv(const char* sEnvFile)
{
  // Initialize configuration
  InitializeEnvConfig();

  // Initialize hash table
  if (initialized_) {
    ReinitializeEnv();
  } else {
    HashTableSize_ = 1024 * 1024; //should be power of two
    Coord2StateIDHashTable_ = new std::vector<SBPLHashEntry*> [HashTableSize_];
    StateID2CoordTable_.clear();
    initialized_ = true;
  }
  return true;
}

void SBPLSpaceInformation::ReinitializeEnv()
{
  if (initialized_ == false || Coord2StateIDHashTable_ == NULL){
    initialized_ = false;
    InitializeEnv(NULL);
    return;
  }
	
  for(unsigned int i=0 ; i<HashTableSize_ ; i++)
    Coord2StateIDHashTable_[i].clear();

  for(unsigned int i=0 ; i<StateID2CoordTable_.size() ; i++)
    if (StateID2CoordTable_[i])
      delete StateID2CoordTable_[i];
  StateID2CoordTable_.clear();

  for(unsigned int i=0 ; i<StateID2IndexMapping.size() ; i++)
    if (StateID2IndexMapping[i])
      delete[] StateID2IndexMapping[i];
	StateID2IndexMapping.clear();
}

void SBPLSpaceInformation::InitializeEnvConfig()
{

}

void SBPLSpaceInformation::SetStartAndGoal(const State::Ptr& start, const State::Ptr& goal)
{
  if (!initialized_) {
    SBPL_ERROR("ERROR: setting start and goal but Sbpl configuration not set yet.\n");
    throw new SBPL_Exception();
  }
  sbplStartState_.state = start;
  sbplStartState_.coord = start->space_->toVectorInt(start);
  sbplGoalState_.state = goal;
  sbplGoalState_.coord = goal->space_->toVectorInt(goal);

  SBPLHashEntry* HashEntry;

  // create start state
  HashEntry = CreateNewHashEntry(sbplStartState_.coord);
	HashEntry->state = sbplStartState_.state;
  sbplStartState_.stateID = HashEntry->stateID;

  // create goal state
  HashEntry = CreateNewHashEntry(sbplGoalState_.coord);
	HashEntry->state = sbplGoalState_.state;
  sbplGoalState_.stateID = HashEntry->stateID;
}

void SBPLSpaceInformation::CreateStartandGoalStates()
{
  SBPLHashEntry* HashEntry;

  // create start state
  HashEntry = CreateNewHashEntry(sbplStartState_.coord);
	HashEntry->state = sbplStartState_.state->clone();
  sbplStartState_.stateID = HashEntry->stateID;

  // create goal state
  HashEntry = CreateNewHashEntry(sbplGoalState_.coord);
	HashEntry->state = sbplGoalState_.state->clone();
  sbplGoalState_.stateID = HashEntry->stateID;

  PrintState(sbplStartState_.stateID, true);
  PrintState(sbplGoalState_.stateID, true);
}

bool SBPLSpaceInformation::InitializeMDPCfg(MDPConfig *MDPCfg)
{
  // initialize MDPCfg with the start and goal ids
  MDPCfg->startstateid = sbplStartState_.stateID;
  MDPCfg->goalstateid = sbplGoalState_.stateID;
  return true;
}

SBPLHashEntry* SBPLSpaceInformation::GetHashEntry(std::vector<unsigned int> &coord)
{
  int binid = GETHASHBIN(coord);

#if DEBUG
  if ((int)Coord2StateIDHashTable_[binid].size() > 500) {
    SBPL_PRINTF("WARNING: Hash table has a bin %d (coord0=%d) of size %d\n",
      binid, coord[0], (int)Coord2StateIDHashTable_[binid].size());

    PrintHashTableHist();
  }
#endif

  // iterate over the states in the bin and select the perfect match
  for (int ind = 0; ind < (int)Coord2StateIDHashTable_[binid].size(); ind++) {
    int j = 0;
    for (j = 0; j < coord.size(); j++) {
      if (Coord2StateIDHashTable_[binid][ind]->coord[j] != coord[j]) {
        break;
      }
    }
    if (j == coord.size()) {
      return Coord2StateIDHashTable_[binid][ind];
    }
  }

  return NULL;
}

SBPLHashEntry* SBPLSpaceInformation::CreateNewHashEntry(std::vector<unsigned int> &coord)
{
  int i;
  SBPLHashEntry* HashEntry = new SBPLHashEntry;
  HashEntry->coord = coord;
  HashEntry->stateID = StateID2CoordTable_.size();

  // insert into the tables
  StateID2CoordTable_.push_back(HashEntry);

  // get the hash table bin
  i = GETHASHBIN(HashEntry->coord);

  // insert the entry into the bin
  Coord2StateIDHashTable_[i].push_back(HashEntry);

  // insert into and initialize the mappings
  int* entry = new int[NUMOFINDICES_STATEID2IND];
  StateID2IndexMapping.push_back(entry);
  for (i = 0; i < NUMOFINDICES_STATEID2IND; i++) {
    StateID2IndexMapping[HashEntry->stateID][i] = -1;
  }
  if (HashEntry->stateID != (int)StateID2IndexMapping.size() - 1) {
    SBPL_ERROR("ERROR in Env... function: last state has incorrect stateID\n");
    throw new SBPL_Exception();
  }
  return HashEntry;
}

void SBPLSpaceInformation::ComputeHeuristicValues()
{
  // whatever necessary pre-computation of heuristic values is done here
}

int SBPLSpaceInformation::GetFromToHeuristic(int FromStateID, int ToStateID)
{
  if (AreEquivalent(FromStateID, ToStateID))
    return 0;
  SBPLHashEntry* FromHashEntry = StateID2CoordTable_[FromStateID];
  SBPLHashEntry* ToHashEntry = StateID2CoordTable_[ToStateID];
  State::Ptr state1 = FromHashEntry->state;
  State::Ptr state2 = ToHashEntry->state;
  int h = planner_->GetFromToHeuristic(state1, state2);;
  //printf("h_12 = %d \n", h);
  return h;
}

int SBPLSpaceInformation::GetGoalHeuristic(int stateID)
{
  SBPLHashEntry* entry = StateID2CoordTable_[stateID];
  int h = planner_->GetFromToHeuristic(entry->state, sbplGoalState_.state);
  //printf("h_g  = %d \n", h);
  return h;
}

int SBPLSpaceInformation::GetStartHeuristic(int stateID)
{
  SBPLHashEntry* entry = StateID2CoordTable_[stateID];
  int h = planner_->GetFromToHeuristic(sbplStartState_.state, entry->state);
  //printf("h_s  = %d \n", h);
  return h;
}

void SBPLSpaceInformation::SetAllActionsandAllOutcomes(CMDPSTATE* state)
{
  SBPL_ERROR("ERROR in myenv..function: SetAllActionsandOutcomes is undefined\n");
  throw new SBPL_Exception();
}

void SBPLSpaceInformation::SetAllPreds(CMDPSTATE* state)
{
  SBPL_ERROR("ERROR in myenv... function: SetAllPreds is undefined\n");
  throw new SBPL_Exception();
}

void SBPLSpaceInformation::GetSuccs(int SourceStateID, std::vector<int>* SuccIDV, std::vector<int>* CostV)
{
  // clear the successor array
  SuccIDV->clear();
  CostV->clear();

  // goal state should be absorbing
  if (SourceStateID == sbplGoalState_.stateID) {
    return;
  }

  // get state and check validity
  SBPLHashEntry* HashEntry = StateID2CoordTable_[SourceStateID];
  State::Ptr state = HashEntry->state;
  bool valid = planner_->StateIsValid(state);

  // debug
  const bool debug = false;
  int mincost = INFINITECOST;
  if (debug) {
    std::cout << "-------------------------------------------------\n";
    std::cout << "visiting " << state->space_->toString(state) << "\n";
    std::cout << "state is " << (valid ? "valid" : "NOT VALID") << "\n";
  }

  // ignore invalid
  if (!valid)
    return;

  // get successors
  std::vector<State::Ptr> states = planner_->GetNeighbors(state);

  // debug
  if (debug) {
    for (unsigned int i = 0; i < states.size(); i++) {
      std::vector<unsigned int> coord = state->space_->toVectorInt(states[i]);
      bool equiv = true;
      for (unsigned int j = 0; j < coord.size() && equiv; j++)
        if (coord[j] != HashEntry->coord[j])
          equiv = false;
      if (equiv) {
        std::cout << "WARNING: neighbour is self !!! (1st warning)\n";
        std::cout << "  " << state->space_->toString(state) << " (self)\n";
        for (unsigned int j = 0; j < states.size(); j++)
          std::cout << "  " << state->space_->toString(states[j]) << " (nei)" << (i==j ? " ***\n" : "\n");
        break;
      }
    }
  }

  // add successors
  SuccIDV->reserve(states.size());
  CostV->reserve(states.size());
  for (unsigned int i = 0; i < states.size(); i++) {
    // do fast feasibility checks
    if (!planner_->TransitionIsValid(state, states[i], true)) // TODO: I should check for feasibility on all factors (even slow ones) at some point. not doing this now... assuming state feasiblilty enough
      continue;
    // try to add successor
    SBPLHashEntry* entry;
    std::vector<unsigned int> coord = state->space_->toVectorInt(states[i]);
    if (StateIsGoal(states[i])) {
      // if this is a goal state
      if (!planner_->StateIsValid(states[i]))
        continue;
      entry = GetHashEntry(sbplGoalState_.coord);
      entry->state = states[i];
    } else {
      // if this is not a goal state
      entry = GetHashEntry(coord);
      if (entry == NULL) {
        entry = CreateNewHashEntry(coord);
        entry->state = states[i];
      } else {
        //std::cout << "-----\n";
        //std::cout << state->space_->toString(entry->state) << "\n";
        //std::cout << state->space_->toString(states[i]) << "\n";
      }
    }
    if (entry->stateID == SourceStateID) {
      if (debug) {
        std::cout << "WARNING: neighbour is self !!!\n";
        std::cout << "  " << state->space_->toString(state) << " (self)\n";
        std::cout << "  " << state->space_->toString(states[i]) << " (nei)\n";
      }
      continue;
    }
    int cost = planner_->GetCost(state, states[i]);
    int stateID = entry->stateID;
    // debug
    if (debug) {
      if (cost < mincost)
        mincost = cost;
      double dist = state->space_->distance(state,states[i]);
      printf("DEBUG: neighbour (%s): cost = %d, dist = %f, cost/dist = %f , heur = %d\n", state->space_->toString(states[i]).c_str(), cost, dist, cost/dist, GetGoalHeuristic(stateID));
    }
    // exceptions
    if (cost >= INFINITECOST)
      continue;
    if (cost <= 0) // TODO: maybe I can return cost = 0 ? think about it.
      continue;
    // add successor
    SuccIDV->push_back(stateID);
    CostV->push_back(cost);
  }

  // debug
  if (debug) {
    printf("  min cost = %d \n", mincost);
  }
}

void SBPLSpaceInformation::GetPreds(int TargetStateID, std::vector<int>* PredIDV, std::vector<int>* CostV)
{
  SBPL_ERROR("ERROR in myenv... function: GetPreds is undefined\n");
  throw new SBPL_Exception();
}

void SBPLSpaceInformation::GetRandomSuccsatDistance(int SourceStateID, std::vector<int>* SuccIDV, std::vector<int>* CLowV)
{
  SBPL_ERROR("ERROR in myenv... function: GetPreds is undefined\n");
  throw new SBPL_Exception();
}

bool SBPLSpaceInformation::AreEquivalent(int State1ID, int State2ID)
{
  SBPLHashEntry* HashEntry1 = StateID2CoordTable_[State1ID];
  SBPLHashEntry* HashEntry2 = StateID2CoordTable_[State2ID];
  for (unsigned int i=0 ; i<HashEntry1->coord.size() ; i++)
    if (HashEntry1->coord[i] != HashEntry2->coord[i])
      return false;
  return true;
}

std::vector<int> SBPLSpaceInformation::GetExpandedStates()
{
  // get expanded state IDs
  int N = StateID2CoordTable_.size();
  std::vector<int> expStates;
  expStates.reserve(N);
  for (int i = 0; i < N; i++)
    expStates.push_back(i);
  return expStates;
}

std::vector<int> SBPLSpaceInformation::GetExpandedGraph()
{
  SBPL_PRINTF("WARNING: now computing expanded graph from the expanded states. this might take a while...");
  std::vector<int> graph;
  // get expanded states
  std::vector<int> expStates = GetExpandedStates();
  graph.reserve(expStates.size()*10);
  // for all expanded states
  for (unsigned int i = 0; i < expStates.size(); i++) {
    // report progress
    if (i % (expStates.size()/10) == 0)
      SBPL_PRINTF("  Completed %3d\%", 100*i/expStates.size());
    // temporary: just show nodes and not connections. TODO: remove this and find faster solution to obtain real graph
    graph.push_back(expStates[i]);
    graph.push_back(expStates[i]);
    continue;
    // skip invalid
    if (!StateIsValid(expStates[i]))
      continue;
    // get successors
    std::vector<int> succ;
    std::vector<int> cost;
    GetSuccs(expStates[i], &succ, &cost);
    for (unsigned int j = 0; j < succ.size(); j++) {
      // skip invalid
      if (!StateIsValid(succ[j]))
        continue;
      // check if successor is on expanded states. if so add edge to graph
      if (std::find(expStates.begin(), expStates.end(), succ[j]) != expStates.end()) {
        graph.push_back(expStates[i]);
        graph.push_back(succ[j]);
      }
    }
  }
  SBPL_PRINTF("  Completed 100\%");
  return graph;
}

std::vector<State::Ptr> SBPLSpaceInformation::GetStates(const std::vector<int>& stateIds)
{
  std::vector<State::Ptr> states;
  SBPLHashEntry* entry;
  for (unsigned int i = 0; i < stateIds.size(); i++) {
    if (entry=StateID2CoordTable_[stateIds[i]])
      states.push_back(entry->state);
  }
  return states;
}

bool SBPLSpaceInformation::StateIsValid(int StateID)
{
	SBPLHashEntry* HashEntry = StateID2CoordTable_[StateID];
	return planner_->StateIsValid(HashEntry->state);
}

bool SBPLSpaceInformation::StateIsGoal(const std::vector<unsigned int>& coord)
{
  State::Ptr state = sbplGoalState_.state->space_->fromVectorInt(coord);
  return StateIsGoal(state);
  //for (unsigned int i = 0; i < coord.size(); i++)
  //  if (coord[i] != sbplGoalState_.coord[i])
  //    return false;
  //return true;
}

bool SBPLSpaceInformation::StateIsGoal(const State::Ptr& state)
{
  double distance = state->space_->distance(state, sbplGoalState_.state);
  return distance < 0.10; // TODO: this should be a parameter
}

int SBPLSpaceInformation::SizeofCreatedEnv()
{
  return (int)StateID2CoordTable_.size();
}

void SBPLSpaceInformation::PrintState(int stateID, bool bVerbose, FILE* fOut)
{
  if (fOut == NULL) fOut = stdout;
  SBPLHashEntry* HashEntry = StateID2CoordTable_[stateID];
  std::stringstream str;
  str << "coords(" << stateID << "): ";
  for (unsigned int i=0 ; i<HashEntry->coord.size() ; i++)
    str << (int)HashEntry->coord[i] << ",";
  SBPL_FPRINTF(fOut, "%s", str.str().c_str());
}

void SBPLSpaceInformation::PrintEnv_Config(FILE* fOut)
{

}

void SBPLSpaceInformation::PrintHashTableHist()
{
  int s0 = 0, s1 = 0, s50 = 0, s100 = 0, s200 = 0, s300 = 0, slarge = 0;

  for (int j = 0; j < (int)HashTableSize_; j++) {
    if ((int)Coord2StateIDHashTable_[j].size() == 0)
      s0++;
    else if ((int)Coord2StateIDHashTable_[j].size() < 50)
      s1++;
    else if ((int)Coord2StateIDHashTable_[j].size() < 100)
      s50++;
    else if ((int)Coord2StateIDHashTable_[j].size() < 200)
      s100++;
    else if ((int)Coord2StateIDHashTable_[j].size() < 300)
      s200++;
    else if ((int)Coord2StateIDHashTable_[j].size() < 400)
      s300++;
    else
      slarge++;
  }
  SBPL_PRINTF("hash table histogram: 0:%d, <50:%d, <100:%d, <200:%d, <300:%d, <400:%d >400:%d\n", s0, s1, s50, s100,
    s200, s300, slarge);
}

