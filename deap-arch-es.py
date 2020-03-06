# run: python -m scoop [this-file.py]

import array
import random
import pdb
import csv
import sys, getopt, os
import time
import numpy as np
import imp

from deap import algorithms
from deap import base
from deap import benchmarks
from deap import creator
from deap import tools
from scoop import futures, shared
from scoop import IS_RUNNING as SCOOP_IS_RUNNING
from libArchitectureSearch import PlannerEvaluatorDummy
from libArchitectureSearch import PlannerEvaluatorAnymal
from libArchitectureSearch import PlannerEvaluatorStarleth

# arguments
PLOTTING = True

# whether arguments have been processed and global variables configured
CONFIGURED = False

# mu number of individuals to select, lambda number of children to produce
MU, LAMBDA = 20, 100 # 20,100 works well... 20/200 also

# Configure global variables (master)
def configureMaster():
    global CONFIGURED
    if CONFIGURED:
      return
    # default parameters
    global MYSYSTEM, NGEN, N_RANDOM_PROBLEMS, RANDOM_SEED, FILE_POPULATION, USE_RECAST, BASELINE, RUN_OPT, OUTPUT_FOLDER
    MYSYSTEM = 'anymal'
    NGEN = 20
    N_RANDOM_PROBLEMS = 0
    RANDOM_SEED = 5489
    FILE_POPULATION = ''
    USE_RECAST = False
    BASELINE = ''
    RUN_OPT = True
    # get arguments
    try:
      opts, args = getopt.getopt(sys.argv[1:],"s:n:g:e:b:l:rc")
    except getopt.GetoptError:
      print("deap-arch-es.py")
      print("  -s <SYSTEM>: the sytem we plan on 'anymal', 'starleth', or 'dummy'")
      print("  -n <NGEN>  : how many generations to run the optimization")
      print("  -g <NPROBS>: how many random problems to generate")
      print("  -e <RSEED> : random seed for problems to generate")
      print("  -b <NAME>  : use baseline method with this name")
      print("  -l <FILE>  : load the initial population from a file")
      print("  -r         : whether to use recast")
      print("  -c         : only check/evaluate population (skip optimization)")
      sys.exit(2)
    # parse arguments (normal)
    for opt, arg in opts:
      if opt == "-s":
        MYSYSTEM = arg
      elif opt == "-n":
        NGEN = int(arg)
      elif opt == "-g":
        N_RANDOM_PROBLEMS = int(arg)
      elif opt == "-e":
        RANDOM_SEED = int(arg)
      elif opt == "-b":
        BASELINE = arg
      elif opt == "-l":
        FILE_POPULATION = arg
      elif opt == "-r":
        USE_RECAST = True
      elif opt == "-c":
        RUN_OPT = False
    # output folder
    OUTPUT_FOLDER = "%s-%s-%s-randprobs%s-%s/" % (BASELINE if BASELINE != '' else "opt", MYSYSTEM, "recast" if USE_RECAST else "norecast", N_RANDOM_PROBLEMS, time.strftime("%Y%m%d-%H%M%S"))
    if not os.path.exists(OUTPUT_FOLDER):
      os.mkdir(OUTPUT_FOLDER)
    # share parameters with SCOOP workers
    if SCOOP_IS_RUNNING:
      shared.setConst(MYSYSTEM=MYSYSTEM)
      shared.setConst(N_RANDOM_PROBLEMS=N_RANDOM_PROBLEMS)
      shared.setConst(RANDOM_SEED=RANDOM_SEED)
      shared.setConst(USE_RECAST=USE_RECAST)
      shared.setConst(BASELINE=BASELINE)
      shared.setConst(OUTPUT_FOLDER=OUTPUT_FOLDER)
    # finish
    configureEvaluator()
    CONFIGURED = True

# Configure global variables (SCOOP)
def configureScoop():
    global CONFIGURED
    if CONFIGURED or not SCOOP_IS_RUNNING:
      return
    global MYSYSTEM, N_RANDOM_PROBLEMS, RANDOM_SEED, USE_RECAST, BASELINE, OUTPUT_FOLDER
    MYSYSTEM = shared.getConst('MYSYSTEM')
    N_RANDOM_PROBLEMS = shared.getConst('N_RANDOM_PROBLEMS')
    RANDOM_SEED = shared.getConst('RANDOM_SEED')
    USE_RECAST = shared.getConst('USE_RECAST')
    BASELINE = shared.getConst('BASELINE')
    OUTPUT_FOLDER = shared.getConst('OUTPUT_FOLDER')
    # finish
    configureEvaluator()
    CONFIGURED = True

# Configure evaluator and problem properties
def configureEvaluator():
    # create evaluator
    global EVALUATOR
    if MYSYSTEM == 'anymal':
      EVALUATOR = PlannerEvaluatorAnymal()
      EVALUATOR.setEnvironment("fsc")
      EVALUATOR.setRecast(USE_RECAST)
      EVALUATOR.setSeed(RANDOM_SEED)
    elif MYSYSTEM == 'starleth':
      EVALUATOR = PlannerEvaluatorStarleth()
      EVALUATOR.setEnvironment("fsc")
      EVALUATOR.setRecast(USE_RECAST)
      EVALUATOR.setSeed(RANDOM_SEED)
    else:
      EVALUATOR = PlannerEvaluatorDummy()
    EVALUATOR.setDebug(False)
    # optionals
    if BASELINE != '': EVALUATOR.setParamsBaseline(BASELINE)
    if N_RANDOM_PROBLEMS > 0: EVALUATOR.generateRandomProblems(N_RANDOM_PROBLEMS)
    # problem properties
    global N, LB, UB, LB_STRATEGY, UB_STRATEGY
    N = EVALUATOR.getDimension()
    LB = EVALUATOR.getLowerBounds()
    UB = EVALUATOR.getUpperBounds()
    LB_STRATEGY = [10]*N     # standard deviation of the mutation
    UB_STRATEGY = [10000]*N  # standard deviation of the mutation

# Individual generator
def generateES(icls, scls):
    vals = []
    for i in range(N):
        vals.append(random.uniform(LB[i],UB[i]))
    vals_strategy = []
    for i in range(N):
        vals_strategy.append(random.uniform(LB_STRATEGY[i],UB_STRATEGY[i]))
    ind = icls(vals)
    ind.strategy = scls(vals_strategy)
    return ind

# Force bounds on individual
def checkBounds(lb, ub):
    def decorator(func):
        def wrapper(*args, **kargs):
            offspring = func(*args, **kargs)
            for child in offspring:
                for i in xrange(len(child)):
                    if child[i] > ub[i]:
                        child[i] = ub[i]
                    elif child[i] < lb[i]:
                        child[i] = lb[i]
            return offspring
        return wrapper
    return decorator

# Force lower bound on strategy (i.e. on the standard deviation of the mutation)
def checkStrategy(lb, ub):
    def decorator(func):
        def wrappper(*args, **kargs):
            children = func(*args, **kargs)
            for child in children:
                for i, s in enumerate(child.strategy):
                    if s < lb[i]:
                        child.strategy[i] = lb[i]
                    if s > ub[i]:
                        child.strategy[i] = ub[i]
            return children
        return wrappper
    return decorator

# Fitness function
def fitness(individual):
    configureScoop()
    #print(MYSYSTEM, N_RANDOM_PROBLEMS, RANDOM_SEED, USE_RECAST, BASELINE, N)
    EVALUATOR.evaluate(list(individual))
    fsucc = EVALUATOR.getLastEvaluationSuccessRate()
    ftime = EVALUATOR.getLastEvaluationTime()
    fcost = EVALUATOR.getLastEvaluationCost()
    return ftime, fcost,

# plot functions
def monitorPlot(values, filename=None):
    if not PLOTTING:
      return []
    v = np.array(values)
    if v.shape != (MU,2,):
        return []
    plt.clf()
    v = v[v[:,0].argsort()]
    plt.plot(v[:,0], v[:,1], "r")
    plt.xlabel("Time")
    plt.ylabel("Cost")
    plt.tight_layout()
    plt.draw()
    if filename == None:
      plt.savefig(OUTPUT_FOLDER+"esnsga-pareto-"+time.strftime("%Y%m%d-%H%M%S")+".png", transparent=True)
    else:
      plt.savefig(filename, transparent=True)
    plt.pause(0.001)
    return []

def saveIndividualArchitectureFigure(ind, i):
    configureScoop()
    EVALUATOR.evaluate(list(ind))
    fsucc = EVALUATOR.getLastEvaluationSuccessRate()
    ftime = EVALUATOR.getLastEvaluationTime()
    fcost = EVALUATOR.getLastEvaluationCost()
    filename = OUTPUT_FOLDER+"esnsga-pareto-ind-time%06d-cost%06d-i%02d.png"%(ftime*1000,fcost*1000,i)
    EVALUATOR.saveLast(filename, "png")
    return filename

# stats
def statAvg(pop):
    fit = np.array([pop[i].fitness.values for i in range(len(pop))])
    return np.mean(fit, axis=0)
def statMin(pop):
    fit = np.array([pop[i].fitness.values for i in range(len(pop))])
    return np.min(fit, axis=0)
def statMax(pop):
    fit = np.array([pop[i].fitness.values for i in range(len(pop))])
    return np.max(fit, axis=0)
def statLog(pop):
    fit = np.array([pop[i].fitness.values for i in range(len(pop))])
    # plot pareto
    strtime = time.strftime("%Y%m%d-%H%M%S")
    monitorPlot(fit, filename = OUTPUT_FOLDER+"esnsga-pareto-"+strtime+"-graph.png")
    # save population
    f = open(OUTPUT_FOLDER+"esnsga-pareto-"+strtime+"-population.csv", "w")
    writer = csv.writer(f, lineterminator = "\n")
    for ind in pop:
        ext = array.array('d',ind)
        ext.extend(ind.strategy)
        writer.writerow(ext)
    f.close()
    # save fitness
    f = open(OUTPUT_FOLDER+"esnsga-pareto-"+strtime+"-values.csv", "w")
    writer = csv.writer(f, lineterminator = "\n")
    for ind in pop:
        writer.writerow(ind.fitness.values)
    f.close()
    return []

# Plotting
if PLOTTING:
  try:
    imp.find_module('matplotlib')
    from matplotlib import pyplot as plt
    plt.style.use('seaborn-white')
    plt.rcParams['font.family'] = 'serif'
    plt.rcParams['font.serif'] = 'Ubuntu'
    plt.rcParams['font.monospace'] = 'Ubuntu Mono'
    plt.rcParams['font.size'] = 10*2
    plt.rcParams['axes.labelsize'] = 10*2
    plt.rcParams['axes.labelweight'] = 'bold'
    plt.rcParams['axes.titlesize'] = 10*2
    plt.rcParams['lines.linewidth'] = 3
    plt.rcParams['xtick.labelsize'] = 8*2
    plt.rcParams['ytick.labelsize'] = 8*2
    plt.rcParams['legend.fontsize'] = 10*2
    plt.rcParams['figure.titlesize'] = 12*2
    plt.rcParams['axes.formatter.useoffset'] = False
  except ImportError:
    PLOTTING = False

# configure DEAP
creator.create("FitnessMax", base.Fitness, weights=(-1.0,-1.0,))
creator.create("Individual", array.array, typecode="d", fitness=creator.FitnessMax, strategy=None)
creator.create("Strategy", array.array, typecode="d")

# main
def main(argv):
    random.seed(0)

    # configure
    configureMaster()

    # configure DEAP
    toolbox = base.Toolbox()
    toolbox.register("individual", generateES, creator.Individual, creator.Strategy)
    toolbox.register("population", tools.initRepeat, list, toolbox.individual)
    toolbox.register("evaluate", fitness)

    toolbox.register("mate", tools.cxESBlend, alpha=0.1)
    toolbox.register("mutate", tools.mutESLogNormal, c=1.0, indpb=0.2)
    toolbox.decorate("mate", checkStrategy(LB_STRATEGY, UB_STRATEGY))
    toolbox.decorate("mutate", checkStrategy(LB_STRATEGY, UB_STRATEGY))
    toolbox.decorate("mate", checkBounds(LB, UB))
    toolbox.decorate("mutate", checkBounds(LB, UB))

    toolbox.register("map", futures.map)
    toolbox.register("select", tools.selNSGA2) # selSPEA2 or selNSGA2

    # log
    mstats = tools.Statistics()
    mstats.register("avg", statAvg)
    mstats.register("min", statMin)
    mstats.register("max", statMax)
    mstats.register("log", statLog)

    # population
    if FILE_POPULATION == '':
        pop = toolbox.population(n=LAMBDA)
    else:
        pop = []
        with open(FILE_POPULATION, "rb") as f:
            reader = csv.reader(f)
            for i, line in enumerate(reader):
                vals = [float(l) for l in line]
                ind = creator.Individual(array.array('d', vals[:len(vals)/2]))
                ind.strategy = creator.Individual(array.array('d', vals[len(vals)/2:]))
                pop.append(ind)

    # a little help on one individual (max computation times and reasonable robot radius)
    if MYSYSTEM == 'starleth' and BASELINE == '' and FILE_POPULATION == '':
      vals = LB
      vals[1::3] = UB[1::3] # maxTime
      vals[2::3] = [0.3 * 100/0.75]*(len(vals)/3) # inscribed radius
      vals[8] = 0.5 * 100/0.75 # circumscribed radius
      ind = creator.Individual(array.array('d', vals))
      ind.strategy = creator.Individual(array.array('d', [random.uniform(LB_STRATEGY[i],UB_STRATEGY[i]) for i in range(N)]))
      pop.insert(0,ind)
      # another one, slighty faster
      vals[1::3] = (UB[1::3] - LB[1::3]) / 2 # maxTime/2
      ind = creator.Individual(array.array('d', vals))
      ind.strategy = creator.Individual(array.array('d', [random.uniform(LB_STRATEGY[i],UB_STRATEGY[i]) for i in range(N)]))
      pop.insert(0,ind)

    #elif MYSYSTEM == 'anymal':
    #  vals = LB
    #  vals[1::2] = UB[1::2] # maxTime
    #  ind = creator.Individual(array.array('d', vals))
    #  ind.strategy = creator.Individual(array.array('d', [random.uniform(LB_STRATEGY[i],UB_STRATEGY[i]) for i in range(N)]))
    #  pop.insert(0,ind)

    # to debug a specific individual
    #de = pop[12]
    #pop = [de]
    #print(pop)

    hof = tools.HallOfFame(1)

    # run algorithm
    if RUN_OPT:
      # eaMuPlusLambda chooses next generation from offspring AND population
      # eaMuCommaLambda chooses next generation from offspring ONLY
      pop, logbook = algorithms.eaMuPlusLambda(pop, toolbox, mu=MU, lambda_=LAMBDA, cxpb=0.6, mutpb=0.3, ngen=NGEN, stats=mstats, halloffame=hof)

    # save all the individuals
    print("Saving all individuals' architecture figures...")
    if SCOOP_IS_RUNNING:
      data = list(futures.map(saveIndividualArchitectureFigure, pop, range(len(pop))))
    else:
      data = list(map(saveIndividualArchitectureFigure, pop, range(len(pop))))
    print("saved:\n"+"\n".join(data))

    # hold the plot
    if PLOTTING:
      print "If there is data on a plot I will show it now for the last time (save it now or lose it)"
      plt.savefig(OUTPUT_FOLDER+'pareto-final.png', transparent=True)
      plt.show()

    # finish
    return pop, logbook, hof
    
if __name__ == "__main__":
    main(sys.argv[1:])
