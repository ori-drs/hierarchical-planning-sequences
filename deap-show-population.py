import array
import pdb
import csv
import sys, getopt, os, glob
import numpy as np

from matplotlib import pyplot as plt
from matplotlib.image import imread
from libArchitectureSearch import PlannerEvaluatorDummy
from libArchitectureSearch import PlannerEvaluatorAnymal
from libArchitectureSearch import PlannerEvaluatorStarleth

# Plotting
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

# plot functions
def monitorPlot(values, filename=None):
    v = np.array(values)
    if v.shape[1] != 2:
        return []
    plt.clf()
    v = v[v[:,0].argsort()]
    plt.plot(v[:,0], v[:,1], "r")
    plt.xlabel("Time")
    plt.ylabel("Cost")
    plt.tight_layout()
    if filename != None:
      plt.savefig(filename, transparent=True)
    plt.show()

# main
def main(argv):
    # get arguments
    folder = ''
    loadfile = ''
    valuesfile = ''
    try:
        opts, args = getopt.getopt(argv,"l:v:d:")
    except getopt.GetoptError:
        print 'deap-show-population.py -l <population_file_csv> -v <values_file_csv> -d <dir_to_get_latest_files_from>'
        sys.exit(2)
    for opt, arg in opts:
        if opt == "-l" and loadfile == '':
            # load the population from custom file
            loadfile = arg
        elif opt == "-v" and valuesfile == '':
            # load the fitness values from custom file
            valuesfile = arg
        elif opt == "-d":
            # load the population and fitness values from the most recent log (last iteration)
            if os.path.exists(arg):
                folder = arg + '/'
                fpop = sorted(glob.glob(arg+'/*-population.csv'))
                fval = sorted(glob.glob(arg+'/*-values.csv'))
                if len(fpop) > 0: loadfile = fpop[-1]
                if len(fval) > 0: valuesfile = fval[-1]
            else:
                print("This path does not exist.")
                sys.exit()
    if loadfile == '':
        sys.exit()

    # population
    pop = []
    with open(loadfile, "rb") as f:
        reader = csv.reader(f)
        for i, line in enumerate(reader):
            vals = [float(l) for l in line]
            pop.append(vals)

    # evaluator
    MYSYSTEM = 'anymal'
    if MYSYSTEM == 'anymal':
      EVALUATOR = PlannerEvaluatorAnymal()
      EVALUATOR.setEnvironment("fsc")
      #EVALUATOR.generateRandomProblems(5)
    elif MYSYSTEM == 'starleth':
      EVALUATOR = PlannerEvaluatorStarleth()
      EVALUATOR.setEnvironment("fsc")
      EVALUATOR.setParamsBaseline()
    else:
      EVALUATOR = PlannerEvaluatorDummy()
    EVALUATOR.setDebug(False)

    # fitness values
    fit = []
    if valuesfile == '':
        # compute fitness
        for i, indstrategy in enumerate(pop):
            ind = indstrategy[:len(indstrategy)/2]
            EVALUATOR.evaluate(list(ind))
            fsucc = EVALUATOR.getLastEvaluationSuccessRate()
            ftime = EVALUATOR.getLastEvaluationTime()
            fcost = EVALUATOR.getLastEvaluationCost()
            vals = [ftime, fcost]
            fit.append(vals)
    else:
        # load fitness from file
        with open(valuesfile, "rb") as f:
            reader = csv.reader(f)
            for i, line in enumerate(reader):
                vals = [float(l) for l in line]
                fit.append(vals)
    fit = np.array(fit)

    # plot pareto
    monitorPlot(fit, folder+"tmp-pareto.png")

    # show graphs and debug planner
    runPlanner = False # set to True if you want to see execution-colored graph (visited, failed and executed edges)
    debug = False # set to True if you want to visualize planner executions. NOTE: timings will be slower because of vis. so graph path might be unexpected
    if runPlanner:
        EVALUATOR.setDebug(debug)
        for i, indstrategy in enumerate(pop):
            ind = indstrategy[:len(indstrategy)/2]
            print("-----------")
            print("idx: %d" % i)
            print("individual: "+str(ind))
            print("fitness: "+str(fit[i,:]))
            filename = "tmp-ind%03d.png"%i
            EVALUATOR.evaluate(list(ind))
            EVALUATOR.saveLast(filename, "png") # this way we draw the graph with execution-related colours (visited, failed and executed edges)
            #plt.figure(i)
            plt.clf()
            plt.imshow(imread(filename))
            plt.draw()
            plt.pause(0.1)
            #plt.show()
    else:
        for i, indstrategy in enumerate(pop):
            ind = indstrategy[:len(indstrategy)/2]
            print("-----------")
            print("idx: %d" % i)
            print("individual: "+str(ind))
            print("fitness: "+str(fit[i,:]))
            filename = "tmp-ind%03d.png"%i
            EVALUATOR.save(list(ind), filename, "png") # this way we only draw the graph and shortest-path
            #plt.figure(i)
            plt.clf()
            plt.imshow(imread(filename))
            plt.draw()
            plt.pause(0.1)
            #plt.show()

    sys.exit()
    return
    
if __name__ == "__main__":
    main(sys.argv[1:])
