import array
import pdb
import csv
import sys, getopt, os, glob
import numpy as np

from matplotlib import pyplot as plt
from matplotlib.image import imread

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

# get fitness values
def getvals(path):
    if os.path.exists(path):
        fval = sorted(glob.glob(path+'/*-values.csv'))
        if len(fval) == 0:
            return []
        valuesfile = fval[-1]
    else:
        print("This path does not exist.")
        sys.exit()
    pop = []
    with open(valuesfile, "rb") as f:
        reader = csv.reader(f)
        for i, line in enumerate(reader):
            vals = [float(l) for l in line]
            pop.append(vals)
    #pop = [ind for ind in pop if ind[0] < 1e6 and ind[1] < 1e6]
    v = np.array(pop)
    if len(v) == 0:
        return v
    return v[v[:,0].argsort()] # sort according to x (for pareto curve)

# main
if __name__ == "__main__":
    if len(sys.argv) == 1:
        print("Usage: deap-save-figs.py [RESULTS_BASE_DIR]")
        sys.exit()
    base = sys.argv[1]
    folders = glob.glob(base+'/*/')
    folders = sorted(folders, reverse=True)
    # table head
    print("%24s %s %s %s" % ("algorithm", "succ", "minT", "minC"))
    # loop over algorithms
    data = []
    name = []
    for folder in folders:
        valsA= getvals(folder)
        vals = valsA[ np.all(valsA, axis=1) < 1e6 , :]
        # compute success rate
        Nprobs = 6
        failC = 1e10/Nprobs
        valsSuccesses = valsA - failC * np.floor(valsA/failC)
        rate = (Nprobs - np.floor(valsA/failC)[0,0]) / Nprobs # assume number is the same for all points
        # compute utopia point
        utopia = np.min(valsSuccesses, axis=0)
        #if len(vals) == 0:
        #    continue
        # success rates
        vals = valsSuccesses
        # plot
        label = os.path.basename(os.path.normpath(base+'/'+folder))
        label = label.split('-')[0]
        data.append(vals)
        name.append(label)
        plt.clf()
        plt.plot(vals[:,0], vals[:,1], "r", marker='o')
        plt.xlabel("Time")
        plt.ylabel("Cost")
        plt.tight_layout()
        plt.draw()
        plt.savefig(folder+'/tmp-pareto.png', transparent=True)
        # table
        print("%24s %3.2f %4.2f %4.2f%s" % (label, rate, utopia[0], utopia[1], "" if rate==1 else "*"))
    if len(data) > 0:
        plt.clf()
        for i in range(len(data)):
            vals = data[i]
            plt.plot(vals[:,0], vals[:,1], label=name[i], marker='o')
        plt.xlabel("Time")
        plt.ylabel("Cost")
        plt.legend()
        plt.tight_layout()
        plt.draw()
        plt.savefig(base+'/tmp-comparison.png', transparent=True)
