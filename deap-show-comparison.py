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
        if len(fval) > 0: valuesfile = fval[-1]
    else:
        print("This path does not exist.")
        sys.exit()
    pop = []
    with open(valuesfile, "rb") as f:
        reader = csv.reader(f)
        for i, line in enumerate(reader):
            vals = [float(l) for l in line]
            pop.append(vals)
    pop = [ind for ind in pop if ind[0] < 1e6 and ind[1] < 1e6]
    v = np.array(pop)
    if len(v) == 0:
        return v
    return v[v[:,0].argsort()] # sort according to x (for pareto curve)

# main
def main(argv):
    # get arguments
    vals_opt = []
    vals_baseline1 = []
    vals_baseline2 = []
    vals_baseline3 = []
    try:
        opts, args = getopt.getopt(argv,"o:1:2:")
    except getopt.GetoptError:
        print 'deap-show-comparison.py -o <opt_dir> -1 <baseline1_dir> -2 <baseline2_dir> -3 <baseline3_dir>'
        sys.exit(2)
    for opt, arg in opts:
        if opt == "-o":
            vals_opt = getvals(arg)
        elif opt == "-1":
            vals_baseline1 = getvals(arg)
        elif opt == "-2":
            vals_baseline2 = getvals(arg)
        elif opt == "-3":
            vals_baseline3 = getvals(arg)
    if len(vals_opt) == 0:
        return

    # plot
    plt.clf()
    plt.plot(vals_opt[:,0], vals_opt[:,1], "r")
    if len(vals_baseline1) != 0: plt.plot(vals_baseline1[:,0], vals_baseline1[:,1], "g.")
    if len(vals_baseline2) != 0: plt.plot(vals_baseline2[:,0], vals_baseline2[:,1], "b.")
    if len(vals_baseline3) != 0: plt.plot(vals_baseline3[:,0], vals_baseline3[:,1], "c.")
    plt.xlabel("Time")
    plt.ylabel("Cost")
    plt.tight_layout()
    plt.savefig('tmp-comparison.png', transparent=True)
    plt.show()

    return
    
if __name__ == "__main__":
    main(sys.argv[1:])
