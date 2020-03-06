# hierarchical-planning-sequences

This code is associated with the following paper:

> Martim Brandao and Ioannis Havoutis, "**Learning Sequences of Approximations for Hierarchical Motion Planning**", *ICAPS 2020*.

## About

Hierarchical motion planning typically solves complex problems by the help of solutions to approximate problems.

What we do here is to:

1) Automatically extract multiple possible sub-spaces of the original state-space of a motion planning problem, including their approximate collision-checking geometry (see test-subspace-models.cpp)
2) Represent the sequencing of approximations as a graph (see ArchitectureSearch.h / cpp)
3) Optimize the parameters of this graph (which lead to a sequence of approximations and a separate computation-time-budget for each approximation), using the evolutionary algorithm NSGA2 (see deap-arch-es.py)

Please read the paper for further details.

## Compile

First do:

```
cd 3rdparty
git clone https://github.com/ANYbotics/grid_map.git
git clone https://github.com/recastnavigation/recastnavigation.git
```

Then install these dependencies:

- [OpenRAVE](https://github.com/rdiankov/openrave)
- [Trajopt (my fork)](https://github.com/martimbrandao/trajopt)
- [SBPL](https://github.com/sbpl/sbpl)
- [PCL 1.8](https://github.com/PointCloudLibrary/pcl)
- DEAP (pip install deap)
- SCOOP (pip install scoop)

Then compile:

```
mkdir build
cd build
cmake ..
make -j
```

## Note

Unfortunately the point cloud of the environment used in the paper's experiments is proprietary. Please replace any mentions to "fsc-oil-rig-map-full-1cm-clean.pcd" by your own environment point cloud.

## Citation

If you use this code in your research, please cite the paper above.
