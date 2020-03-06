#include "ArchitectureSearch.h"
#include "ArchitectureSearchAnymal.h"
#include "RobotModelFixedPrimitives.h"
#include "EnvironmentConversions.h"
#include <grid_map_core/grid_map_core.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>

#ifndef FLPLIGHTWEIGHT
  #include "ViewerTrajopt.h"
#endif

std::string to_string(const std::vector<double>& vec)
{
  std::stringstream ss;
  for (unsigned int i = 0; i < vec.size(); i++)
    ss << vec[i] << " ";
  return ss.str();
}

int main(int argc, char *argv[])
{
  std::string strEnvironment("fsc");

  // test Anymal baselines
  printf("testing anymal BFS baseline...\n");

  arch::PlannerEvaluatorAnymal anymal;
  anymal.setParamsBaseline("BFS");
  flp::Graph anymalGraph = anymal.getGraph();
  anymalGraph.savepng("BFS.png");

  printf("done.\n");

  // debug mode ?
  bool debug = false;
  if (argc >= 2) {
    std::string a = argv[1];
    if (a == "debug")
      debug = true;
  }

  // load environment
  EnvironmentGridMap::Ptr environment( new EnvironmentGridMap() );
  if (strEnvironment == "fsc") {

    // FSC environment
    // load PCL file
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud( new pcl::PointCloud<pcl::PointXYZRGB>() );
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB> ("fsc-oil-rig-map-full-1cm-clean.pcd", *cloud) == -1) {
      PCL_ERROR ("Couldn't read .pcd file \n");
      printf("Couldn't read .pcd file \n");
      return -1;
    }
    // chop off top floor
    std::cout << "Cropping Z...\n";
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered( new pcl::PointCloud<pcl::PointXYZRGB>() );
    cloud_filtered->points.reserve(cloud->points.size());
    for (unsigned int i = 0; i < cloud->points.size(); i++) {
      const pcl::PointXYZRGB &pt = cloud->points[i];
      if (pt.x >=   0.0 && pt.x <= 40.0 &&
          pt.y >=   0.0 && pt.y <= 80.0 &&
          pt.z >= -10.0 && pt.z <=  1.0) {
        cloud_filtered->points.push_back(pt);
      }
    }
    // convert to gridmap
    std::cout << "Converting to grid_map...\n";
    grid_map::GridMap gridmap({"elevation"});
    convert(cloud_filtered, gridmap, 0.05);
    // fix some issues with the map
    for (grid_map::SpiralIterator iterator(gridmap, grid_map::Position(9.65,22.3), 0.70); !iterator.isPastEnd(); ++iterator) {
      gridmap.at("elevation", *iterator) = -1.28;
    }
    // save
    environment->map_ = gridmap;

  } else {

    // waves environment
    grid_map::GridMap map({"elevation"});
    map.setFrameId("map");
    map.setGeometry(grid_map::Length(4.0, 4.0), 0.03);
    double t = 100;
    for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
      grid_map::Position position;
      map.getPosition(*it, position);
      map.at("elevation", *it) = -0.6 + 0.05 * std::sin(3.0 * t + 5.0 * position.y());
    }
    environment->map_ = map;

  }

  // show
  Viewer::Ptr viewer;
  if (debug) {
    #ifndef FLPLIGHTWEIGHT
    viewer.reset( new ViewerTrajopt() );
    viewer->updateEnvironment(environment);
    viewer->drawAndIdle();
    #else
    std::cout << "WARNING: library was compiled lightweight, viewer will not be used\n";
    #endif
  }

  // state space units
  arch::VirtualStateSpace space;
  space.units.push_back(arch::VirtualStateSpaceUnit("XYZ"));
//space.units.push_back(arch::VirtualStateSpaceUnit("r"));
  space.units.push_back(arch::VirtualStateSpaceUnit("p"));
  space.units.push_back(arch::VirtualStateSpaceUnit("y"));
  space.units.push_back(arch::VirtualStateSpaceUnit("M"));

  // robot models
  arch::Model modelCyli;
  modelCyli.statesAccepted.push_back("XYZ");
  modelCyli.statesAccepted.push_back("M");
  modelCyli.statesDefaultValue["M"] = 0;

  arch::Model modelFeet;
  modelFeet.statesAccepted.push_back("XYZ");
//modelFeet.statesAccepted.push_back("r");
  modelFeet.statesAccepted.push_back("p");
  modelFeet.statesAccepted.push_back("y");
  modelFeet.statesAccepted.push_back("M");
//modelFeet.statesDefaultValue["r"] = 0;
  modelFeet.statesDefaultValue["p"] = 0;
  modelFeet.statesDefaultValue["M"] = 0;

  std::vector<arch::Model> models;
  models.push_back(modelCyli);
  models.push_back(modelFeet);

  // planner robot models
  std::vector<flp::Primitive> pmodelCyliBody;
  {
    flp::Primitive body;
    body.type = flp::PrimitiveTypeSphere;
    body.sphere.center << 0.0, 0.0, 0.05;
    body.sphere.radius = 0.30;
    pmodelCyliBody.push_back(body);
  }
  std::vector<flp::Primitive> pmodelCyliCont;
  {
    flp::Primitive c;
    c.type = flp::PrimitiveTypeSphere;
    c.sphere.radius = 0.30;
    c.sphere.center <<  0.0, 0.0, -0.54; pmodelCyliCont.push_back(c);
  }
  std::vector<flp::Primitive> pmodelFeetBody;
  {
    flp::Primitive body;
    body.type = flp::PrimitiveTypeSphere;
    body.sphere.center << 0.2, 0.0, 0.05;
    body.sphere.radius = 0.30;
    pmodelFeetBody.push_back(body);
    body.sphere.center(0) = 0.0;
    pmodelFeetBody.push_back(body);
    body.sphere.center(0) =-0.2;
    pmodelFeetBody.push_back(body);
  }
  std::vector<flp::Primitive> pmodelFeetCont;
  {
    flp::Primitive c;
    c.type = flp::PrimitiveTypeSphere;
    c.sphere.radius = 0.15;
    c.sphere.center <<  0.33, 0.22, -0.54; pmodelFeetCont.push_back(c);
    c.sphere.center << -0.33, 0.22, -0.54; pmodelFeetCont.push_back(c);
    c.sphere.center <<  0.33,-0.22, -0.54; pmodelFeetCont.push_back(c);
    c.sphere.center << -0.33,-0.22, -0.54; pmodelFeetCont.push_back(c);
  }
  RobotModel::Ptr pmodelCyli( new RobotModelFixedPrimitives(StateSpace::Ptr(), pmodelCyliBody, pmodelCyliCont) );
  RobotModel::Ptr pmodelFeet( new RobotModelFixedPrimitives(StateSpace::Ptr(), pmodelFeetBody, pmodelFeetCont) );
  pmodelCyli->setContactProjectionOffset(Eigen::Vector3d(0,0,0.15));
  pmodelFeet->setContactProjectionOffset(Eigen::Vector3d(0,0,0.05));
  std::vector< std::pair<arch::Model, RobotModel::Ptr> > pmodels;
  pmodels.push_back( std::pair<arch::Model, RobotModel::Ptr>(modelCyli, pmodelCyli) );
  pmodels.push_back( std::pair<arch::Model, RobotModel::Ptr>(modelFeet, pmodelFeet) );

  // planner
  arch::Planner planner;
//planner.nbins_["r"] = std::vector<int>(1, 64);
  planner.nbins_["p"] = std::vector<int>(1, 64);
  planner.nbins_["y"] = std::vector<int>(1, 64);
  planner.nbins_["M"] = std::vector<int>(1, 3);
  planner.models_ = pmodels;
  planner.viewer_ = viewer;
  if (debug) planner.verbose_ = true;

  // environment and start/goal poses
  if (strEnvironment == "fsc") {
    planner.environment_ = environment;
    planner.Tstart_ = flp::Transform(8.65,22.5,-1.3, 0.0, 0.0, 1.6); // stair entrance to room
    planner.Tgoal_ =  flp::Transform(8.2, 28.0,-1.25,0.0, 0.0, 1.6);
    planner.setRecast(false);
  } else {
    planner.environment_ = environment;
    planner.Tstart_ = flp::Transform(0,0,0,0,0,0);
    planner.Tgoal_ = flp::Transform(1.5,1.5,0,0.0,-0.15,3.14/2);
    planner.setRecast(false);
  }

  // set bounds and discretization
  std::vector<double> xyzlb, xyzub;
  environment->getSpatialBounds(xyzlb, xyzub); xyzub[2] += 1.0;
  std::vector<int> nbins(3);
  nbins[0] = (xyzub[0] - xyzlb[0]) / 0.05;
  nbins[1] = (xyzub[1] - xyzlb[1]) / 0.05;
  nbins[2] = (xyzub[2] - xyzlb[2]) / 0.05;
  planner.lb_["XYZ"] = xyzlb;
  planner.ub_["XYZ"] = xyzub;
  planner.nbins_["XYZ"] = nbins;

  // random poses
  if (viewer && false) {
    arch::PlannerEvaluatorAnymal evalAny;
    evalAny.setEnvironment("fsc");
    evalAny.setDebug(true);
    evalAny.generateRandomProblems(5);
  }
  if (viewer && false) {
    std::mt19937 gen;
    std::vector<double> lb, ub;
    environment->getSpatialBounds(lb, ub);
    printf("x = %f to %f \n", lb[0], ub[0]);
    printf("y = %f to %f \n", lb[1], ub[1]);
    printf("z = %f to %f \n", lb[2], ub[2]);
    StateSpace::Ptr ss = planner.getStateSpace("XYZ ");
    for (unsigned int i = 0; i < 10; i++) {
      flp::Point pt = environment->sampleUniform(gen);
      printf("New XYZ: %f, %f, %f \n", pt.p(0), pt.p(1), pt.p(2));
      flp::Transform T(pt.p(0), pt.p(1), pt.p(2), 0, 0, 0);
      State::Ptr state = ss->getStateZero();
      ss->setTransform(state, T);
      State::Ptr proj = pmodelCyli->project(state, environment);
      viewer->clearPrimitives();
      viewer->addPrimitives(pmodelCyli->getPrimitives(proj));
      viewer->drawAndIdle();
    }
  }

  // architecture graph
  std::vector<double> defaultEdgeParameters(1,0);

  flp::Graph g = arch::getAllValidSubspaces(space, models, defaultEdgeParameters);
  std::vector<double> vec = g.getEdgesVector();
  printf("Num nodes: %d \n", g.getNumberOfNodes());
  printf("Num edges: %d \n", g.getNumberOfEdges());
  printf("VectorSiz: %d \n", (int)vec.size());
  printf("Vector   : %s \n", to_string(vec).c_str());
  g.savepdf("arch-graph.pdf");

  // random vector
  std::mt19937 gen;
  std::uniform_int_distribution<int> distribution(0, 100);
  for (unsigned int i = 0; i < vec.size(); i++) {
    vec[i] = distribution(gen);
  }
  g.setEdgesVector(vec);
  printf("VectorRnd: %s \n", to_string(vec).c_str());
  g.savepdf("arch-graph-rand.pdf");

  std::vector<int> path = g.getShortestPath(0, 1);
  g.setColorPath(path, "green");
  g.savepdf("arch-graph-rand-colored.pdf");

  // find path
  printf("\nOnline shortest-path:\n");
  flp::Graph graphUpdated;
  flp::Graph graphDraw;
  std::vector<int> graphPath;
  bool found = arch::findShortestPathOnline(g, 0, 1, planner, graphUpdated, graphDraw, graphPath);
  graphDraw.savepdf("arch-graph-rand-execution.pdf");

  return 0;
}

