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
  // model
  std::vector<flp::Primitive> pmodelSphereBody;
  {
    flp::Primitive body;
    body.type = flp::PrimitiveTypeSphere;
    body.sphere.center << 0.0, 0.0, 0.05;
    body.sphere.radius = 0.30;
    pmodelSphereBody.push_back(body);
  }
  std::vector<flp::Primitive> pmodelSphereCont;
  {
    flp::Primitive c;
    c.type = flp::PrimitiveTypeSphere;
    c.sphere.radius = 0.30;
    c.sphere.center <<  0.0, 0.0, -0.54; pmodelSphereCont.push_back(c);
  }

  // model
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

  // model
  std::vector<flp::Primitive> pmodelCyliBody;
  {
    flp::Primitive body;
    body.type = flp::PrimitiveTypeCylinder;
    body.cylinder.center << 0.0, 0.0, 0.05;
    body.cylinder.radius = 0.30;
    body.cylinder.height = 0.30;
    pmodelCyliBody.push_back(body);
  }
  std::vector<flp::Primitive> pmodelCyliCont = pmodelCyliBody;
  pmodelCyliCont[0].cylinder.center(2) = -0.54;

  // model
  std::vector<flp::Primitive> pmodelCyliBody2 = pmodelCyliBody;
  pmodelCyliBody2[0].cylinder.radius = 0.5;
  std::vector<flp::Primitive> pmodelCyliCont2 = pmodelCyliCont;
  pmodelCyliCont2[0].cylinder.radius = 0.5;

  // model
  std::vector<flp::Primitive> pmodelFeetBodyCyl;
  {
    flp::Primitive body;
    body.type = flp::PrimitiveTypeCylinder;
    body.cylinder.center << 0.2, 0.0, 0.05;
    body.cylinder.radius = 0.30;
    body.cylinder.height = 0.30;
    pmodelFeetBodyCyl.push_back(body);
    body.cylinder.center(0) = 0.0;
    pmodelFeetBodyCyl.push_back(body);
    body.cylinder.center(0) =-0.2;
    pmodelFeetBodyCyl.push_back(body);
  }
  std::vector<flp::Primitive> pmodelFeetContCyl = pmodelFeetBodyCyl;
  pmodelFeetContCyl[0].cylinder.center(2) = -0.54;
  pmodelFeetContCyl[1].cylinder.center(2) = -0.54;
  pmodelFeetContCyl[2].cylinder.center(2) = -0.54;


  #ifndef FLPLIGHTWEIGHT

  // viewer
  Viewer::Ptr viewer;
  viewer.reset( new ViewerTrajopt() ); // ViewerTrajopt("anymal_drs.zae")

  // show 0
  viewer->drawAndIdle();

  // show 1
  viewer->clearPrimitives();
  viewer->addPrimitives(pmodelSphereBody);
  viewer->addPrimitives(pmodelSphereCont);
  viewer->drawAndIdle();

  // show 2
  viewer->clearPrimitives();
  viewer->addPrimitives(pmodelCyliBody);
  viewer->addPrimitives(pmodelCyliCont);
  viewer->drawAndIdle();

  // show 3
  viewer->clearPrimitives();
  viewer->addPrimitives(pmodelCyliBody2);
  viewer->addPrimitives(pmodelCyliCont2);
  viewer->drawAndIdle();

  // show 4
  viewer->clearPrimitives();
  viewer->addPrimitives(pmodelFeetBody);
  viewer->addPrimitives(pmodelFeetCont);
  viewer->drawAndIdle();  

  // show 5
  viewer->clearPrimitives();
  viewer->addPrimitives(pmodelFeetBodyCyl);
  viewer->addPrimitives(pmodelFeetContCyl);
  viewer->drawAndIdle();

  #else

  std::cout << "WARNING: library was compiled lightweight, viewer will not be used\n";

  #endif

  return 0;
}

