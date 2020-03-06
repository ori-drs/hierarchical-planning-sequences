#include "ViewerRAVE.h"
#include <pcl/common/common.h>

ViewerRAVEBase::ViewerRAVEBase()
{
  init("");
}

ViewerRAVEBase::ViewerRAVEBase(const std::string& robotModelPath)
{
  init(robotModelPath);
}

void ViewerRAVEBase::init(const std::string& robotModelPath)
{
  OpenRAVE::RaveInitialize(true, false ? OpenRAVE::Level_Debug : OpenRAVE::Level_Info);
  raveEnv_ = OpenRAVE::RaveCreateEnvironment();
  raveEnv_->StopSimulation();
  if (robotModelPath.length() > 0)
    raveEnv_->Load(robotModelPath);

  // get OpenRAVE robot pointer
  std::vector<OpenRAVE::RobotBasePtr> robots;
  raveEnv_->GetRobots(robots);
  if (robots.size() == 0)
    return;
  raveRobot_ = robots[0];

  // get state space
  std::vector<double> Jlb, Jub;
  raveRobot_->GetDOFLimits(Jlb, Jub);
  std::vector<int> Jnb(Jlb.size(),0);
  std::vector<double> XYZlb(3,-1e6);
  std::vector<double> XYZub(3, 1e6);
  std::vector<int> XYZnb(3,0);
  int SO3nb(0);
  robotStateSpace_ = StateSpaceJointsSE3::Ptr( new StateSpaceJointsSE3(Jlb,Jub,Jnb,XYZlb,XYZub,XYZnb,SO3nb) );
}

void ViewerRAVEBase::removeMesh()
{
  if (raveMesh_)
    raveEnv_->Remove(raveMesh_);
}

void ViewerRAVEBase::updateEnvironment(const Environment::Ptr& environment)
{
  EnvironmentPCL::Ptr p1 = boost::dynamic_pointer_cast<EnvironmentPCL>(environment);
  if (p1 != 0) return updateEnvironment(*p1);
  EnvironmentGridMap::Ptr p2 = boost::dynamic_pointer_cast<EnvironmentGridMap>(environment);
  if (p2 != 0) return updateEnvironment(*p2);
  EnvironmentRecast::Ptr p3 = boost::dynamic_pointer_cast<EnvironmentRecast>(environment);
  if (p3 != 0) return updateEnvironment(*p3);
}

void ViewerRAVEBase::updateEnvironment(const EnvironmentPCL& environment)
{
  removeMesh();
  viewer_handle_mesh_.clear();
  viewer_handle_cloud_.reset();

  // add mesh
  if (environment.mesh_ && environment.mesh_->polygons.size() > 0) {

    // create mesh or set of meshes
    raveMesh_ = RaveCreateKinBody(raveEnv_);
    OpenRAVE::TriMesh trimesh;
    trimesh.vertices.resize(environment.mesh_cloud_->points.size());
    for (int i = 0; i < environment.mesh_cloud_->points.size(); i++) {
      trimesh.vertices[i].x = environment.mesh_cloud_->points[i].x;
      trimesh.vertices[i].y = environment.mesh_cloud_->points[i].y;
      trimesh.vertices[i].z = environment.mesh_cloud_->points[i].z;
    }
    trimesh.indices.resize(environment.mesh_->polygons.size()*3);
    for (int i = 0; i < environment.mesh_->polygons.size(); i++) {
      for (int j = 0; j < 3; j++) {
        trimesh.indices[i*3+j] = environment.mesh_->polygons[i].vertices[j];
      }
    }

    // init
    raveMesh_->InitFromTrimesh(trimesh,true);
    raveMesh_->GetLinks()[0]->GetGeometry(0)->SetAmbientColor(OpenRAVE::Vector(0.3,0.3,0.3));
    raveMesh_->GetLinks()[0]->GetGeometry(0)->SetDiffuseColor(OpenRAVE::Vector(0.3,0.3,0.3));

    // add mesh to env
    raveMesh_->SetName("trimesh");
    raveEnv_->Add(raveMesh_);

  }

  // colormap
  const float *r, *g, *b;
  if (environmentColormap_ == ColormapAutumn) {
    r = autumn_r;
    g = autumn_g;
    b = autumn_b;
  } else if (environmentColormap_ == ColormapCool) {
    r = cool_r;
    g = cool_g;
    b = cool_b;
  } else if (environmentColormap_ == ColormapHSV) {
    r = hsv_r;
    g = hsv_g;
    b = hsv_b;
  }
  pcl::PointXYZRGB min_pt, max_pt;
  pcl::getMinMax3D(*environment.cloud_, min_pt, max_pt);
  bool colorHeight = (environmentColorField_=="elevation");

  // create point cloud handler
  unsigned int size = environment.cloud_->size();
  float *pts = new float[3*size];
  float *colors = new float[3*size];
  for (unsigned int i = 0; i < size; i++) {
    pcl::PointXYZRGB &pt = environment.cloud_->points[i];
    pts[i*3+0] = pt.x;
    pts[i*3+1] = pt.y;
    pts[i*3+2] = pt.z;
    if (colorHeight) {
      int colidx = std::max(0, std::min(63, (int)((pt.z - min_pt.z) / (max_pt.z - min_pt.z)*64.)));
      colors[i*3+0] = r[colidx];
      colors[i*3+1] = g[colidx];
      colors[i*3+2] = b[colidx];
    } else {
      colors[i*3+0] = pt.r/255.;
      colors[i*3+1] = pt.g/255.;
      colors[i*3+2] = pt.b/255.;
    }
  }
  viewer_handle_cloud_ = raveEnv_->plot3(pts, size, 3*sizeof(float), 2, colors);
  delete[] pts;
  delete[] colors;
}

void ViewerRAVEBase::updateEnvironment(const EnvironmentGridMap& environment)
{
  viewer_handle_mesh_.clear();
  viewer_handle_cloud_.reset();

  // colormap
  const float *r, *g, *b;
  if (environmentColormap_ == ColormapAutumn) {
    r = autumn_r;
    g = autumn_g;
    b = autumn_b;
  } else if (environmentColormap_ == ColormapCool) {
    r = cool_r;
    g = cool_g;
    b = cool_b;
  } else if (environmentColormap_ == ColormapHSV) {
    r = hsv_r;
    g = hsv_g;
    b = hsv_b;
  }
  const double alpha = environmentAlpha_;

  // params
  const float resolution = environment.map_.getResolution();
  const std::string layerZ = "elevation";
  const std::string layerF = "friction";
  const std::string layerT = "traversability";
  std::string layer;
  if (environment.map_.exists(environmentColorField_))
    layer = environmentColorField_;
  else if (environment.map_.exists(layerT))
    layer = layerT;
  else if (environment.map_.exists(layerF))
    layer = layerF;
  else
    layer = layerZ;

  // compute range
  double minval = std::numeric_limits<double>::infinity();
  double maxval =-std::numeric_limits<double>::infinity();
  const grid_map::Matrix& mapdata = environment.map_[layer];
  for (unsigned int i = 0; i < mapdata.rows(); i++) {
    for (unsigned int j = 0; j < mapdata.cols(); j++) {
      if (!std::isnan(mapdata(i,j))) {
        if (mapdata(i,j) < minval)
          minval = mapdata(i,j);
        if (mapdata(i,j) > maxval)
          maxval = mapdata(i,j);
      }
    }
  }
  std::cout << minval << "\n";
  std::cout << maxval << "\n";

  // aux
  grid_map::Position position;
  grid_map::Index i1,i2,i3;
  bool ok1,ok2,ok3;
  float z1,z2,z3;
  int vcol,vcol1,vcol2,vcol3;
  std::vector<float> pts;
  std::vector<Eigen::Vector4f> col;

  // add cells
  viewer_handle_mesh_.clear();
  for (grid_map::GridMapIterator iterator(environment.map_); !iterator.isPastEnd(); ++iterator) {
    environment.map_.getPosition(*iterator, position);
    float x = position.x();
    float y = position.y();
    float z = environment.map_.at(layerZ, *iterator);
    if (std::isnan(z)) continue;
    // neighbor points
    ok1 = environment.map_.getIndex(grid_map::Position(x+resolution, y           ), i1);
    ok2 = environment.map_.getIndex(grid_map::Position(x           , y+resolution), i2);
    ok3 = environment.map_.getIndex(grid_map::Position(x+resolution, y-resolution), i3);
    if (ok1) z1 = environment.map_.at(layerZ, i1);
    if (ok2) z2 = environment.map_.at(layerZ, i2);
    if (ok3) z3 = environment.map_.at(layerZ, i3);
    // vertex colors
    vcol           = std::max(0, std::min(63, (int)((environment.map_.at(layer, *iterator) - minval)/(maxval-minval)*64.)));
    if (ok1) vcol1 = std::max(0, std::min(63, (int)((environment.map_.at(layer, i1) - minval)/(maxval-minval)*64.)));
    if (ok2) vcol2 = std::max(0, std::min(63, (int)((environment.map_.at(layer, i2) - minval)/(maxval-minval)*64.)));
    if (ok3) vcol3 = std::max(0, std::min(63, (int)((environment.map_.at(layer, i3) - minval)/(maxval-minval)*64.)));
    // triangle up
    if (ok1 && ok2 && !std::isnan(z1) && !std::isnan(z2)) {
      pts.push_back(x);            pts.push_back(y);            pts.push_back(z);
      pts.push_back(x+resolution); pts.push_back(y);            pts.push_back(z1);
      pts.push_back(x);            pts.push_back(y+resolution); pts.push_back(z2);
      col.push_back(Eigen::Vector4f( r[vcol ], g[vcol ], b[vcol ], alpha ));
      col.push_back(Eigen::Vector4f( r[vcol1], g[vcol1], b[vcol1], alpha ));
      col.push_back(Eigen::Vector4f( r[vcol2], g[vcol2], b[vcol2], alpha ));
    }
    // triangle down
    if (ok1 && ok3 && !std::isnan(z1) && !std::isnan(z3)) {
      pts.push_back(x);            pts.push_back(y);            pts.push_back(z);
      pts.push_back(x+resolution); pts.push_back(y);            pts.push_back(z1);
      pts.push_back(x+resolution); pts.push_back(y-resolution); pts.push_back(z3);
      col.push_back(Eigen::Vector4f( r[vcol ], g[vcol ], b[vcol ], alpha ));
      col.push_back(Eigen::Vector4f( r[vcol1], g[vcol1], b[vcol1], alpha ));
      col.push_back(Eigen::Vector4f( r[vcol3], g[vcol3], b[vcol3], alpha ));
    }
  }
  // convert to boost multiarray
  boost::multi_array<float,2> colors(boost::extents[col.size()][4]);
  for (unsigned int i = 0; i < col.size(); i++) {
    colors[i][0] = col[i](0);
    colors[i][1] = col[i](1);
    colors[i][2] = col[i](2);
    colors[i][3] = col[i](3);
  }
  if (pts.size() > 0)
    viewer_handle_mesh_.push_back(raveEnv_->drawtrimesh(&pts[0], 3*sizeof(float), NULL, pts.size()/9, colors));
}

void ViewerRAVEBase::updateEnvironment(const EnvironmentRecast& environment)
{
  removeMesh();
  viewer_handle_mesh_.clear();
  viewer_handle_cloud_.reset();

  // recast get data
  pcl::PolygonMesh::Ptr mesh;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
  std::vector<Eigen::Vector3d> lines;
  if (!environment.recast_.getNavMesh(mesh, cloud, lines) || cloud->points.size() < 3) {
    printf("Recast NavMesh is empty.\n");
    return;
  }

  // create mesh
  raveMesh_ = RaveCreateKinBody(raveEnv_);
  OpenRAVE::TriMesh trimesh;
  trimesh.vertices.resize(cloud->points.size());
  for (int i = 0; i < cloud->points.size(); i++) {
    trimesh.vertices[i].x = cloud->points[i].x;
    trimesh.vertices[i].y = cloud->points[i].y;
    trimesh.vertices[i].z = cloud->points[i].z;
  }
  trimesh.indices.resize(mesh->polygons.size()*3);
  for (int i = 0; i < mesh->polygons.size(); i++) {
    for (int j = 0; j < 3; j++) {
      trimesh.indices[i*3+j] = mesh->polygons[i].vertices[j];
    }
  }

  // init
  raveMesh_->InitFromTrimesh(trimesh,true);
  raveMesh_->GetLinks()[0]->GetGeometry(0)->SetAmbientColor(OpenRAVE::Vector(0.3,0.3,0.3));
  raveMesh_->GetLinks()[0]->GetGeometry(0)->SetDiffuseColor(OpenRAVE::Vector(0.3,0.3,0.3));

  // add mesh to env
  raveMesh_->SetName("trimesh");
  raveEnv_->Add(raveMesh_);

  // triangle lines
  clearLineList();
  addLineList(lines, Eigen::Vector4d(0,0,0,1));
}

void ViewerRAVEBase::updateRobot(const State::Ptr& state)
{
  if (!raveRobot_ || !robotStateSpace_)
    return;
  // get robot state (Joints + SE3)
  StateJointsSE3::Ptr s = boost::dynamic_pointer_cast<StateJointsSE3>(state);
  if (!s) {
    s = boost::dynamic_pointer_cast<StateJointsSE3>( state->space_->project(state, robotStateSpace_) );
  }
  if (!s) {
    std::cout << "ERROR: input state is not displayable (i.e. can't project to StateJointsSE3)\n";
    return;
  }
  // get transform and joints
  flp::Transform T = s->getTransform();
  std::vector<double> joints = boost::static_pointer_cast<StateReal>(s->states_[0])->values_;
  // display
  OpenRAVE::Transform orT(OpenRAVE::Vector(T.qx, T.qy, T.qz, T.qw), OpenRAVE::Vector(T.x, T.y, T.z)); // rot, trans
  raveRobot_->SetTransform(orT);
  if (joints.size() > 0 )
    raveRobot_->SetDOFValues(joints, orT, true);
}

void ViewerRAVEBase::setTransparencyRobot(double percent)
{
  // go through each geometry and set it to transparent
  std::vector<OpenRAVE::KinBody::LinkPtr> links = raveRobot_->GetLinks();
  for (unsigned int i = 0; i < links.size(); i++) {
    std::vector<OpenRAVE::KinBody::Link::GeometryPtr> geometries = links[i]->GetGeometries();
    for (unsigned int j = 0; j < links.size(); j++) {
      geometries[j]->SetTransparency(percent/100);
    }
  }
}

void ViewerRAVEBase::addPrimitives(const std::vector<flp::Primitive>& primitives)
{
  // spheres
  std::vector<OpenRAVE::Vector> spheres;
  for (unsigned int i = 0 ; i < primitives.size(); i++) {
    if (primitives[i].type == flp::PrimitiveTypeSphere) {
      const Eigen::Vector3d& c = primitives[i].sphere.center;
      spheres.push_back( OpenRAVE::Vector(c(0), c(1), c(2), primitives[i].sphere.radius) );
    }
  }
  if (spheres.size() > 0) {
    // name
    char name[100];
    sprintf(name, "spheres%d", (int)primitives_.size());
    // add body
    OpenRAVE::KinBodyPtr s = RaveCreateKinBody(raveEnv_);
    s->InitFromSpheres(spheres,false);
    s->SetName(name);
    raveEnv_->Add(s);
    primitives_.push_back(s);
  }
  // cylinders
  std::list<OpenRAVE::KinBody::GeometryInfo> geometries;
  for (unsigned int i = 0 ; i < primitives.size(); i++) {
    if (primitives[i].type == flp::PrimitiveTypeCylinder) {
      const Eigen::Vector3d& c = primitives[i].cylinder.center;
      OpenRAVE::KinBody::GeometryInfo geom;
      geom._type = OpenRAVE::GeometryType::GT_Cylinder;
      geom._vGeomData.x = primitives[i].cylinder.radius;
      geom._vGeomData.y = primitives[i].cylinder.height;
      geom._t.trans = OpenRAVE::Vector(c(0), c(1), c(2));
      geom._t.rot = OpenRAVE::Vector(0,0,0,1);
      geometries.push_back(geom);
    }
  }
  if (geometries.size() > 0) {
    // name
    char name[100];
    sprintf(name, "geom%d", (int)primitives_.size());
    // add body
    OpenRAVE::KinBodyPtr s = RaveCreateKinBody(raveEnv_);
    s->InitFromGeometries(geometries);
    s->SetName(name);
    raveEnv_->Add(s);
    primitives_.push_back(s);
  }
}

void ViewerRAVEBase::clearPrimitives()
{
  for (unsigned int i = 0 ; i < primitives_.size(); i++) {
    raveEnv_->Remove(primitives_[i]);
    //viewer_->RemoveKinBody(primitives_[i]);
  }
  primitives_.clear();
}

void ViewerRAVEBase::addLineList(const std::vector<Eigen::Vector3d>& lineList, const Eigen::Vector4d& color)
{
  if (lineList.size() < 2) {
    printf("line list is empty.\n");
    return;
  }
  unsigned int size = lineList.size();
  float *pts = new float[3*size];
  for (unsigned int i = 0; i < size; i++) {
    pts[i*3+0] = lineList[i](0);
    pts[i*3+1] = lineList[i](1);
    pts[i*3+2] = lineList[i](2);
  }
  viewer_handle_lines_.push_back(raveEnv_->drawlinelist(pts, size, 3*sizeof(float), 10, OpenRAVE::Vector(color(0),color(1),color(2),color(3))));
  delete[] pts;
}

void ViewerRAVEBase::addLineList(const std::vector<Eigen::Vector3d>& lineList, const std::vector<Eigen::Vector4d>& lineColors)
{
  if (lineList.size() < 2) {
    printf("line list is empty.\n");
    return;
  }
  assert(lineList.size() == lineColors.size()*2);
  std::vector<Eigen::Vector3d> lines;
  lines.reserve(lineList.size());
  for (unsigned int i = 0; i < lineColors.size(); i++) {
    lines.push_back(lineList[i*2]);
    lines.push_back(lineList[i*2+1]);
    if (i == lineColors.size()-1 || lineColors[i] != lineColors[i+1]) {
      addLineList(lines, lineColors[i]);
      lines.clear();
    }
  }
}

void ViewerRAVEBase::clearLineList()
{
  viewer_handle_lines_.clear();
}

void ViewerRAVEBase::draw()
{

}

void ViewerRAVEBase::drawAndIdle()
{
  do {
   std::cout << '\n' << "Press ENTER to continue...";
  } while (std::cin.get() != '\n');
}


//----------------------------------------------------------------------------

ViewerRAVE::ViewerRAVE() : ViewerRAVEBase()
{
  viewerThread_ = boost::thread(&ViewerRAVE::runViewer, this);
}

ViewerRAVE::ViewerRAVE(const std::string& robotModelPath) : ViewerRAVEBase(robotModelPath)
{
  viewerThread_ = boost::thread(&ViewerRAVE::runViewer, this);
}

ViewerRAVE::~ViewerRAVE()
{
  raveEnv_->Destroy();
}

void ViewerRAVE::runViewer()
{
  // print available viewers
  if (true) {
    std::map<OpenRAVE::InterfaceType, std::vector<std::string> > interfacenames;
    OpenRAVE::RaveGetLoadedInterfaces(interfacenames);
    std::vector<std::string>::const_iterator it;
    std::cout << "Available viewers:\n";
    for (it = interfacenames[OpenRAVE::PT_Viewer].begin(); it != interfacenames[OpenRAVE::PT_Viewer].end(); it++)
      std::cout << *it << "\n";
  }
  // default viewer
  if(!viewer_) {
    std::string viewername = ""; //OpenRAVE::RaveGetDefaultViewerType();
    if(viewername.size() > 0) {
      viewer_ = OpenRAVE::RaveCreateViewer(raveEnv_, viewername);
    }
  }
  // any viewer
  if(!viewer_) {
    std::map<OpenRAVE::InterfaceType, std::vector<std::string> > interfacenames;
    OpenRAVE::RaveGetLoadedInterfaces(interfacenames);
    std::vector<std::string>::const_iterator it;
    for (it = interfacenames[OpenRAVE::PT_Viewer].begin(); it != interfacenames[OpenRAVE::PT_Viewer].end(); it++) {
      viewer_ = OpenRAVE::RaveCreateViewer(raveEnv_, *it);
      if(!!viewer_)
        break;
    }
  }
  if (!viewer_) {
    printf("Could not find any valid viewer...\n");
    return;
  }
  // start
  raveEnv_->Add(viewer_);
  viewer_->main(true); // infinite loop
}

