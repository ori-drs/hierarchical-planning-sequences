#include "EnvironmentConversions.h"
#include <pcl/common/common.h>

bool convert(const grid_map::GridMap& gridmap, pcl::PolygonMesh& mesh)
{
  const std::string layerZ = "elevation";
  const float resolution = gridmap.getResolution();

  // aux
  grid_map::Position position;
  grid_map::Index i1,i2,i3;
  bool ok1,ok2,ok3;
  float z1,z2,z3;
  std::vector<float> pts;

  // add cells
  for (grid_map::GridMapIterator iterator(gridmap); !iterator.isPastEnd(); ++iterator) {
    gridmap.getPosition(*iterator, position);
    float x = position.x();
    float y = position.y();
    float z = gridmap.at(layerZ, *iterator);
    if (std::isnan(z)) continue;
    // neighbor points
    ok1 = gridmap.getIndex(grid_map::Position(x+resolution, y           ), i1);
    ok2 = gridmap.getIndex(grid_map::Position(x           , y+resolution), i2);
    ok3 = gridmap.getIndex(grid_map::Position(x+resolution, y-resolution), i3);
    if (ok1) z1 = gridmap.at(layerZ, i1);
    if (ok2) z2 = gridmap.at(layerZ, i2);
    if (ok3) z3 = gridmap.at(layerZ, i3);
    // triangle up
    if (ok1 && ok2 && !std::isnan(z1) && !std::isnan(z2)) {
      pts.push_back(x);            pts.push_back(y);            pts.push_back(z);
      pts.push_back(x+resolution); pts.push_back(y);            pts.push_back(z1);
      pts.push_back(x);            pts.push_back(y+resolution); pts.push_back(z2);
    }
    // triangle down
    if (ok1 && ok3 && !std::isnan(z1) && !std::isnan(z3)) {
      pts.push_back(x);            pts.push_back(y);            pts.push_back(z);
      pts.push_back(x+resolution); pts.push_back(y);            pts.push_back(z1);
      pts.push_back(x+resolution); pts.push_back(y-resolution); pts.push_back(z3);
    }
  }

  // convert to pcl mesh
  int npts = pts.size()/3;
  int ntri = npts/3;
  mesh.polygons.resize(ntri);
  for (int i = 0; i < ntri; i++) {
    mesh.polygons[i].vertices.resize(3);
    mesh.polygons[i].vertices[0] = i*3+0;
    mesh.polygons[i].vertices[1] = i*3+1;
    mesh.polygons[i].vertices[2] = i*3+2;
  }
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud( new pcl::PointCloud<pcl::PointXYZ>() );
  cloud->points.resize(pts.size());
  for (int i = 0; i < npts; i++) {
    cloud->points[i].x = pts[i*3+0];
    cloud->points[i].y = pts[i*3+1];
    cloud->points[i].z = pts[i*3+2];
  }
  pcl::toPCLPointCloud2 (*cloud, mesh.cloud);

  return true;
}

bool convert(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, grid_map::GridMap& gridmap, double resolution)
{
  const std::string layerZ = "elevation";

  // borders
  pcl::PointXYZRGB ptmin, ptmax;
  pcl::getMinMax3D(*cloud, ptmin, ptmax);

  // setup gridmap
  gridmap.setFrameId("odom");
  gridmap.setGeometry(grid_map::Length(ptmax.x - ptmin.x, ptmax.y - ptmin.y), resolution);

  // insert cloud
  grid_map::Matrix& data = gridmap[layerZ];
  unsigned int npts = cloud->points.size();
  for (unsigned int i = 0; i < npts; i++) {
    const pcl::PointXYZRGB& pt = cloud->points[i];
    grid_map::Position position(pt.x, pt.y);
    grid_map::Index index;
    if (gridmap.getIndex(position, index)) {
      double z = data(index(0), index(1));
      if (std::isnan(z) || z < pt.z) {
        data(index(0), index(1)) = pt.z;
      }
    }
  }
  return true;
}

