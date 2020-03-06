#include "EnvironmentPCL.h"
#include <pcl/conversions.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/filters/crop_hull.h>
#include <pcl/common/common.h>

EnvironmentPCL::EnvironmentPCL()
  : mesh_(new pcl::PolygonMesh())
  , mesh_cloud_(new pcl::PointCloud<pcl::PointXYZRGB>())
  , cloud_(new pcl::PointCloud<pcl::PointXYZRGB>())
  , tree_(new pcl::KdTreeFLANN<pcl::PointXYZRGB>())
  , cloud_resolution_(0.05)
{

}

bool EnvironmentPCL::getCollision(const flp::Primitive& primitive, const CollisionType& type)
{
  if (primitive.type == flp::PrimitiveTypeSphere) {
    // range search
    pcl::PointXYZRGB center;
    center.x = primitive.sphere.center(0);
    center.y = primitive.sphere.center(1);
    center.z = primitive.sphere.center(2);
    std::vector<int> idx;
    std::vector<float> sqdist;
    tree_->radiusSearch(center, primitive.sphere.radius, idx, sqdist);
    if (type == CollisionTypeInside) {
      // collision with the inside of the sphere
      return idx.size() > 0;
    } else if (type == CollisionTypeAbove) {
      std::cout << "TODO: PCL CollisionTypeAbove\n";
    } else if (type == CollisionTypeAboveMaxHeight) {
      std::cout << "TODO: PCL CollisionTypeAboveMaxHeight\n";
    }
    //for (unsigned int i = 0;  i < idx.size(); i++) {
    //  if (cloud_->points[idx[i]].z > primitive.sphere.center(2) + 0.01)
    //    return true;
    //}
  } else if (primitive.type == flp::PrimitiveTypeCylinder) {
    // range search
    pcl::PointXYZRGB center;
    center.x = primitive.cylinder.center(0);
    center.y = primitive.cylinder.center(1);
    center.z = primitive.cylinder.center(2);
    std::vector<int> idx;
    std::vector<float> sqdist;
    tree_->radiusSearch(center, primitive.cylinder.radius, idx, sqdist);
    for (unsigned int i = 0;  i < idx.size(); i++) {
      float z = cloud_->points[idx[i]].z;
      if (z > primitive.cylinder.center(2) - primitive.cylinder.height && z < primitive.cylinder.center(2) + primitive.cylinder.height)
        return true;
    }
  } else if (primitive.type == flp::PrimitiveTypeBox) {
    // TODO
  } else if (primitive.type == flp::PrimitiveTypeSolid) {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pricloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr hull(new pcl::PointCloud<pcl::PointXYZRGB>());
    std::vector<pcl::Vertices> polygons;
    std::vector<int> idx;
    // cloud of primitive vertices
    for (unsigned int i = 0; i < primitive.solid.polygons.size(); i++) {
      for (unsigned int j = 0; j < primitive.solid.polygons[i].vertices.size(); j++) {
        const Eigen::Vector3d& pt = primitive.solid.polygons[i].vertices[j];
        pricloud->push_back(pcl::PointXYZRGB(pt(0), pt(1), pt(2)));
      }
    }
    // convex hull
    pcl::ConvexHull<pcl::PointXYZRGB> fhull;
    fhull.setInputCloud (pricloud);
    fhull.reconstruct (*hull, polygons);
    // crop hull OR check dot product of normals
    pcl::CropHull<pcl::PointXYZRGB> fcrop;
    fcrop.setInputCloud (cloud_);
    fcrop.setHullCloud (hull);
    fcrop.setHullIndices (polygons);
    fcrop.setDim (fhull.getDimension());
    fcrop.filter (idx);
    if (type == CollisionTypeInside) {
      // collision with the inside of the solid
      return idx.size() > 0;
    } else if (type == CollisionTypeAbove) {
      std::cout << "TODO: PCL CollisionTypeAbove\n";
    } else if (type == CollisionTypeAboveMaxHeight) {
      std::cout << "TODO: PCL CollisionTypeAboveMaxHeight\n";
    }
  }
  return false;
}

bool EnvironmentPCL::getCollisionSweptVolume(const flp::Primitive& primitive1, const flp::Primitive& primitive2, double& freeFraction, double& collidingFraction)
{
  // TODO
  std::cout << "TODO: getCollisionSweptVolume\n";
  return true;
}

std::vector<flp::Point> EnvironmentPCL::getPointsInside(const flp::Primitive& primitive)
{
  std::vector<flp::Point> points;
  if (primitive.type == flp::PrimitiveTypeSphere) {
    // range search
    pcl::PointXYZRGB center;
    center.x = primitive.sphere.center(0);
    center.y = primitive.sphere.center(1);
    center.z = primitive.sphere.center(2);
    std::vector<int> idx;
    std::vector<float> sqdist;
    tree_->radiusSearch(center, primitive.sphere.radius, idx, sqdist);
    // add
    for (unsigned int i = 0;  i < idx.size(); i++) {
      const pcl::PointXYZRGB& pt = cloud_->points[idx[i]];
      flp::Point p(pt.x, pt.y, pt.z);
      points.push_back(p);
    }
  } else if (primitive.type == flp::PrimitiveTypeCylinder) {
    // range search
    pcl::PointXYZRGB center;
    center.x = primitive.cylinder.center(0);
    center.y = primitive.cylinder.center(1);
    center.z = primitive.cylinder.center(2);
    std::vector<int> idx;
    std::vector<float> sqdist;
    tree_->radiusSearch(center, primitive.cylinder.radius, idx, sqdist);
    // add
    for (unsigned int i = 0;  i < idx.size(); i++) {
      const pcl::PointXYZRGB& pt = cloud_->points[idx[i]];
      flp::Point p(pt.x, pt.y, pt.z);
      if (pt.z > primitive.cylinder.center(2) - primitive.cylinder.height && pt.z < primitive.cylinder.center(2) + primitive.cylinder.height)
        points.push_back(p);
    }
  } else if (primitive.type == flp::PrimitiveTypeBox) {
    // TODO
  } else if (primitive.type == flp::PrimitiveTypeSolid) {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pricloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr hull(new pcl::PointCloud<pcl::PointXYZRGB>());
    std::vector<pcl::Vertices> polygons;
    std::vector<int> idx;
    // cloud of primitive vertices
    for (unsigned int i = 0; i < primitive.solid.polygons.size(); i++) {
      for (unsigned int j = 0; j < primitive.solid.polygons[i].vertices.size(); j++) {
        const Eigen::Vector3d& pt = primitive.solid.polygons[i].vertices[j];
        pricloud->push_back(pcl::PointXYZRGB(pt(0), pt(1), pt(2)));
      }
    }
    // convex hull
    pcl::ConvexHull<pcl::PointXYZRGB> fhull;
    fhull.setInputCloud (pricloud);
    fhull.reconstruct (*hull, polygons);
    // crop hull OR check dot product of normals
    pcl::CropHull<pcl::PointXYZRGB> fcrop;
    fcrop.setInputCloud (cloud_);
    fcrop.setHullCloud (hull);
    fcrop.setHullIndices (polygons);
    fcrop.setDim (fhull.getDimension());
    fcrop.filter (idx);
    // add
    for (unsigned int i = 0;  i < idx.size(); i++) {
      const pcl::PointXYZRGB& pt = cloud_->points[idx[i]];
      flp::Point p(pt.x, pt.y, pt.z);
      points.push_back(p);
    }
  }
  return points;
}

flp::Point EnvironmentPCL::getNearestNeighbor(const flp::Point& point)
{
  pcl::PointXYZRGB center(point.p(0), point.p(1), point.p(2));
  std::vector<int> idx;
  std::vector<float> sqdist;
  tree_->nearestKSearch(center, 1, idx, sqdist);
  if (idx.size() == 0)
    return point;
  const pcl::PointXYZRGB& pt = cloud_->points[idx[0]];
  return flp::Point(pt.x, pt.y, pt.z);
}

int EnvironmentPCL::getNumIntersectedPoints(const Eigen::Vector3d start, const Eigen::Vector3d target)
{
  std::cout << "TODO: getNumIntersectedPoints\n";
  return 0;
}

flp::Point EnvironmentPCL::sampleUniform(std::mt19937& generator)
{
  std::cout << "TODO: sampleUniform\n";
  return flp::Point(0,0,0);
}

double EnvironmentPCL::getSpatialResolution()
{
  return cloud_resolution_;
}

void EnvironmentPCL::getSpatialBounds(std::vector<double>& lb, std::vector<double>& ub)
{
  pcl::PointXYZRGB min_pt, max_pt;
  pcl::getMinMax3D(*cloud_, min_pt, max_pt);
  lb.resize(3);
  ub.resize(3);
  lb[0] = min_pt.x;
  lb[1] = min_pt.y;
  lb[2] = min_pt.z;
  ub[0] = max_pt.x;
  ub[1] = max_pt.y;
  ub[2] = max_pt.z;
}

void EnvironmentPCL::addBox(double cx, double cy, double cz, double sx, double sy, double sz)
{
  addPlaneX(cx+sx/2.0, cy, cz, sy, sz);
  addPlaneX(cx-sx/2.0, cy, cz, sy, sz);
  addPlaneY(cx, cy+sy/2.0, cz, sx, sz);
  addPlaneY(cx, cy-sy/2.0, cz, sx, sz);
  addPlaneZ(cx, cy, cz+sz/2.0, sx, sy);
  addPlaneZ(cx, cy, cz-sz/2.0, sx, sy);
}

void EnvironmentPCL::addPlaneX(double cx, double cy, double cz, double sy, double sz)
{
  // mesh of two triangles
  pcl::PointXYZRGB pt1(cx,cy-sy/2.0,cz-sz/2.0);
  pcl::PointXYZRGB pt2(cx,cy+sy/2.0,cz-sz/2.0);
  pcl::PointXYZRGB pt3(cx,cy+sy/2.0,cz+sz/2.0);
  pcl::PointXYZRGB pt4(cx,cy-sy/2.0,cz+sz/2.0);
  mesh_cloud_->points.push_back(pt1);
  mesh_cloud_->points.push_back(pt2);
  mesh_cloud_->points.push_back(pt3);
  mesh_cloud_->points.push_back(pt4);
  mesh_cloud_->width = mesh_cloud_->points.size();
  mesh_cloud_->height = 1;
  pcl::toPCLPointCloud2 (*mesh_cloud_, mesh_->cloud);
  pcl::Vertices poly;
  poly.vertices.resize(3);
  poly.vertices[0] = mesh_cloud_->points.size()-4;
  poly.vertices[1] = mesh_cloud_->points.size()-3;
  poly.vertices[2] = mesh_cloud_->points.size()-2;
  mesh_->polygons.push_back(poly);
  poly.vertices[0] = mesh_cloud_->points.size()-4;
  poly.vertices[1] = mesh_cloud_->points.size()-2;
  poly.vertices[2] = mesh_cloud_->points.size()-1;
  mesh_->polygons.push_back(poly);
  // dense point cloud
  for (double y = cy-sy/2.0; y < cy+sy/2.0+cloud_resolution_/2.0; y += cloud_resolution_)
    for (double z = cz-sz/2.0; z < cz+sz/2.0+cloud_resolution_/2.0; z += cloud_resolution_)
      cloud_->points.push_back(pcl::PointXYZRGB(cx,y,z));
  tree_->setInputCloud(cloud_);
}

void EnvironmentPCL::addPlaneY(double cx, double cy, double cz, double sx, double sz)
{
  // mesh of two triangles
  pcl::PointXYZRGB pt1(cx-sx/2.0,cy,cz-sz/2.0);
  pcl::PointXYZRGB pt2(cx+sx/2.0,cy,cz-sz/2.0);
  pcl::PointXYZRGB pt3(cx+sx/2.0,cy,cz+sz/2.0);
  pcl::PointXYZRGB pt4(cx-sx/2.0,cy,cz+sz/2.0);
  mesh_cloud_->points.push_back(pt1);
  mesh_cloud_->points.push_back(pt2);
  mesh_cloud_->points.push_back(pt3);
  mesh_cloud_->points.push_back(pt4);
  mesh_cloud_->width = mesh_cloud_->points.size();
  mesh_cloud_->height = 1;
  pcl::toPCLPointCloud2 (*mesh_cloud_, mesh_->cloud);
  pcl::Vertices poly;
  poly.vertices.resize(3);
  poly.vertices[0] = mesh_cloud_->points.size()-4;
  poly.vertices[1] = mesh_cloud_->points.size()-3;
  poly.vertices[2] = mesh_cloud_->points.size()-2;
  mesh_->polygons.push_back(poly);
  poly.vertices[0] = mesh_cloud_->points.size()-4;
  poly.vertices[1] = mesh_cloud_->points.size()-2;
  poly.vertices[2] = mesh_cloud_->points.size()-1;
  mesh_->polygons.push_back(poly);
  // dense point cloud
  for (double x = cx-sx/2.0; x < cx+sx/2.0+cloud_resolution_/2.0; x += cloud_resolution_)
    for (double z = cz-sz/2.0; z < cz+sz/2.0+cloud_resolution_/2.0; z += cloud_resolution_)
      cloud_->points.push_back(pcl::PointXYZRGB(x,cy,z));
  tree_->setInputCloud(cloud_);
}

void EnvironmentPCL::addPlaneZ(double cx, double cy, double cz, double sx, double sy)
{
  // mesh of two triangles
  pcl::PointXYZRGB pt1(cx-sx/2.0,cy-sy/2.0,cz);
  pcl::PointXYZRGB pt2(cx+sx/2.0,cy-sy/2.0,cz);
  pcl::PointXYZRGB pt3(cx+sx/2.0,cy+sy/2.0,cz);
  pcl::PointXYZRGB pt4(cx-sx/2.0,cy+sy/2.0,cz);
  mesh_cloud_->points.push_back(pt1);
  mesh_cloud_->points.push_back(pt2);
  mesh_cloud_->points.push_back(pt3);
  mesh_cloud_->points.push_back(pt4);
  mesh_cloud_->width = mesh_cloud_->points.size();
  mesh_cloud_->height = 1;
  pcl::toPCLPointCloud2 (*mesh_cloud_, mesh_->cloud);
  pcl::Vertices poly;
  poly.vertices.resize(3);
  poly.vertices[0] = mesh_cloud_->points.size()-4;
  poly.vertices[1] = mesh_cloud_->points.size()-3;
  poly.vertices[2] = mesh_cloud_->points.size()-2;
  mesh_->polygons.push_back(poly);
  poly.vertices[0] = mesh_cloud_->points.size()-4;
  poly.vertices[1] = mesh_cloud_->points.size()-2;
  poly.vertices[2] = mesh_cloud_->points.size()-1;
  mesh_->polygons.push_back(poly);
  // dense point cloud
  for (double x = cx-sx/2.0; x < cx+sx/2.0+cloud_resolution_/2.0; x += cloud_resolution_)
    for (double y = cy-sy/2.0; y < cy+sy/2.0+cloud_resolution_/2.0; y += cloud_resolution_)
      cloud_->points.push_back(pcl::PointXYZRGB(x,y,cz));
  tree_->setInputCloud(cloud_);
}

