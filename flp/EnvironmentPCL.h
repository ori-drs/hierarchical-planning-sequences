#pragma once
#include "Environment.h"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PolygonMesh.h>
#include <pcl/kdtree/kdtree_flann.h>

class EnvironmentPCL : public Environment
{
public:
  typedef boost::shared_ptr<EnvironmentPCL> Ptr;

  EnvironmentPCL();
  virtual bool getCollision(const flp::Primitive& primitive, const CollisionType& type = CollisionTypeInside);
  virtual bool getCollisionSweptVolume(const flp::Primitive& primitive1, const flp::Primitive& primitive2, double& freeFraction, double& collidingFraction);
  virtual std::vector<flp::Point> getPointsInside(const flp::Primitive& primitive);
  virtual flp::Point getNearestNeighbor(const flp::Point& point);
  virtual int getNumIntersectedPoints(const Eigen::Vector3d start, const Eigen::Vector3d target);
  virtual flp::Point sampleUniform(std::mt19937& generator);
  virtual double getSpatialResolution();
  virtual void getSpatialBounds(std::vector<double>& lb, std::vector<double>& ub);

  void addBox(double cx, double cy, double cz, double sx, double sy, double sz);
  void addPlaneX(double cx, double cy, double cz, double sy, double sz);
  void addPlaneY(double cx, double cy, double cz, double sx, double sz);
  void addPlaneZ(double cx, double cy, double cz, double sx, double sy);

  pcl::PolygonMesh::Ptr mesh_;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr mesh_cloud_;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_;
  pcl::KdTreeFLANN<pcl::PointXYZRGB>::Ptr tree_;
  double cloud_resolution_;
};

