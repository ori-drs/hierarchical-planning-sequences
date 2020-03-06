#pragma once
#include "Environment.h"
#include <grid_map_core/grid_map_core.hpp>

class EnvironmentGridMap : public Environment
{
public:
  typedef boost::shared_ptr<EnvironmentGridMap> Ptr;

  EnvironmentGridMap();
  virtual bool getCollision(const flp::Primitive& primitive, const CollisionType& type = CollisionTypeInside);
  virtual bool getCollisionSweptVolume(const flp::Primitive& primitive1, const flp::Primitive& primitive2, double& freeFraction, double& collidingFraction);
  virtual void getSweptVolumeStats(const std::vector<flp::Primitive>& primitive1, const std::vector<flp::Primitive>& primitive2, int minPrimitivesColliding, double& firstCollidingFraction, std::vector<flp::Point>& min);
  virtual std::vector<flp::Point> getPointsInside(const flp::Primitive& primitive);
  virtual flp::Point getNearestNeighbor(const flp::Point& point);
  virtual int getNumIntersectedPoints(const Eigen::Vector3d start, const Eigen::Vector3d target);
  virtual flp::Point sampleUniform(std::mt19937& generator);
  virtual double getSpatialResolution();
  virtual void getSpatialBounds(std::vector<double>& lb, std::vector<double>& ub);

  grid_map::GridMap map_;
  const std::string layerElevation_;
  const std::string layerFriction_;
  const std::string layerRoughness_;
  const std::string layerTraversability_;
};

