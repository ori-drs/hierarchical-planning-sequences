#pragma once
#include "flp.h"
#include <Eigen/Geometry>
#include <boost/shared_ptr.hpp>
#include <random>

// load stl (file)
// get mesh
// get point cloud
// get heightmap
// get 3d grid
// get graph of clouds
// get graph of heightmaps


class Environment
{
public:
  typedef boost::shared_ptr<Environment> Ptr;

  enum CollisionType {
    CollisionTypeInside,
    CollisionTypeAbove,
    CollisionTypeAboveMaxHeight
  };

  Environment() {}
  bool getCollision(const std::vector<flp::Primitive>& primitives) {
    for (unsigned int i = 0; i < primitives.size(); i++)
      if (getCollision(primitives[i]))
        return true;
    return false;
  }
  std::vector<flp::Point> getPointsInside(const std::vector<flp::Primitive>& primitives) {
    std::vector<flp::Point> points;
    for (unsigned int i = 0; i < primitives.size(); i++) {
      std::vector<flp::Point> p = getPointsInside(primitives[i]);
      points.insert(points.end(), p.begin(), p.end());
    }
    return points;
  }
  virtual bool getCollision(const flp::Primitive& primitive, const CollisionType& type = CollisionTypeInside) = 0;
  virtual bool getCollisionSweptVolume(const flp::Primitive& primitive1, const flp::Primitive& primitive2, double& freeFraction, double& collidingFraction) = 0;
  virtual std::vector<flp::Point> getPointsInside(const flp::Primitive& primitive) = 0;
  virtual flp::Point getNearestNeighbor(const flp::Point& point) = 0;
  virtual int getNumIntersectedPoints(const Eigen::Vector3d start, const Eigen::Vector3d target) = 0;
  virtual flp::Point sampleUniform(std::mt19937& generator) = 0;
  virtual double getSpatialResolution() = 0;
  virtual void getSpatialBounds(std::vector<double>& lb, std::vector<double>& ub) = 0;
};

