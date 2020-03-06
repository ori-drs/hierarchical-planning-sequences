#pragma once
#include "Environment.h"
#include "RobotModelFixedPrimitives.h"
#include "recastapp/RecastPlanner.h"
#include <pcl/PolygonMesh.h>

class EnvironmentRecast : public Environment
{
public:
  typedef boost::shared_ptr<EnvironmentRecast> Ptr;
  EnvironmentRecast();
  EnvironmentRecast(const pcl::PolygonMesh& mesh);

  struct Params {
    Params() : cellSize(0.2f), cellHeight(0.2f), agentHeight(1.0f), agentRadius(0.2f), agentMaxClimb(0.2f), agentMaxSlopeDegrees(45.0f) {}
    float cellSize;
    float cellHeight;
    float agentHeight;
    float agentRadius;
    float agentMaxClimb;
    float agentMaxSlopeDegrees;
  };
  void build(const pcl::PolygonMesh& mesh, const Params& params);

  virtual bool getCollision(const flp::Primitive& primitive, const CollisionType& type = CollisionTypeInside);
  virtual bool getCollisionSweptVolume(const flp::Primitive& primitive1, const flp::Primitive& primitive2, double& freeFraction, double& collidingFraction);
  virtual void getSweptVolumeStats(const std::vector<flp::Primitive>& primitive1, const std::vector<flp::Primitive>& primitive2, int minPrimitivesColliding, double& firstCollidingFraction, std::vector<flp::Point>& min);
  virtual std::vector<flp::Point> getPointsInside(const flp::Primitive& primitive);
  virtual flp::Point getNearestNeighbor(const flp::Point& point);
  virtual int getNumIntersectedPoints(const Eigen::Vector3d start, const Eigen::Vector3d target);
  virtual flp::Point sampleUniform(std::mt19937& generator);
  virtual double getSpatialResolution();
  virtual void getSpatialBounds(std::vector<double>& lb, std::vector<double>& ub);

  recastapp::RecastPlanner recast_;
};

