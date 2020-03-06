#include "EnvironmentRecast.h"

EnvironmentRecast::EnvironmentRecast()
{

}

EnvironmentRecast::EnvironmentRecast(const pcl::PolygonMesh& mesh)
{
  Params params;
  build(mesh, params);
}

void EnvironmentRecast::build(const pcl::PolygonMesh& mesh, const Params& params)
{
  recast_.stg.cellSize = params.cellSize;
  recast_.stg.cellHeight = params.cellHeight;
  recast_.stg.agentHeight = params.agentHeight;
  recast_.stg.agentRadius = params.agentRadius;
  recast_.stg.agentMaxClimb = params.agentMaxClimb;
  recast_.stg.agentMaxSlope = params.agentMaxSlopeDegrees;
  recast_.build(mesh);
}

bool EnvironmentRecast::getCollision(const flp::Primitive& primitive, const CollisionType& type)
{
  // TODO
  return false;
}

bool EnvironmentRecast::getCollisionSweptVolume(const flp::Primitive& primitive1, const flp::Primitive& primitive2, double& freeFraction, double& collidingFraction)
{
  // TODO
  return false;
}

void EnvironmentRecast::getSweptVolumeStats(const std::vector<flp::Primitive>& primitive1, const std::vector<flp::Primitive>& primitive2, int minPrimitivesColliding, double& firstCollidingFraction, std::vector<flp::Point>& min)
{
  // TODO
}

std::vector<flp::Point> EnvironmentRecast::getPointsInside(const flp::Primitive& primitive)
{
  // TODO
  return std::vector<flp::Point>();
}

flp::Point EnvironmentRecast::getNearestNeighbor(const flp::Point& point)
{
  // TODO
  return point;
}

int EnvironmentRecast::getNumIntersectedPoints(const Eigen::Vector3d start, const Eigen::Vector3d target)
{
  // TODO
  return 0;
}

flp::Point EnvironmentRecast::sampleUniform(std::mt19937& generator)
{
  // TODO
  return flp::Point(0,0,0);
}

double EnvironmentRecast::getSpatialResolution()
{
  // TODO
  return 0;
}

void EnvironmentRecast::getSpatialBounds(std::vector<double>& lb, std::vector<double>& ub)
{
  // TODO
}

