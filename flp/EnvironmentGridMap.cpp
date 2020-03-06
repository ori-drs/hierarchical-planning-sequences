#include "EnvironmentGridMap.h"
#include <iostream>

EnvironmentGridMap::EnvironmentGridMap()
  : layerElevation_("elevation")
  , layerFriction_("friction")
  , layerRoughness_("roughness")
  , layerTraversability_("traversability")
{

}

bool EnvironmentGridMap::getCollision(const flp::Primitive& primitive, const CollisionType& type)
{
  grid_map::Matrix& data = map_[layerElevation_];
  // search point
  if (primitive.type == flp::PrimitiveTypeSphere) { // sphere
    // range search assuming sphere ~ circle
    const double radius = primitive.sphere.radius + getSpatialResolution(); // radius + safety margin (grid resolution)
    grid_map::Position center(primitive.sphere.center(0), primitive.sphere.center(1));
    grid_map::Position pt;
    int totalPts = 0;
    int invalidPts = 0;
    for (grid_map::CircleIterator it(map_, center, radius); !it.isPastEnd(); ++it) {
      const grid_map::Index index(*it);
      double z = data(index(0), index(1));
      totalPts++;
      if (std::isnan(z)) {
        invalidPts++;
        continue;
      }
      map_.getPosition(index, pt);
      Eigen::Vector3d eiPt(pt.x(),pt.y(),z);
      Eigen::Vector3d eiVec = eiPt - primitive.sphere.center;
      double zsphere = primitive.sphere.center(2) + sqrt( pow(radius,2.0) - pow(eiVec(0),2.0) - pow(eiVec(1),2.0) );
      if (type == CollisionTypeInside) {
        // collision with the inside of the sphere
        if (eiVec.norm() <= radius)
          return true;
        // if grid_map height is above the sphere then it is colliding too
        if (z > zsphere)
          return true;
      } else if (type == CollisionTypeAbove) {
        // point is above the sphere
        if (z > zsphere + 0.01)
          return true;
      } else if (type == CollisionTypeAboveMaxHeight) {
        // point is above the maximum height of the sphere
        if (z > primitive.sphere.center(2) + primitive.sphere.radius)
          return true;
      }
    }
  } else if (primitive.type == flp::PrimitiveTypeCylinder) { // cylinder
    // range search assuming cylinder ~ circle
    const double radius = primitive.cylinder.radius + getSpatialResolution(); // radius + safety margin (grid resolution)
    grid_map::Position center(primitive.cylinder.center(0), primitive.cylinder.center(1));
    grid_map::Position pt;
    int totalPts = 0;
    int invalidPts = 0;
    for (grid_map::CircleIterator it(map_, center, radius); !it.isPastEnd(); ++it) {
      const grid_map::Index index(*it);
      double z = data(index(0), index(1));
      totalPts++;
      if (std::isnan(z)) {
        invalidPts++;
        continue;
      }
      map_.getPosition(index, pt);
      // check inside radius
      double distXY = sqrt( pow(pt.x()-primitive.cylinder.center(0), 2.0) + pow(pt.y()-primitive.cylinder.center(1), 2.0) );
      if (distXY > primitive.cylinder.radius)
        continue;
      double zbottom = primitive.cylinder.center(2) - primitive.cylinder.height/2;
      double ztop = primitive.cylinder.center(2) + primitive.cylinder.height/2;
      if (type == CollisionTypeInside) {
        // collision with the inside of the cylinder
        if (z > zbottom)
          return true;
      } else if (type == CollisionTypeAbove) {
        // point is above the cylinder
        if (z > ztop + 0.01)
          return true;
      } else if (type == CollisionTypeAboveMaxHeight) {
        // point is above the maximum height of the cylinder
        if (z > ztop)
          return true;
      }
    }
  } else if (primitive.type == flp::PrimitiveTypeBox) { // box
    // TODO
    std::cout << "TODO\n";
  } else if (primitive.type == flp::PrimitiveTypeSolid) { // solid
    // TODO
    std::cout << "TODO\n";
  }
  return false;
}

bool EnvironmentGridMap::getCollisionSweptVolume(const flp::Primitive& primitive1, const flp::Primitive& primitive2, double& freeFraction, double& collidingFraction)
{
  bool collision = false;
  freeFraction = 1.0;
  collidingFraction = 1.0;
  if (primitive1.type != primitive2.type) {
    std::cout << "ERROR: swept volume between different primitives\n";
    return collision;
  }
  grid_map::Matrix& data = map_[layerElevation_];
  // search point
  if (primitive1.type == flp::PrimitiveTypeSphere) { // sphere
    // get radius
    double radius = primitive1.sphere.radius;
    if (primitive1.sphere.radius != primitive2.sphere.radius) {
      std::cout << "WARNING: spheres have different radii, will use conservative collision checking\n";
      radius = std::max(primitive1.sphere.radius, primitive2.sphere.radius);
    }
    // build top-view bounding box (approximate convex hull)
    Eigen::Vector3d a = primitive1.sphere.center;
    Eigen::Vector3d b = primitive2.sphere.center;
    Eigen::Vector3d x = (b-a).normalized();
    Eigen::Vector3d z (0,0,1);
    Eigen::Vector3d y = (z.cross(x)).normalized();
    x = y.cross(z);
    double dist2d = x.dot(b-a);
    Eigen::Vector3d p1 = a - x*radius - y*radius;
    Eigen::Vector3d p2 = b + x*radius - y*radius;
    Eigen::Vector3d p3 = b + x*radius + y*radius;
    Eigen::Vector3d p4 = a - x*radius + y*radius;
    grid_map::Polygon polygon;
    polygon.addVertex(grid_map::Position(p1(0),p1(1)));
    polygon.addVertex(grid_map::Position(p2(0),p2(1)));
    polygon.addVertex(grid_map::Position(p3(0),p3(1)));
    polygon.addVertex(grid_map::Position(p4(0),p4(1)));
    // iterate through points inside bounding box
    grid_map::Position pt;
    for (grid_map::PolygonIterator it(map_, polygon); !it.isPastEnd(); ++it) {
      const grid_map::Index index(*it);
      map_.getPosition(index, pt);
      // check point is inside the box or above
      Eigen::Vector3d p(pt.x(), pt.y(), data(index(0),index(1)));
      double fraction = x.dot(p-a) / dist2d;
      double zsafe = a(2) + fraction * (b(2)-a(2)) - radius;
      // TODO: what to do with NaNs... some maps have them all over the place even if not boundary
      if (p(2) >= zsafe) {
        collision = true;
        if (fraction < freeFraction)
          freeFraction = fraction;
      } else {
        if (fraction < collidingFraction)
          collidingFraction = fraction;
      }
    }
  } else if (primitive1.type == flp::PrimitiveTypeCylinder) { // cylinder ... approximate by sphere for now
    // get radius
    double radius = primitive1.cylinder.radius;
    if (primitive1.cylinder.radius != primitive2.cylinder.radius) {
      std::cout << "WARNING: cylinder have different radii, will use conservative collision checking\n";
      radius = std::max(primitive1.cylinder.radius, primitive2.cylinder.radius);
    }
    // build top-view bounding box (approximate convex hull)
    Eigen::Vector3d a = primitive1.cylinder.center;
    Eigen::Vector3d b = primitive2.cylinder.center;
    Eigen::Vector3d x = (b-a).normalized();
    Eigen::Vector3d z (0,0,1);
    Eigen::Vector3d y = (z.cross(x)).normalized();
    x = y.cross(z);
    double dist2d = x.dot(b-a);
    Eigen::Vector3d p1 = a - x*radius - y*radius;
    Eigen::Vector3d p2 = b + x*radius - y*radius;
    Eigen::Vector3d p3 = b + x*radius + y*radius;
    Eigen::Vector3d p4 = a - x*radius + y*radius;
    grid_map::Polygon polygon;
    polygon.addVertex(grid_map::Position(p1(0),p1(1)));
    polygon.addVertex(grid_map::Position(p2(0),p2(1)));
    polygon.addVertex(grid_map::Position(p3(0),p3(1)));
    polygon.addVertex(grid_map::Position(p4(0),p4(1)));
    // iterate through points inside bounding box
    grid_map::Position pt;
    for (grid_map::PolygonIterator it(map_, polygon); !it.isPastEnd(); ++it) {
      const grid_map::Index index(*it);
      map_.getPosition(index, pt);
      // check point is inside the box or above
      Eigen::Vector3d p(pt.x(), pt.y(), data(index(0),index(1)));
      double fraction = x.dot(p-a) / dist2d;
      double zsafe = a(2) + fraction * (b(2)-a(2)) - radius;
      // TODO: what to do with NaNs... some maps have them all over the place even if not boundary
      if (p(2) >= zsafe) {
        collision = true;
        if (fraction < freeFraction)
          freeFraction = fraction;
      } else {
        if (fraction < collidingFraction)
          collidingFraction = fraction;
      }
    }
  } else if (primitive1.type == flp::PrimitiveTypeBox) { // box
    // TODO
    std::cout << "TODO\n";
  } else if (primitive1.type == flp::PrimitiveTypeSolid) { // solid
    // TODO
    std::cout << "TODO\n";
  }
  return collision;
}

void EnvironmentGridMap::getSweptVolumeStats(const std::vector<flp::Primitive>& primitive1, const std::vector<flp::Primitive>& primitive2, int minPrimitivesColliding, double& firstCollidingFraction, std::vector<flp::Point>& min)
{
  assert(primitive1.size() == primitive2.size() && primitive1.size() > 0);
  assert(primitive1[0].type == flp::PrimitiveTypeSphere);
  bool existsFriction = map_.exists(layerFriction_);
  bool existsRoughness = map_.exists(layerRoughness_);
  bool existsTraversability = map_.exists(layerTraversability_);
  grid_map::Matrix& data = map_[layerElevation_];
  grid_map::Matrix* dataF = existsFriction ? &(map_[layerFriction_]) : NULL;
  grid_map::Matrix* dataR = existsRoughness ? &(map_[layerRoughness_]) : NULL;
  grid_map::Matrix* dataT = existsTraversability ? &(map_[layerTraversability_]) : NULL;
  // setup
  firstCollidingFraction = 1.0;
  flp::Point defaultpt(0,0,0);
  defaultpt.label = 1; // 1:no-collision 2:collision
  defaultpt.friction = std::numeric_limits<double>::infinity();
  defaultpt.roughness = std::numeric_limits<double>::infinity();
  defaultpt.traversability = std::numeric_limits<double>::infinity();
  min = std::vector<flp::Point>(primitive1.size(), defaultpt);
  // get bounding box for each primitive as it is swept from 1 to 2
  std::vector<grid_map::Polygon> boundingBoxes;
  std::vector<Eigen::Vector3d> starts;
  std::vector<Eigen::Vector3d> vectors;
  std::vector<double> lengths;
  std::vector<double> radii;
  for (unsigned int i = 0; i < primitive1.size(); i++) {
    // get radius
    double radius = primitive1[i].sphere.radius;
    if (primitive1[i].sphere.radius != primitive2[i].sphere.radius) {
      std::cout << "WARNING: spheres have different radii, will use conservative collision checking\n";
      radius = std::max(primitive1[i].sphere.radius, primitive2[i].sphere.radius);
    }
    // build top-view bounding box (approximate convex hull)
    Eigen::Vector3d a = primitive1[i].sphere.center;
    Eigen::Vector3d b = primitive2[i].sphere.center;
    Eigen::Vector3d x = (b-a).normalized();
    Eigen::Vector3d z (0,0,1);
    Eigen::Vector3d y = (z.cross(x)).normalized();
    x = y.cross(z);
    Eigen::Vector3d p1 = a - x*radius - y*radius;
    Eigen::Vector3d p2 = b + x*radius - y*radius;
    Eigen::Vector3d p3 = b + x*radius + y*radius;
    Eigen::Vector3d p4 = a - x*radius + y*radius;
    grid_map::Polygon polygon;
    polygon.addVertex(grid_map::Position(p1(0),p1(1)));
    polygon.addVertex(grid_map::Position(p2(0),p2(1)));
    polygon.addVertex(grid_map::Position(p3(0),p3(1)));
    polygon.addVertex(grid_map::Position(p4(0),p4(1)));
    boundingBoxes.push_back(polygon);
    starts.push_back(a);
    vectors.push_back(b-a);
    lengths.push_back(vectors.back().norm());
    radii.push_back(radius);
  }
  // 
  double length = lengths[0];
  double resolution = 0.10;
  std::vector< std::vector<flp::Point> > tmins;
  // for each primitive individually
  for (unsigned int i = 0; i < primitive1.size(); i++) {
    // we store statistics at discrete bins
    std::vector<flp::Point> tmin(length/resolution+1, defaultpt);
    // iterate through points inside bounding box
    grid_map::Position pt;
    for (grid_map::PolygonIterator it(map_, boundingBoxes[i]); !it.isPastEnd(); ++it) {
      const grid_map::Index index(*it);
      map_.getPosition(index, pt);
      // check point is inside the box or above
      flp::Point p(pt.x(), pt.y(), data(index(0),index(1)));
      if (existsFriction) p.friction = (*dataF)(index(0),index(1));
      if (existsRoughness) p.roughness = (*dataR)(index(0),index(1));
      if (existsTraversability) p.traversability = (*dataT)(index(0),index(1));
      double fraction = sqrt(vectors[i].dot(p.p-starts[i])) / lengths[i];
      double zsafe = starts[i](2) + fraction * vectors[i](2) - radii[i];
      int idx = std::min((int)tmin.size()-1, (int)(fraction * lengths[i] / resolution));
      if (p.p(2) >= zsafe)
        tmin[idx].label = 2;
      else
        tmin[idx].label = 1;
      tmin[idx].friction = std::min(tmin[idx].friction, p.friction);
      tmin[idx].roughness = std::min(tmin[idx].roughness, p.roughness);
      tmin[idx].traversability = std::min(tmin[idx].traversability, p.traversability);
    }
    tmins.push_back(tmin);
  }
  // for each timestep
  for (unsigned int i = 0; i < tmins[0].size(); i++) {
    int numCollisions = 0;
    for (unsigned int j = 0; j < tmins.size(); j++) {
      if (tmins[j][i].label == 2) numCollisions++;
      if (tmins[j][i].friction < min[j].friction) min[j].friction = tmins[j][i].friction;
      if (tmins[j][i].roughness < min[j].roughness) min[j].roughness = tmins[j][i].roughness;
      if (tmins[j][i].traversability < min[j].traversability) min[j].traversability = tmins[j][i].traversability;
    }
    if (numCollisions < minPrimitivesColliding) {
      double fraction = (double)i/(double)(tmins[0].size());
      if (fraction < firstCollidingFraction)
        firstCollidingFraction = fraction;
    }
  }
}

std::vector<flp::Point> EnvironmentGridMap::getPointsInside(const flp::Primitive& primitive)
{
  bool existsFriction = map_.exists(layerFriction_);
  bool existsRoughness = map_.exists(layerRoughness_);
  bool existsTraversability = map_.exists(layerTraversability_);
  grid_map::Matrix& data = map_[layerElevation_];
  grid_map::Matrix* dataF = existsFriction ? &(map_[layerFriction_]) : NULL;
  grid_map::Matrix* dataR = existsRoughness ? &(map_[layerRoughness_]) : NULL;
  grid_map::Matrix* dataT = existsTraversability ? &(map_[layerTraversability_]) : NULL;

  // search point
  std::vector<flp::Point> points;
  points.reserve(1000);
  if (primitive.type == flp::PrimitiveTypeSphere) { // sphere
    // range search assuming sphere ~ circle
    double radius = primitive.sphere.radius;
    grid_map::Position center(primitive.sphere.center(0), primitive.sphere.center(1));
    grid_map::Position pt;
    for (grid_map::CircleIterator it(map_, center, radius); !it.isPastEnd(); ++it) {
      const grid_map::Index index(*it);
      double z = data(index(0),index(1));
      if (std::isnan(z))
        continue;
      map_.getPosition(index, pt);
      flp::Point p(pt.x(), pt.y(), z);
      // check point is actually inside the sphere
      if ((p.p-primitive.sphere.center).norm() <= radius) {
        // add data
        if (existsFriction) p.friction = (*dataF)(index(0),index(1));
        if (existsRoughness) p.roughness = (*dataR)(index(0),index(1));
        if (existsTraversability) p.traversability = (*dataT)(index(0),index(1));
        points.push_back(p);
      }
    }
  } else if (primitive.type == flp::PrimitiveTypeCylinder) { // cylinder ... approximate by sphere for now
    // range search assuming cylinder ~ circle
    double radius = primitive.cylinder.radius;
    grid_map::Position center(primitive.cylinder.center(0), primitive.cylinder.center(1));
    grid_map::Position pt;
    for (grid_map::CircleIterator it(map_, center, radius); !it.isPastEnd(); ++it) {
      const grid_map::Index index(*it);
      double z = data(index(0),index(1));
      if (std::isnan(z))
        continue;
      map_.getPosition(index, pt);
      flp::Point p(pt.x(), pt.y(), z);
      // check point is actually inside the cylinder
      if ((p.p-primitive.cylinder.center).norm() <= radius) {
        // add data
        if (existsFriction) p.friction = (*dataF)(index(0),index(1));
        if (existsRoughness) p.roughness = (*dataR)(index(0),index(1));
        if (existsTraversability) p.traversability = (*dataT)(index(0),index(1));
        points.push_back(p);
      }
    }
  } else if (primitive.type == flp::PrimitiveTypeBox) { // box
    // TODO
    std::cout << "TODO\n";
  } else if (primitive.type == flp::PrimitiveTypeSolid) { // solid
    // TODO
    std::cout << "TODO\n";
  }
  return points;
}

flp::Point EnvironmentGridMap::getNearestNeighbor(const flp::Point& point)
{
  // get height
  grid_map::Position position(point.p(0), point.p(1));
  grid_map::Index index;
  if (!map_.getIndex(position, index))
    return point;
  float z = map_.at(layerElevation_, index);
  //if (std::isnan(z)) {
  //  std::cout << "NAN\n";
  //  return point;
  //}

  // return 3D point with correct height
  flp::Point pt = point;
  pt.p(2) = z;
  return pt;
}

int EnvironmentGridMap::getNumIntersectedPoints(const Eigen::Vector3d start, const Eigen::Vector3d target)
{
  // ray cast
  grid_map::Position pos1(start(0), start(1));
  grid_map::Position pos2(target(0), target(1));
  grid_map::Position pos;
  double totaldist = (pos2-pos1).norm();
  int numIntersections = 0;
  int relative =-1; // whether the terrain is under (-1) or above (+1) the ray
  for (grid_map::LineIterator it(map_, pos1, pos2); !it.isPastEnd(); ++it) {
    // get position
    const grid_map::Index index(*it);
    double z = map_.at(layerElevation_, *it);
    if (std::isnan(z))
      z = -std::numeric_limits<double>::infinity();
    map_.getPosition(index, pos);
    // check transition from height>ray to height<ray or back
    double fraction = (pos-pos1).norm() / totaldist;
    double zline = start(2) + (target(2) - start(2)) * fraction;
    if (z < zline && relative > 0) numIntersections++;
    if (z >=zline && relative < 0) numIntersections++;
    if (z >=zline) relative = 1;
    if (z < zline) relative =-1;
  }
  return numIntersections;
}

flp::Point EnvironmentGridMap::sampleUniform(std::mt19937& generator)
{
  bool existsFriction = map_.exists(layerFriction_);
  bool existsRoughness = map_.exists(layerRoughness_);
  bool existsTraversability = map_.exists(layerTraversability_);
  grid_map::Matrix& data = map_[layerElevation_];
  grid_map::Matrix* dataF = existsFriction ? &(map_[layerFriction_]) : NULL;
  grid_map::Matrix* dataR = existsRoughness ? &(map_[layerRoughness_]) : NULL;
  grid_map::Matrix* dataT = existsTraversability ? &(map_[layerTraversability_]) : NULL;

  std::uniform_int_distribution<int> pi (0, map_.getSize()(0) - 1);
  std::uniform_int_distribution<int> pj (0, map_.getSize()(1) - 1);
  for (unsigned int t = 0; t < 100; t++) {
    grid_map::Index index(pi(generator), pj(generator));
    double z = data(index(0),index(1));
    if (std::isnan(z))
      continue;
    grid_map::Position pt;
    map_.getPosition(index, pt);
    flp::Point p(pt.x(), pt.y(), z);
    if (existsFriction) p.friction = (*dataF)(index(0),index(1));
    if (existsRoughness) p.roughness = (*dataR)(index(0),index(1));
    if (existsTraversability) p.traversability = (*dataT)(index(0),index(1));
    return p;
  }
  std::cout << "ERROR: could not sample point.\n";
  return flp::Point(0,0,0);
}

double EnvironmentGridMap::getSpatialResolution()
{
  return map_.getResolution();
}

void EnvironmentGridMap::getSpatialBounds(std::vector<double>& lb, std::vector<double>& ub)
{
  // min-max elevation
  const grid_map::Matrix& data = map_[layerElevation_];
  double minz = std::numeric_limits<double>::infinity();
  double maxz =-std::numeric_limits<double>::infinity();
  for (unsigned int i = 0; i < data.rows(); i++) {
    for (unsigned int j = 0; j < data.cols(); j++) {
      const double val = data(i,j);
      if (std::isnan(val)) continue;
      if (val < minz) minz = val;
      if (val > maxz) maxz = val;
    }
  }

  // min-max xy
  float minx = std::numeric_limits<double>::infinity();
  float maxx =-std::numeric_limits<double>::infinity();
  float miny = std::numeric_limits<double>::infinity();
  float maxy =-std::numeric_limits<double>::infinity();
  for (grid_map::GridMapIterator iterator(map_); !iterator.isPastEnd(); ++iterator) {
    grid_map::Position position;
    map_.getPosition(*iterator, position);
    float x = position.x();
    float y = position.y();
    if (x < minx) minx = x;
    if (y < miny) miny = y;
    if (x > maxx) maxx = x;
    if (y > maxy) maxy = y;
  }
  grid_map::Length length = map_.getLength();
  grid_map::Position center = map_.getPosition();
  lb.resize(3);
  ub.resize(3);
  ub[0] = length(0)/2 + center(0);
  ub[1] = length(1)/2 + center(1);
  ub[2] = maxz;
  lb[0] = ub[0] - length(0);
  lb[1] = ub[1] - length(1);
  lb[2] = minz;
  //printf("Limits x [%f, %f]\n", minx, maxx);
  //printf("Limits y [%f, %f]\n", miny, maxy);
  //printf("Limits z [%f, %f]\n", minz, maxz);
  //std::cout << "center: " << center.transpose() << "\n";
  //std::cout << "length: " << length.transpose() << "\n";

  // min-max xy
  //int maxi = std::max(0, map_.getSize()(0) - 1);
  //int maxj = std::max(0, map_.getSize()(1) - 1);
  //grid_map::Index idx1(maxi, maxj);
  //grid_map::Index idx2(0, 0);
  //grid_map::Position pos1;
  //grid_map::Position pos2;
  //if (!map_.getPosition(idx1, pos1))
  //  printf("could not get map position at borders\n");
  //if (!map_.getPosition(idx2, pos2))
  //  printf("could not get map position at borders\n");
  //lb.resize(3);
  //ub.resize(3);
  //lb[0] = pos1(0);
  //lb[1] = pos1(1);
  //lb[2] = minz;
  //ub[0] = pos2(0);
  //ub[1] = pos2(1);
  //ub[2] = maxz;
}

