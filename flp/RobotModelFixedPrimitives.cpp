#include "RobotModelFixedPrimitives.h"
#include "StateSpaceBaseSwitchContacts.h"
#include <Eigen/Geometry>

RobotModelFixedPrimitives::RobotModelFixedPrimitives(const StateSpace::Ptr& space, const std::vector<flp::Primitive>& collision, const std::vector<flp::Primitive>& contact)
  : RobotModel(space)
  , collision_(collision)
  , contact_(contact)
{

}

std::vector<flp::Primitive> RobotModelFixedPrimitives::getPrimitivesCollision(const State::Ptr& state)
{
  // get transform
  flp::Transform T;
  if (!state->space_->getTransform(state, T))
    return std::vector<flp::Primitive>();
  Eigen::Transform<double,3,Eigen::Affine> eiT = T.getEigenTransform();
  // apply transform
  std::vector<flp::Primitive> primitives;
  for (unsigned int i = 0; i < collision_.size(); i++) {
    flp::Primitive pri = collision_[i];
    pri.transform(eiT);
    primitives.push_back(pri);
  }
  return primitives;
}

std::vector<flp::Primitive> RobotModelFixedPrimitives::getPrimitivesContact(const State::Ptr& state)
{
  // see if we can get contact points from state
  std::vector<Eigen::Vector3d> contactPoints;
  if (state->space_->getContacts(state, contactPoints) && contactPoints.size() == contact_.size()) {
    std::vector<flp::Primitive> primitives;
    for (unsigned int i = 0; i < contact_.size(); i++) {
      flp::Primitive pri = contact_[i];
      if (pri.type == flp::PrimitiveTypeSphere) {
        pri.sphere.center = contactPoints[i];
      } else if (pri.type == flp::PrimitiveTypeCylinder) {
        pri.cylinder.center = contactPoints[i];
      } else {
        std::cout << "WARNING: primitive type not handled.\n";
        continue;
      }
      primitives.push_back(pri);
    }
    return primitives;
  }
  // get transform
  flp::Transform T;
  if (!state->space_->getTransform(state, T))
    return std::vector<flp::Primitive>();
  Eigen::Transform<double,3,Eigen::Affine> eiT = T.getEigenTransform();
  // apply transform
  std::vector<flp::Primitive> primitives;
  for (unsigned int i = 0; i < contact_.size(); i++) {
    flp::Primitive pri = contact_[i];
    pri.transform(eiT);
    primitives.push_back(pri);
  }
  return primitives;
}

// TODO: this should return a state in the same space as the input. i.e. we need re-projections from SE3 to all states, 
//       or to define getTransform() and setTransform() to all states or spaces
State::Ptr RobotModelFixedPrimitives::project(const State::Ptr& state, const Environment::Ptr& environment)
{
  State::Ptr proj = state->clone();

  // get, project and set contacts if we can
  std::vector<Eigen::Vector3d> contacts;
  if (state->space_->getContacts(state, contacts)) {
    for (unsigned int i = 0; i < contacts.size(); i++) {
      // get closest 3D point
      flp::Point pt(contacts[i]);
      flp::Point nn = environment->getNearestNeighbor(pt);
      if (!std::isnan(nn.p(2)))
        contacts[i] = nn.p + contactProjectionOffset_;
    }
    // set new contacts
    if (state->space_->setContacts(proj, contacts))
      return proj;
  }

  // get, project and set transform if we can
  flp::Transform T;
  if (state->space_->getTransform(state, T)) {
    std::vector<flp::Primitive> primitivesContact = getPrimitivesContact(state);
    if (primitivesContact.size() == 0)
      return State::Ptr();

    // compute avg of adjustments to each contact
    Eigen::Vector3d sum(0,0,0);
    for (unsigned int i = 0; i < primitivesContact.size(); i++) {
      if (primitivesContact[i].type == flp::PrimitiveTypeSphere) {
        flp::Point pt(primitivesContact[i].sphere.center);
        flp::Point nn = environment->getNearestNeighbor(pt);
        if (!std::isnan(nn.p(2)))
          sum += (nn.p - pt.p);
      } else if (primitivesContact[i].type == flp::PrimitiveTypeCylinder) {
        flp::Point pt(primitivesContact[i].cylinder.center);
        flp::Point nn = environment->getNearestNeighbor(pt);
        if (!std::isnan(nn.p(2)))
          sum += (nn.p - pt.p);
      } else {
        printf("TODO: other primitives\n");
      }
    }
    Eigen::Vector3d diff = (sum / (double)primitivesContact.size()) + contactProjectionOffset_;

    // set new transform
    T.x += diff(0);
    T.y += diff(1);
    T.z += diff(2);
    if (state->space_->setTransform(proj, T))
      return proj;
  }

  return State::Ptr();
}

Eigen::Vector3d RobotModelFixedPrimitives::sampleUniformCOP2COM(std::mt19937& generator)
{
  double minz = std::numeric_limits<double>::infinity();
  double maxz =-std::numeric_limits<double>::infinity();
  for (unsigned int i = 0; i < contact_.size(); i++) {
    const flp::Primitive& pri = contact_[i];
    if (pri.type == flp::PrimitiveTypeSphere) {
      double z1 = pri.sphere.center(2) - pri.sphere.radius;
      double z2 = pri.sphere.center(2) + pri.sphere.radius;
      if (z1 < minz) minz = z1;
      if (z2 < minz) minz = z2;
      if (z1 > maxz) maxz = z1;
      if (z2 > maxz) maxz = z2;
    } else if (pri.type == flp::PrimitiveTypeCylinder) {
      double z1 = pri.cylinder.center(2) - pri.cylinder.radius;
      double z2 = pri.cylinder.center(2) + pri.cylinder.radius;
      if (z1 < minz) minz = z1;
      if (z2 < minz) minz = z2;
      if (z1 > maxz) maxz = z1;
      if (z2 > maxz) maxz = z2;
    } else {
      std::cout << "WARNING: primitive type not handled.\n";
      continue;
    }
  }
  std::uniform_real_distribution<double> dis(minz, maxz);
  return Eigen::Vector3d(0, 0, -dis(generator));
}

