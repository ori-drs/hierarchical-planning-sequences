#pragma once
#include "flp.h"
#include "State.h"
#include "StateSpace.h"
#include "Environment.h"
#include <boost/shared_ptr.hpp>

// update state
// update environment
// show

class Viewer
{
public:
  typedef boost::shared_ptr<Viewer> Ptr;

  enum Colormap {
    ColormapAutumn,
    ColormapCool,
    ColormapHSV
  };

  Viewer() : environmentAlpha_(1.0), environmentColormap_(ColormapAutumn), environmentColorField_("elevation") {}
  virtual void updateEnvironment(const Environment::Ptr& environment) = 0;
  virtual void updateRobot(const State::Ptr& state) = 0;
  virtual void setTransparencyRobot(double percent) = 0;

  //TODO: addRobotModel
  //TODO: clearAll (and remove the other clear functions for simplicity?)

  virtual void addPrimitives(const std::vector<flp::Primitive>& primitives) = 0;
  virtual void clearPrimitives() = 0;

  virtual void addLineList(const std::vector<Eigen::Vector3d>& lineList, const Eigen::Vector4d& color) = 0;
  virtual void addLineList(const std::vector<Eigen::Vector3d>& lineList, const std::vector<Eigen::Vector4d>& lineColors) = 0;
  virtual void clearLineList() = 0;

  virtual void draw() = 0;
  virtual void drawAndIdle() = 0;

  double environmentAlpha_;
  Colormap environmentColormap_;
  std::string environmentColorField_;

protected:
  static const float autumn_r[64];
  static const float autumn_g[64];
  static const float autumn_b[64];
  static const float cool_r[64];
  static const float cool_g[64];
  static const float cool_b[64];
  static const float hsv_r[64];
  static const float hsv_g[64];
  static const float hsv_b[64];
};

