#pragma once
#include "Viewer.h"
#include "EnvironmentPCL.h"
#include "EnvironmentGridMap.h"
#include "EnvironmentRecast.h"
#include "StateSpace.h"
#include <openrave-core.h>

class ViewerRAVEBase : public Viewer
{
public:
  typedef boost::shared_ptr<ViewerRAVEBase> Ptr;
  ViewerRAVEBase();
  ViewerRAVEBase(const std::string& robotModelPath);

  virtual void updateEnvironment(const Environment::Ptr& environment);
  virtual void updateRobot(const State::Ptr& state);
  virtual void setTransparencyRobot(double percent);

  virtual void addPrimitives(const std::vector<flp::Primitive>& primitives);
  virtual void clearPrimitives();

  virtual void addLineList(const std::vector<Eigen::Vector3d>& lineList, const Eigen::Vector4d& color);
  virtual void addLineList(const std::vector<Eigen::Vector3d>& lineList, const std::vector<Eigen::Vector4d>& lineColors);
  virtual void clearLineList();

  virtual void draw();
  virtual void drawAndIdle();

protected:
  void init(const std::string& robotModelPath);
  void removeMesh();
  void updateEnvironment(const EnvironmentPCL& environment);
  void updateEnvironment(const EnvironmentGridMap& environment);
  void updateEnvironment(const EnvironmentRecast& environment);
  OpenRAVE::EnvironmentBasePtr raveEnv_;
  OpenRAVE::KinBodyPtr raveMesh_;
  OpenRAVE::RobotBasePtr raveRobot_;
  StateSpaceJointsSE3::Ptr robotStateSpace_;

  OpenRAVE::GraphHandlePtr viewer_handle_cloud_;
  std::vector<OpenRAVE::GraphHandlePtr> viewer_handle_lines_;
  std::vector<OpenRAVE::GraphHandlePtr> viewer_handle_mesh_;
  std::vector<OpenRAVE::KinBodyPtr> primitives_;
};

class ViewerRAVE : public ViewerRAVEBase
{
public:
  typedef boost::shared_ptr<ViewerRAVE> Ptr;
  ViewerRAVE();
  ViewerRAVE(const std::string& robotModelPath);
  ~ViewerRAVE();
protected:
  void runViewer();
  boost::thread viewerThread_;
  OpenRAVE::ViewerBasePtr viewer_;
};

