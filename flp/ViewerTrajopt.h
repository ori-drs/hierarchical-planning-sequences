#pragma once
#include "ViewerRAVE.h"
#include "osgviewer/osgviewer.hpp"

class ViewerTrajopt : public ViewerRAVEBase
{
public:
  typedef boost::shared_ptr<ViewerTrajopt> Ptr;
  ViewerTrajopt();
  ViewerTrajopt(const std::string& robotModelPath);
  void setTransparencyRobot(double percent);
  void clearPrimitives();
  void addLineList(const std::vector<Eigen::Vector3d>& lineList, const Eigen::Vector4d& color);
  void draw();
  void drawAndIdle();
protected:
  OSGViewerPtr viewer_;
};

