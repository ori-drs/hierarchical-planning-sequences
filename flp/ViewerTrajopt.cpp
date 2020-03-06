#include "ViewerTrajopt.h"

ViewerTrajopt::ViewerTrajopt() : ViewerRAVEBase()
{
  viewer_ = OSGViewer::GetOrCreate(raveEnv_);
}

ViewerTrajopt::ViewerTrajopt(const std::string& robotModelPath) : ViewerRAVEBase(robotModelPath)
{
  viewer_ = OSGViewer::GetOrCreate(raveEnv_);
}

void ViewerTrajopt::setTransparencyRobot(double percent)
{
  viewer_->SetTransparency(raveRobot_, percent/100);
}

void ViewerTrajopt::clearPrimitives()
{
  for (unsigned int i = 0 ; i < primitives_.size(); i++) {
    raveEnv_->Remove(primitives_[i]);
    viewer_->RemoveKinBody(primitives_[i]);
  }
  primitives_.clear();
}

void ViewerTrajopt::addLineList(const std::vector<Eigen::Vector3d>& lineList, const Eigen::Vector4d& color)
{
  if (lineList.size() < 2) {
    printf("line list is empty.\n");
    return;
  }
  unsigned int size = lineList.size();
  float *pts = new float[3*size];
  for (unsigned int i = 0; i < size; i++) {
    pts[i*3+0] = lineList[i](0);
    pts[i*3+1] = lineList[i](1);
    pts[i*3+2] = lineList[i](2);
  }
  viewer_handle_lines_.push_back(viewer_->drawlinelist(pts, size, 3*sizeof(float), 10, OpenRAVE::Vector(color(0),color(1),color(2),color(3))));
  delete[] pts;
}

void ViewerTrajopt::draw()
{
  viewer_->Draw();
  usleep(1000);
}

void ViewerTrajopt::drawAndIdle()
{
  viewer_->Idle();
}

