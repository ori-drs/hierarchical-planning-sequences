#include "Sample.h"
#include "InputGeom.h"
#include <pcl/PolygonMesh.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <boost/shared_ptr.hpp>
#include <string>

namespace recastapp
{

  class RecastPlanner
  {
  public:
    RecastPlanner();
    bool build (const pcl::PolygonMesh& pclMesh);
    bool loadAndBuild (const std::string& file);
    bool query (const pcl::PointXYZ& start, const pcl::PointXYZ& end, std::vector<pcl::PointXYZ>& path);
    bool getNavMesh (pcl::PolygonMesh::Ptr& pclmesh, pcl::PointCloud<pcl::PointXYZ>::Ptr& pclcloud, std::vector<Eigen::Vector3d>& lineList) const;
    bool getNavMesh (pcl::PolygonMesh::Ptr& pclmesh) const;
    bool getNavMesh (std::vector<Eigen::Vector3d>& lineList) const;
    BuildSettings stg;
  private:
    BuildContext ctx;
    boost::shared_ptr<Sample> sample;
    boost::shared_ptr<InputGeom> geom;
    bool needToRotateMesh;
  };

}
