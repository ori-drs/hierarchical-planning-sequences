#include "RecastPlanner.h"
#include "MySample.h"
#include "Recast.h"
#include "DetourNavMesh.h"
#include "DetourNavMeshQuery.h"
#include "DetourCommon.h"
#include <Eigen/Dense>
#include <pcl/io/vtk_lib_io.h>

using namespace recastapp;

RecastPlanner::RecastPlanner () : needToRotateMesh(true)
{
  stg.cellSize = 0.3f;
  stg.cellHeight = 0.2f;
  stg.agentHeight = 2.0f;
  stg.agentRadius = 0.6f;
  stg.agentMaxClimb = 0.9f;
  stg.agentMaxSlope = 45.0f;
  stg.regionMinSize = 8;
  stg.regionMergeSize = 20;
  stg.edgeMaxLen = 12.0f;
  stg.edgeMaxError = 1.3f;
  stg.vertsPerPoly = 6.0f;
  stg.detailSampleDist = 6.0f;
  stg.detailSampleMaxError = 1.0f;
  stg.partitionType = SAMPLE_PARTITION_WATERSHED;
}

bool RecastPlanner::build (const pcl::PolygonMesh& pclMesh)
{
  // Build navmesh
  geom = boost::shared_ptr<InputGeom>(new InputGeom());
  if (!geom->load(&ctx, pclMesh, needToRotateMesh))
    return false;
  geom->setBuildSettings(stg);
  sample = boost::shared_ptr<Sample>(new MySample());
  sample->setContext(&ctx);
  sample->handleMeshChanged(geom.get());
  sample->handleSettings();
  sample->handleBuild();
  return true;
}

bool RecastPlanner::loadAndBuild (const std::string& file)
{
  // Load mesh file
  pcl::PolygonMesh pclMesh;
  if (!pcl::io::loadPolygonFile (file, pclMesh))
    return false;
  return build (pclMesh);
}

bool RecastPlanner::query (const pcl::PointXYZ& start, const pcl::PointXYZ& end, std::vector<pcl::PointXYZ>& path)
{
  if (!sample)
    return false;
  dtNavMesh* navmesh = sample->getNavMesh();
  dtNavMeshQuery* navquery = sample->getNavMeshQuery();
  if (!navmesh || !navquery) return false;

  static const int MAX_POLYS = 256;
  dtPolyRef polys[MAX_POLYS];
  float straight[MAX_POLYS*3];
  const float polyPickExt[3] = {2,4,2};
  int npolys = 0;
  int nstraight = 0;

  dtQueryFilter filter;
  filter.setIncludeFlags(0x3);
  filter.setExcludeFlags(0x0);

  // Convert
  float spos[3];
  float epos[3];
  float nspos[3];
  float nepos[3];
  if (needToRotateMesh) {
    spos[0] = start.x;
    spos[1] = start.z;
    spos[2] =-start.y;
    epos[0] = end.x;
    epos[1] = end.z;
    epos[2] =-end.y;
  } else {
    spos[0] = start.x;
    spos[1] = start.y;
    spos[2] = start.z;
    epos[0] = end.x;
    epos[1] = end.y;
    epos[2] = end.z;
  }

  // Find start points	
  dtPolyRef startRef, endRef;
  navquery->findNearestPoly(spos, polyPickExt, &filter, &startRef, nspos);
  navquery->findNearestPoly(epos, polyPickExt, &filter, &endRef, nepos);
  if (!startRef || !endRef)
    return false;

  // Find path
  bool foundFullPath = false;
  if (navquery->findPath(startRef, endRef, spos, epos, &filter, polys, &npolys, MAX_POLYS) == DT_SUCCESS)
    foundFullPath = true;

  // Find straight path
  if (npolys)
    navquery->findStraightPath(spos, epos, polys, npolys, straight, 0, 0, &nstraight, MAX_POLYS);
  else
    return false;

  // Convert
  path.resize(nstraight);
  if (needToRotateMesh) {
  for (int i = 0; i < nstraight; i++) {
    path[i].x = straight[i*3+0]; // in Recast x points front
    path[i].y =-straight[i*3+2]; // in Recast z points right
    path[i].z = straight[i*3+1]; // in Recast y points up
  }
  } else {
    for (int i = 0; i < nstraight; i++) {
      path[i].x = straight[i*3+0];
      path[i].y = straight[i*3+1];
      path[i].z = straight[i*3+2];
    }
  }
  return foundFullPath;
}

bool RecastPlanner::getNavMesh (pcl::PolygonMesh::Ptr& pclmesh, pcl::PointCloud<pcl::PointXYZ>::Ptr& pclcloud, std::vector<Eigen::Vector3d>& lineList) const
{
  if (!sample) return false;
  const dtNavMesh* mesh = sample->getNavMesh();
  if (!mesh) return false;

  pclmesh.reset( new pcl::PolygonMesh() );
  pclcloud.reset( new pcl::PointCloud<pcl::PointXYZ>() );

  // go through all tiles
  pclcloud->points.reserve(mesh->getMaxTiles() * 10 * 3);
  for (int i = 0; i < mesh->getMaxTiles(); ++i) {
    const dtMeshTile* tile = mesh->getTile(i);
    if (!tile->header) continue;
    dtPolyRef base = mesh->getPolyRefBase(tile);
    int tileNum = mesh->decodePolyIdTile(base);
    for (int i = 0; i < tile->header->polyCount; ++i) {
      const dtPoly* p = &tile->polys[i];
      if (p->getType() == DT_POLYTYPE_OFFMESH_CONNECTION)	// Skip off-mesh links.
        continue;
      const dtPolyDetail* pd = &tile->detailMeshes[i];
      // go through all triangles
      for (int j = 0; j < pd->triCount; ++j) {
        const unsigned char* t = &tile->detailTris[(pd->triBase+j)*4];
        for (int k = 0; k < 3; ++k) {
          const float* verts;
          if (t[k] < p->vertCount)
            verts = &tile->verts[p->verts[t[k]]*3];
          else
            verts = &tile->detailVerts[(pd->vertBase+t[k]-p->vertCount)*3];
          // add vertex to pcl cloud
          pcl::PointXYZ vert;
          vert.x = verts[0];
          vert.y =-verts[2];
          vert.z = verts[1];
          pclcloud->points.push_back(vert);
        }
      }
    }
  }

  // build pcl mesh
  pcl::toPCLPointCloud2 (*pclcloud, pclmesh->cloud);
  int ntri = pclcloud->points.size()/3;
  pclmesh->polygons.resize(ntri);
  for (int i = 0; i < ntri; i++) {
    pclmesh->polygons[i].vertices.resize(3);
    pclmesh->polygons[i].vertices[0] = i*3+0;
    pclmesh->polygons[i].vertices[1] = i*3+1;
    pclmesh->polygons[i].vertices[2] = i*3+2;
  }

  // line list
  lineList.reserve(ntri*6);
  for (int i = 0; i < ntri; i++) {
    const pcl::PointXYZ& p1 = pclcloud->points[ pclmesh->polygons[i].vertices[0] ];
    const pcl::PointXYZ& p2 = pclcloud->points[ pclmesh->polygons[i].vertices[1] ];
    const pcl::PointXYZ& p3 = pclcloud->points[ pclmesh->polygons[i].vertices[2] ];
    Eigen::Vector3d v1 (p1.x, p1.y, p1.z);
    Eigen::Vector3d v2 (p2.x, p2.y, p2.z);
    Eigen::Vector3d v3 (p3.x, p3.y, p3.z);
    lineList.push_back(v1);
    lineList.push_back(v2);
    lineList.push_back(v2);
    lineList.push_back(v3);
    lineList.push_back(v3);
    lineList.push_back(v1);
  }
  return true;
}

bool RecastPlanner::getNavMesh (pcl::PolygonMesh::Ptr& pclmesh) const
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr pclcloud;
  std::vector<Eigen::Vector3d> lineList;
  return getNavMesh(pclmesh, pclcloud, lineList);
}

bool RecastPlanner::getNavMesh (std::vector<Eigen::Vector3d>& lineList) const
{
  pcl::PolygonMesh::Ptr pclmesh;
  pcl::PointCloud<pcl::PointXYZ>::Ptr pclcloud;
  return getNavMesh(pclmesh, pclcloud, lineList);
}

