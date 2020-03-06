#pragma once
#include "EnvironmentPCL.h"
#include "EnvironmentGridMap.h"
#include "EnvironmentRecast.h"

bool convert(const grid_map::GridMap& gridmap, pcl::PolygonMesh& mesh);
bool convert(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, grid_map::GridMap& gridmap, double resolution);

