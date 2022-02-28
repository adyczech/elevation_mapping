/*
 * PointXYZConfidenceRatio.cpp
 *
 *  Created on: Nov 26, 2020
 *      Author: Magnus Gärtner
 *   Institute: ETH Zurich, ANYbotics
 *
 *   This file contains explicit definitions of algorithms used with our custom pcl type.
 */

#define PCL_NO_PRECOMPILE
#include "elevation_mapping/PointXYZConfidenceRatio.hpp"

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/pcl_base.h>
#include <pcl/point_cloud.h>

template class pcl::PointCloud<pcl::PointXYZConfidenceRatio>;
template class pcl::PCLBase<pcl::PointXYZConfidenceRatio>;    // NOLINT(cppcoreguidelines-special-member-functions)
template class pcl::VoxelGrid<pcl::PointXYZConfidenceRatio>;  // NOLINT(cppcoreguidelines-special-member-functions)
template void pcl::removeNaNFromPointCloud<pcl::PointXYZConfidenceRatio>(
    const pcl::PointCloud<pcl::PointXYZConfidenceRatio>& cloud_in, pcl::PointCloud<pcl::PointXYZConfidenceRatio>& cloud_out,
    std::vector<int, std::allocator<int> >& index);
template class pcl::ExtractIndices<pcl::PointXYZConfidenceRatio>;
template class pcl::PassThrough<pcl::PointXYZConfidenceRatio>;

std::ostream& operator<<(std::ostream& os, const pcl::PointXYZConfidenceRatio& p) {
  os << "(" << p.x << "," << p.y << "," << p.z << " - " << p.confidence_ratio << ")";                     // NOLINT(cppcoreguidelines-pro-type-union-access)
  return (os);
}
