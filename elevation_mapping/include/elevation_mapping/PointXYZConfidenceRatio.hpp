/*
 * PointXYZConfidenceRatio.hpp
 *
 *  Created on: Nov 26, 2020
 *      Author: Magnus Gärtner
 *   Institute: ETH Zurich, ANYbotics
 *
 *   This file defines our custom pcl type, ie including a confidence_ratio.
 *   Adapted from https://github.com/PointCloudLibrary/pcl/blob/master/common/include/pcl/impl/point_types.hpp
 */

#pragma once

#include <pcl/pcl_macros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace pcl {

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
struct _PointXYZConfidenceRatio {  // NOLINT(cppcoreguidelines-pro-type-union-access)
  PCL_ADD_POINT4D;  // NOLINT(cppcoreguidelines-pro-type-union-access, readability-const-return-type, modernize-avoid-c-arrays) This adds
                    // the members x,y,z which can also be accessed using the point (which is float[4])
  union {
    struct {
      float confidence_ratio;  // NOLINT(readability-identifier-naming)
    };
    float data_c[4];  // NOLINT(readability-identifier-naming, modernize-avoid-c-arrays)
  };
  PCL_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;  // enforce SSE padding for correct memory alignment
#pragma GCC diagnostic pop

struct PointXYZConfidenceRatio : public _PointXYZConfidenceRatio {
  inline explicit PointXYZConfidenceRatio(const _PointXYZConfidenceRatio& p) : _PointXYZConfidenceRatio() {
    // XZY
    x = p.x;         // NOLINT(cppcoreguidelines-pro-type-union-access)
    y = p.y;         // NOLINT(cppcoreguidelines-pro-type-union-access)
    z = p.z;         // NOLINT(cppcoreguidelines-pro-type-union-access)
    data[3] = 1.0f;  // NOLINT(cppcoreguidelines-pro-type-union-access)

    // Confidence
    confidence_ratio = p.confidence_ratio;  // NOLINT(cppcoreguidelines-pro-type-union-access)
  }

  inline explicit PointXYZConfidenceRatio(float _confidence_ratio = 1.f)
      : PointXYZConfidenceRatio(0.f, 0.f, 0.f, _confidence_ratio) {}

  inline PointXYZConfidenceRatio(float _x, float _y, float _z, float _confidence_ratio = 1.f)
      : _PointXYZConfidenceRatio() {
    x = _x;          // NOLINT(cppcoreguidelines-pro-type-union-access)
    y = _y;          // NOLINT(cppcoreguidelines-pro-type-union-access)
    z = _z;          // NOLINT(cppcoreguidelines-pro-type-union-access)
    data[3] = 1.0f;  // NOLINT(cppcoreguidelines-pro-type-union-access)

    confidence_ratio = _confidence_ratio;  // NOLINT(cppcoreguidelines-pro-type-union-access)
  }

  friend std::ostream& operator<<(std::ostream& os, const PointXYZConfidenceRatio& p);
};

PCL_EXPORTS std::ostream& operator<<(std::ostream& os, const PointXYZConfidenceRatio& p);

}  // namespace pcl

namespace elevation_mapping {
using PointCloudType = pcl::PointCloud<pcl::PointXYZConfidenceRatio>;
}  // namespace elevation_mapping

POINT_CLOUD_REGISTER_POINT_STRUCT(pcl::_PointXYZConfidenceRatio,  // NOLINT(modernize-avoid-c-arrays, readability-const-return-type) here
                                                                     // we assume a XYZ + "confidence_ratio" (as fields)
                                  (float, x, x)                      // NOLINT
                                  (float, y, y)                      // NOLINT
                                  (float, z, z)                      // NOLINT
                                  (float, confidence_ratio, confidence_ratio))  // NOLINT

POINT_CLOUD_REGISTER_POINT_WRAPPER(pcl::PointXYZConfidenceRatio, pcl::_PointXYZConfidenceRatio)
