/*
 * StereoSensorProcessor.cpp
 *
 *  Created on: Jun 6, 2014
 *      Author: Hannes Keller
 */

#pragma once

#include "elevation_mapping/PointXYZConfidenceRatio.hpp"
#include "elevation_mapping/sensor_processors/SensorProcessorBase.hpp"

namespace elevation_mapping {

/*!
 * Sensor processor for stereo camera sensors.
 * Cleans the point cloud, transforms it to a desired frame, and
 * computes the measurement variances based on a sensor model in
 * the desired frame.
 *
 * "Localization and Path Planning of a Climbing Robot for Corrosion Monitoring", Hannes Keller, Semester Project, ETH Zurich, 2014.
 */

class StereoSensorProcessor : public SensorProcessorBase {
 public:
  /*!
   * Constructor.
   * @param node the shared pointer to the ROS node node handle.
   */
  StereoSensorProcessor(rclcpp::Node::SharedPtr node, const SensorProcessorBase::GeneralParameters& generalParameters);

  /*!
   * Destructor.
   */
  ~StereoSensorProcessor() override;

 private:
  /*!
   * Reads and verifies the parameters.
   * @param[in] processorNamespace parameter namespace to read parameters from
   * @return true if successful.
   */
  bool readParameters(std::string processorNamespace) override;

  /*!
   * Computes the elevation map height variances for each point in a point cloud with the
   * sensor model and the robot pose covariance.
   * @param[in] pointCloud the point cloud for which the variances are computed.
   * @param[in] robotPoseCovariance the robot pose covariance matrix.
   * @param[out] variances the elevation map height variances.
   * @return true if successful.
   */
  bool computeVariances(const PointCloudType::ConstPtr pointCloud, const Eigen::Matrix<double, 6, 6>& robotPoseCovariance,
                        Eigen::VectorXf& variances) override;

  /*!
   * Cuts off points that are not within the cutoff interval
   * @param pointCloud the point cloud to filter.
   * @return true if successful.
   */
  bool filterPointCloudSensorType(const PointCloudType::Ptr pointCloud) override;

  //! Helper functions to get i-j indices out of a single index.
  int getI(int index);
  int getJ(int index);

  //! Stores 'original' point cloud indices of the cleaned point cloud.
  std::vector<int> indices_;
  int originalWidth_;
};

}  // namespace elevation_mapping
