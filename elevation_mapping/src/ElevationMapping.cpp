/*
 * ElevationMapping.cpp
 *
 *  Created on: Nov 12, 2013
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, ANYbotics
 */

#include <cmath>
#include <string>
#include <chrono>

#include <grid_map_msgs/msg/grid_map.hpp>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <boost/bind.hpp>
#include <boost/thread/recursive_mutex.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <kindr/Core>
#include <kindr_ros/kindr_ros.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "elevation_mapping/ElevationMap.hpp"
#include "elevation_mapping/ElevationMapping.hpp"
#include "elevation_mapping/PointXYZRGBConfidenceRatio.hpp"
#include "elevation_mapping/sensor_processors/SensorProcessorBase.hpp"

using namespace std::literals::chrono_literals;

namespace elevation_mapping {

ElevationMapping::ElevationMapping(const rclcpp::NodeOptions & options)
    : Node("elevation_mapping", options),
      robotPoseCacheSize_(200),
      ignoreRobotMotionUpdates_(false),
      updatesEnabled_(true),
      isContinuouslyFusing_(false),
      receivedFirstMatchingPointcloudAndPose_(false),
      initializeElevationMap_(false),
      initializationMethod_(0),
      lengthInXInitSubmap_(1.2),
      lengthInYInitSubmap_(1.8),
      marginInitSubmap_(0.3),
      initSubmapHeightOffset_(0.0) {
#ifndef NDEBUG
  // Print a warning if built in debug.
  RCLCPP_WARN(this->get_logger(), "CMake Build Type is 'Debug'. Change to 'Release' for better performance.");
#endif

  RCLCPP_INFO(this->get_logger(), "Elevation mapping node started.");

  // Initialize the tf buffer here to give it a head start
  transformBuffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  transformListener_ = std::make_shared<tf2_ros::TransformListener>(*transformBuffer_);

  // We need to use shared_from_this to initialize some of the member objects, so create this timer that will 
  // execute a callback 100ms after the node has been constructed.
  initTimer_ = this->create_wall_timer(100ms, std::bind(&ElevationMapping::initializeNode, this));

  RCLCPP_DEBUG(this->get_logger(), "Waiting for node initialization timer (100ms).");
}

void ElevationMapping::initializeNode() {
  RCLCPP_DEBUG(this->get_logger(), "Node initialization timer callback executed.");

  map_ = std::make_shared<ElevationMap>(shared_from_this());
  robotMotionMapUpdater_ = std::make_shared<RobotMotionMapUpdater>(shared_from_this());
  inputSources_ = std::make_shared<InputSourceManager>(shared_from_this());
  
  readParameters();
  setupSubscribers();
  setupServices();
  setupTimers();

  initialize();

  initTimer_.cancel();  // Cancel the timer, as it is no longer needed

  RCLCPP_INFO(this->get_logger(), "Successfully launched node.");
}

void ElevationMapping::setupSubscribers() {  // Handle input_sources configuration.
  const bool configuredInputSources = inputSources_->configureFromRos("input_sources");

  if (configuredInputSources) {
    inputSources_->registerCallbacks(*this, make_pair("pointcloud", &ElevationMapping::pointCloudCallback));
  }

  if (!robotPoseTopic_.empty()) {
    robotPoseSubscriber_.subscribe(node_, robotPoseTopic_, 1);
    robotPoseCache_.connectInput(robotPoseSubscriber_);
    robotPoseCache_.setCacheSize(robotPoseCacheSize_);
  } else {
    ignoreRobotMotionUpdates_ = true;
  }
}

void ElevationMapping::setupServices() {
  // Multi-threading for fusion.
  rclcpp::AdvertiseServiceOptions advertiseServiceOptionsForTriggerFusion = rclcpp::AdvertiseServiceOptions::create<std_srvs::srv::Empty>(
      "trigger_fusion", boost::bind(&ElevationMapping::fuseEntireMapServiceCallback, this, _1, _2), rclcpp::VoidConstPtr(),
      &fusionServiceQueue_);
  fusionTriggerService_ = node_->advertiseService(advertiseServiceOptionsForTriggerFusion);

  rclcpp::AdvertiseServiceOptions advertiseServiceOptionsForGetFusedSubmap = rclcpp::AdvertiseServiceOptions::create<grid_map_msgs::srv::GetGridMap>(
      "get_submap", boost::bind(&ElevationMapping::getFusedSubmapServiceCallback, this, _1, _2), rclcpp::VoidConstPtr(), &fusionServiceQueue_);
  fusedSubmapService_ = node_->advertiseService(advertiseServiceOptionsForGetFusedSubmap);

  rclcpp::AdvertiseServiceOptions advertiseServiceOptionsForGetRawSubmap = rclcpp::AdvertiseServiceOptions::create<grid_map_msgs::srv::GetGridMap>(
      "get_raw_submap", boost::bind(&ElevationMapping::getRawSubmapServiceCallback, this, _1, _2), rclcpp::VoidConstPtr(),
      &fusionServiceQueue_);
  rawSubmapService_ = node_->advertiseService(advertiseServiceOptionsForGetRawSubmap);

  clearMapService_ = node_->advertiseService("clear_map", &ElevationMapping::clearMapServiceCallback, this);
  enableUpdatesService_ = node_->advertiseService("enable_updates", &ElevationMapping::enableUpdatesServiceCallback, this);
  disableUpdatesService_ = node_->advertiseService("disable_updates", &ElevationMapping::disableUpdatesServiceCallback, this);
  maskedReplaceService_ = node_->advertiseService("masked_replace", &ElevationMapping::maskedReplaceServiceCallback, this);
  saveMapService_ = node_->advertiseService("save_map", &ElevationMapping::saveMapServiceCallback, this);
  loadMapService_ = node_->advertiseService("load_map", &ElevationMapping::loadMapServiceCallback, this);
}

void ElevationMapping::setupTimers() {
  mapUpdateTimer_ = node_->createTimer(maxNoUpdateDuration_, &ElevationMapping::mapUpdateTimerCallback, this, true, false);

  if (!fusedMapPublishTimerDuration_.isZero()) {
    rclcpp::TimerOptions timerOptions =
        rclcpp::TimerOptions(fusedMapPublishTimerDuration_, boost::bind(&ElevationMapping::publishFusedMapCallback, this, _1),
                          &fusionServiceQueue_, false, false);
    fusedMapPublishTimer_ = node_->createTimer(timerOptions);
  }

  // Multi-threading for visibility cleanup. Visibility clean-up does not help when continuous clean-up is enabled.
  if (map_->enableVisibilityCleanup_ && !visibilityCleanupTimerDuration_.isZero() && !map_->enableContinuousCleanup_) {
    rclcpp::TimerOptions timerOptions =
        rclcpp::TimerOptions(visibilityCleanupTimerDuration_, boost::bind(&ElevationMapping::visibilityCleanupCallback, this, _1),
                          &visibilityCleanupQueue_, false, false);
    visibilityCleanupTimer_ = node_->createTimer(timerOptions);
  }
}

ElevationMapping::~ElevationMapping() {
  // Shutdown all services.

  {  // Fusion Service Queue
    rawSubmapService_.shutdown();
    fusionTriggerService_.shutdown();
    fusedSubmapService_.shutdown();
    fusedMapPublishTimer_.stop();

    fusionServiceQueue_.disable();
    fusionServiceQueue_.clear();
  }

  {  // Visibility cleanup queue
    visibilityCleanupTimer_.stop();

    visibilityCleanupQueue_.disable();
    visibilityCleanupQueue_.clear();
  }

  node_->shutdown();

  // Join threads.
  if (fusionServiceThread_.joinable()) {
    fusionServiceThread_.join();
  }
  if (visibilityCleanupThread_.joinable()) {
    visibilityCleanupThread_.join();
  }
}

bool ElevationMapping::readParameters() {
  // ElevationMapping parameters.
  node_->param("point_cloud_topic", pointCloudTopic_, std::string("/points"));
  node_->param("robot_pose_with_covariance_topic", robotPoseTopic_, std::string("/pose"));
  node_->param("track_point_frame_id", trackPointFrameId_, std::string("/robot"));
  node_->param("track_point_x", trackPoint_.x(), 0.0);
  node_->param("track_point_y", trackPoint_.y(), 0.0);
  node_->param("track_point_z", trackPoint_.z(), 0.0);

  node_->param("robot_pose_cache_size", robotPoseCacheSize_, 200);
  RCLCPP_ASSERT(robotPoseCacheSize_ >= 0);

  double minUpdateRate;
  node_->param("min_update_rate", minUpdateRate, 2.0);
  if (minUpdateRate == 0.0) {
    maxNoUpdateDuration_.fromSec(0.0);
    RCLCPP_WARN(this->get_logger(), "Rate for publishing the map is zero.");
  } else {
    maxNoUpdateDuration_.fromSec(1.0 / minUpdateRate);
  }
  RCLCPP_ASSERT(!maxNoUpdateDuration_.isZero());

  double timeTolerance;
  node_->param("time_tolerance", timeTolerance, 0.0);
  timeTolerance_.fromSec(timeTolerance);

  double fusedMapPublishingRate;
  node_->param("fused_map_publishing_rate", fusedMapPublishingRate, 1.0);
  if (fusedMapPublishingRate == 0.0) {
    fusedMapPublishTimerDuration_.fromSec(0.0);
    RCLCPP_WARN(
        "Rate for publishing the fused map is zero. The fused elevation map will not be published unless the service `triggerFusion` is "
        "called.");
  } else if (std::isinf(fusedMapPublishingRate)) {
    isContinuouslyFusing_ = true;
    fusedMapPublishTimerDuration_.fromSec(0.0);
  } else {
    fusedMapPublishTimerDuration_.fromSec(1.0 / fusedMapPublishingRate);
  }

  double visibilityCleanupRate;
  node_->param("visibility_cleanup_rate", visibilityCleanupRate, 1.0);
  if (visibilityCleanupRate == 0.0) {
    visibilityCleanupTimerDuration_.fromSec(0.0);
    RCLCPP_WARN(this->get_logger(), "Rate for visibility cleanup is zero and therefore disabled.");
  } else {
    visibilityCleanupTimerDuration_.fromSec(1.0 / visibilityCleanupRate);
    map_->visibilityCleanupDuration_ = 1.0 / visibilityCleanupRate;
  }

  // ElevationMap parameters. TODO Move this to the elevation map class.
  node_->param("map_frame_id", mapFrameId_, std::string("/map"));
  map_->setFrameId(mapFrameId_);

  grid_map::Length length;
  grid_map::Position position;
  double resolution;
  node_->param("length_in_x", length(0), 1.5);
  node_->param("length_in_y", length(1), 1.5);
  node_->param("position_x", position.x(), 0.0);
  node_->param("position_y", position.y(), 0.0);
  node_->param("resolution", resolution, 0.01);
  map_->setGeometry(length, resolution, position);

  node_->param("min_variance", map_->minVariance_, pow(0.003, 2));
  node_->param("max_variance", map_->maxVariance_, pow(0.03, 2));
  node_->param("mahalanobis_distance_threshold", map_->mahalanobisDistanceThreshold_, 2.5);
  node_->param("multi_height_noise", map_->multiHeightNoise_, pow(0.003, 2));
  node_->param("min_horizontal_variance", map_->minHorizontalVariance_, pow(resolution / 2.0, 2));  // two-sigma
  node_->param("max_horizontal_variance", map_->maxHorizontalVariance_, 0.5);
  node_->param("underlying_map_topic", map_->underlyingMapTopic_, std::string());
  node_->param("enable_visibility_cleanup", map_->enableVisibilityCleanup_, true);
  node_->param("enable_continuous_cleanup", map_->enableContinuousCleanup_, false);
  node_->param("scanning_duration", map_->scanningDuration_, 1.0);
  node_->param("masked_replace_service_mask_layer_name", maskedReplaceServiceMaskLayerName_, std::string("mask"));

  // Settings for initializing elevation map
  node_->param("initialize_elevation_map", initializeElevationMap_, false);
  node_->param("initialization_method", initializationMethod_, 0);
  node_->param("length_in_x_init_submap", lengthInXInitSubmap_, 1.2);
  node_->param("length_in_y_init_submap", lengthInYInitSubmap_, 1.8);
  node_->param("margin_init_submap", marginInitSubmap_, 0.3);
  node_->param("init_submap_height_offset", initSubmapHeightOffset_, 0.0);
  node_->param("target_frame_init_submap", targetFrameInitSubmap_, std::string("/footprint"));

  if (!robotMotionMapUpdater_->readParameters()) {
    return false;
  }

  return true;
}

bool ElevationMapping::initialize() {
  RCLCPP_INFO(this->get_logger(), "Elevation mapping node initializing ... ");
  fusionServiceThread_ = boost::thread(boost::bind(&ElevationMapping::runFusionServiceThread, this));
  rclcpp::Duration(1.0).sleep();  // Need this to get the TF caches fill up.
  resetMapUpdateTimer();
  fusedMapPublishTimer_.start();
  visibilityCleanupThread_ = boost::thread(boost::bind(&ElevationMapping::visibilityCleanupThread, this));
  visibilityCleanupTimer_.start();
  initializeElevationMap();
  return true;
}

void ElevationMapping::runFusionServiceThread() {
  rclcpp::Rate loopRate(20);

  while (node_->ok()) {
    fusionServiceQueue_.callAvailable();

    // Sleep until the next execution.
    loopRate.sleep();
  }
}

void ElevationMapping::visibilityCleanupThread() {
  rclcpp::Rate loopRate(20);

  while (node_->ok()) {
    visibilityCleanupQueue_.callAvailable();

    // Sleep until the next execution.
    loopRate.sleep();
  }
}

void ElevationMapping::pointCloudCallback(const sensor_msgs::msg::PointCloud2ConstPtr& pointCloudMsg, bool publishPointCloud,
                                          const SensorProcessorBase::Ptr& sensorProcessor) {
  RCLCPP_DEBUG(this->get_logger(), "Processing data from: %s", pointCloudMsg->header.frame_id.c_str());
  if (!updatesEnabled_) {
    RCLCPP_WARN_THROTTLE(
    this->get_logger(),
    *this->get_clock(),
    10,
     "Updating of elevation map is disabled. (Warning message is throttled, 10s.)");
    if (publishPointCloud) {
      map_->setTimestamp(rclcpp::Time::now());
      map_->postprocessAndPublishRawElevationMap();
    }
    return;
  }

  // Check if point cloud has corresponding robot pose at the beginning
  if (!receivedFirstMatchingPointcloudAndPose_) {
    const double oldestPoseTime = robotPoseCache_.getOldestTime().toSec();
    const double currentPointCloudTime = pointCloudMsg->header.stamp.toSec();

    if (currentPointCloudTime < oldestPoseTime) {
      RCLCPP_WARN_THROTTLE(
    this->get_logger(),
    *this->get_clock(),
    5,
     "No corresponding point cloud and pose are found. Waiting for first match. (Warning message is throttled, 5s.)");
      return;
    } else {
      RCLCPP_INFO(this->get_logger(), "First corresponding point cloud and pose found, elevation mapping started. ");
      receivedFirstMatchingPointcloudAndPose_ = true;
    }
  }

  stopMapUpdateTimer();

  // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud.
  // TODO(max): Double check with http://wiki.ros.org/hydro/Migration
  pcl::PCLPointCloud2 pcl_pc;
  pcl_conversions::toPCL(*pointCloudMsg, pcl_pc);

  PointCloudType::Ptr pointCloud(new PointCloudType);
  pcl::fromPCLPointCloud2(pcl_pc, *pointCloud);
  lastPointCloudUpdateTime_.fromNSec(1000 * pointCloud->header.stamp);

  RCLCPP_DEBUG(this->get_logger(), "ElevationMap received a point cloud (%i points) for elevation mapping.", static_cast<int>(pointCloud->size()));

  // Get robot pose covariance matrix at timestamp of point cloud.
  Eigen::Matrix<double, 6, 6> robotPoseCovariance;
  robotPoseCovariance.setZero();
  if (!ignoreRobotMotionUpdates_) {
    boost::shared_ptr<geometry_msgs::msg::PoseWithCovarianceStamped const> poseMessage =
        robotPoseCache_.getElemBeforeTime(lastPointCloudUpdateTime_);
    if (!poseMessage) {
      // Tell the user that either for the timestamp no pose is available or that the buffer is possibly empty
      if (robotPoseCache_.getOldestTime().toSec() > lastPointCloudUpdateTime_.toSec()) {
        RCLCPP_ERROR(this->get_logger(), "The oldest pose available is at %f, requested pose at %f", robotPoseCache_.getOldestTime().toSec(),
                  lastPointCloudUpdateTime_.toSec());
      } else {
        RCLCPP_ERROR(this->get_logger(), "Could not get pose information from robot for time %f. Buffer empty?", lastPointCloudUpdateTime_.toSec());
      }
      return;
    }
    robotPoseCovariance = Eigen::Map<const Eigen::MatrixXd>(poseMessage->pose.covariance.data(), 6, 6);
  }

  // Process point cloud.
  PointCloudType::Ptr pointCloudProcessed(new PointCloudType);
  Eigen::VectorXf measurementVariances;
  if (!sensorProcessor->process(pointCloud, robotPoseCovariance, pointCloudProcessed, measurementVariances,
                                 pointCloudMsg->header.frame_id)) {
    if (!sensorProcessor->isTfAvailableInBuffer()) {
      RCLCPP_INFO_THROTTLE(
    this->get_logger(),
    *this->get_clock(),
    10,
     "Waiting for tf transformation to be available. (Message is throttled, 10s.)");
      return;
    }
    RCLCPP_ERROR(this->get_logger(), "Point cloud could not be processed.");
    resetMapUpdateTimer();
    return;
  }

  boost::recursive_mutex::scoped_lock scopedLock(map_->getRawDataMutex());

  // Update map location.
  updateMapLocation();

  // Update map from motion prediction.
  if (!updatePrediction(lastPointCloudUpdateTime_)) {
    RCLCPP_ERROR(this->get_logger(), "Updating process noise failed.");
    resetMapUpdateTimer();
    return;
  }

  // Clear the map if continuous clean-up was enabled.
  if (map_->enableContinuousCleanup_) {
    RCLCPP_DEBUG(this->get_logger(), "Clearing elevation map before adding new point cloud.");
    map_->clear();
  }

  // Add point cloud to elevation map.
  if (!map_->add(pointCloudProcessed, measurementVariances, lastPointCloudUpdateTime_,
                Eigen::Affine3d(sensorProcessor->transformationSensorToMap_))) {
    RCLCPP_ERROR(this->get_logger(), "Adding point cloud to elevation map failed.");
    resetMapUpdateTimer();
    return;
  }

  if (publishPointCloud) {
    // Publish elevation map.
    map_->postprocessAndPublishRawElevationMap();
    if (isFusingEnabled()) {
      map_->fuseAll();
      map_->publishFusedElevationMap();
    }
  }

  resetMapUpdateTimer();
}

void ElevationMapping::mapUpdateTimerCallback(const rclcpp::TimerEvent&) {
  if (!updatesEnabled_) {
    RCLCPP_WARN_THROTTLE(
    this->get_logger(),
    *this->get_clock(),
    10,
     "Updating of elevation map is disabled. (Warning message is throttled, 10s.)");
    map_->setTimestamp(rclcpp::Time::now());
    map_->postprocessAndPublishRawElevationMap();
    return;
  }

  rclcpp::Time time = rclcpp::Time::now();
  if ((lastPointCloudUpdateTime_ - time) <= maxNoUpdateDuration_) {  // there were updates from sensordata, no need to force an update.
    return;
  }
  RCLCPP_WARN_THROTTLE(
    this->get_logger(),
    *this->get_clock(),
    5,
     "Elevation map is updated without data from the sensor. (Warning message is throttled, 5s.)");

  boost::recursive_mutex::scoped_lock scopedLock(map_->getRawDataMutex());

  stopMapUpdateTimer();

  // Update map from motion prediction.
  if (!updatePrediction(time)) {
    RCLCPP_ERROR(this->get_logger(), "Updating process noise failed.");
    resetMapUpdateTimer();
    return;
  }

  // Publish elevation map.
  map_->postprocessAndPublishRawElevationMap();
  if (isFusingEnabled()) {
    map_->fuseAll();
    map_->publishFusedElevationMap();
  }

  resetMapUpdateTimer();
}

void ElevationMapping::publishFusedMapCallback(const rclcpp::TimerEvent&) {
  if (!map_->hasFusedMapSubscribers()) {
    return;
  }
  RCLCPP_DEBUG(this->get_logger(), "Elevation map is fused and published from timer.");
  boost::recursive_mutex::scoped_lock scopedLock(map_->getFusedDataMutex());
  map_->fuseAll();
  map_->publishFusedElevationMap();
}

void ElevationMapping::visibilityCleanupCallback(const rclcpp::TimerEvent&) {
  RCLCPP_DEBUG(this->get_logger(), "Elevation map is running visibility cleanup.");
  // Copy constructors for thread-safety.
  map_->visibilityCleanup(rclcpp::Time(lastPointCloudUpdateTime_));
}

bool ElevationMapping::fuseEntireMapServiceCallback(std_srvs::srv::Empty::Request&, std_srvs::srv::Empty::Response&) {
  boost::recursive_mutex::scoped_lock scopedLock(map_->getFusedDataMutex());
  map_->fuseAll();
  map_->publishFusedElevationMap();
  return true;
}

bool ElevationMapping::isFusingEnabled() {
  return isContinuouslyFusing_ && map_->hasFusedMapSubscribers();
}

bool ElevationMapping::updatePrediction(const rclcpp::Time& time) {
  if (ignoreRobotMotionUpdates_) {
    return true;
  }

  RCLCPP_DEBUG(this->get_logger(), "Updating map with latest prediction from time %f.", robotPoseCache_.getLatestTime().toSec());

  if (time + timeTolerance_ < map_->getTimeOfLastUpdate()) {
    RCLCPP_ERROR(this->get_logger(), "Requested update with time stamp %f, but time of last update was %f.", time.toSec(), map_->getTimeOfLastUpdate().toSec());
    return false;
  } else if (time < map_->getTimeOfLastUpdate()) {
    RCLCPP_DEBUG(this->get_logger(), "Requested update with time stamp %f, but time of last update was %f. Ignoring update.", time.toSec(),
              map_->getTimeOfLastUpdate().toSec());
    return true;
  }

  // Get robot pose at requested time.
  boost::shared_ptr<geometry_msgs::msg::PoseWithCovarianceStamped const> poseMessage = robotPoseCache_.getElemBeforeTime(time);
  if (!poseMessage) {
    // Tell the user that either for the timestamp no pose is available or that the buffer is possibly empty
    if (robotPoseCache_.getOldestTime().toSec() > lastPointCloudUpdateTime_.toSec()) {
      RCLCPP_ERROR(this->get_logger(), "The oldest pose available is at %f, requested pose at %f", robotPoseCache_.getOldestTime().toSec(),
                lastPointCloudUpdateTime_.toSec());
    } else {
      RCLCPP_ERROR(this->get_logger(), "Could not get pose information from robot for time %f. Buffer empty?", lastPointCloudUpdateTime_.toSec());
    }
    return false;
  }

  kindr::HomTransformQuatD robotPose;
  kindr_ros::convertFromRosGeometryMsg(poseMessage->pose.pose, robotPose);
  // Covariance is stored in row-major in ROS: http://docs.ros.org/api/geometry_msgs/html/msg/PoseWithCovariance.html
  Eigen::Matrix<double, 6, 6> robotPoseCovariance =
      Eigen::Map<const Eigen::Matrix<double, 6, 6, Eigen::RowMajor>>(poseMessage->pose.covariance.data(), 6, 6);

  // Compute map variance update from motion prediction.
  robotMotionMapUpdater_->update(map_, robotPose, robotPoseCovariance, time);

  return true;
}

bool ElevationMapping::updateMapLocation() {
  RCLCPP_DEBUG(this->get_logger(), "Elevation map is checked for relocalization.");

  geometry_msgs::msg::PointStamped trackPoint;
  trackPoint.header.frame_id = trackPointFrameId_;
  trackPoint.header.stamp = rclcpp::Time(0);
  kindr_ros::convertToRosGeometryMsg(trackPoint_, trackPoint.point);
  geometry_msgs::msg::PointStamped trackPointTransformed;

  try {
    trackPointTransformed = transformBuffer_->.transform(trackPoint, map_->getFrameId());
  } catch (tf2::TransformException& ex) {
    RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
    return false;
  }

  kindr::Position3D position3d;
  kindr_ros::convertFromRosGeometryMsg(trackPointTransformed.point, position3d);
  grid_map::Position position = position3d.vector().head(2);
  map_->move(position);
  return true;
}

bool ElevationMapping::getFusedSubmapServiceCallback(grid_map_msgs::srv::GetGridMap::Request& request,
                                                     grid_map_msgs::srv::GetGridMap::Response& response) {
  grid_map::Position requestedSubmapPosition(request.position_x, request.position_y);
  grid_map::Length requestedSubmapLength(request.length_x, request.length_y);
  RCLCPP_DEBUG(this->get_logger(), "Elevation submap request: Position x=%f, y=%f, Length x=%f, y=%f.", requestedSubmapPosition.x(), requestedSubmapPosition.y(),
            requestedSubmapLength(0), requestedSubmapLength(1));
  boost::recursive_mutex::scoped_lock scopedLock(map_->getFusedDataMutex());
  map_->fuseArea(requestedSubmapPosition, requestedSubmapLength);

  bool isSuccess;
  grid_map::Index index;
  grid_map::GridMap subMap = map_->getFusedGridMap().getSubmap(requestedSubmapPosition, requestedSubmapLength, index, isSuccess);
  scopedLock.unlock();

  if (request.layers.empty()) {
    grid_map::GridMapRosConverter::toMessage(subMap, response.map);
  } else {
    std::vector<std::string> layers;
    for (const std::string& layer : request.layers) {
      layers.push_back(layer);
    }
    grid_map::GridMapRosConverter::toMessage(subMap, layers, response.map);
  }

  RCLCPP_DEBUG(this->get_logger(), "Elevation submap responded with timestamp %f.", map_->getTimeOfLastFusion().toSec());
  return isSuccess;
}

bool ElevationMapping::getRawSubmapServiceCallback(grid_map_msgs::srv::GetGridMap::Request& request,
                                                   grid_map_msgs::srv::GetGridMap::Response& response) {
  grid_map::Position requestedSubmapPosition(request.position_x, request.position_y);
  grid_map::Length requestedSubmapLength(request.length_x, request.length_y);
  RCLCPP_DEBUG(this->get_logger(), "Elevation raw submap request: Position x=%f, y=%f, Length x=%f, y=%f.", requestedSubmapPosition.x(),
            requestedSubmapPosition.y(), requestedSubmapLength(0), requestedSubmapLength(1));
  boost::recursive_mutex::scoped_lock scopedLock(map_->getRawDataMutex());

  bool isSuccess;
  grid_map::Index index;
  grid_map::GridMap subMap = map_->getRawGridMap().getSubmap(requestedSubmapPosition, requestedSubmapLength, index, isSuccess);
  scopedLock.unlock();

  if (request.layers.empty()) {
    grid_map::GridMapRosConverter::toMessage(subMap, response.map);
  } else {
    std::vector<std::string> layers;
    for (const std::string& layer : request.layers) {
      layers.push_back(layer);
    }
    grid_map::GridMapRosConverter::toMessage(subMap, layers, response.map);
  }
  return isSuccess;
}

bool ElevationMapping::disableUpdatesServiceCallback(std_srvs::srv::Empty::Request& /*request*/, std_srvs::srv::Empty::Response& /*response*/) {
  RCLCPP_INFO(this->get_logger(), "Disabling updates.");
  updatesEnabled_ = false;
  return true;
}

bool ElevationMapping::enableUpdatesServiceCallback(std_srvs::srv::Empty::Request& /*request*/, std_srvs::srv::Empty::Response& /*response*/) {
  RCLCPP_INFO(this->get_logger(), "Enabling updates.");
  updatesEnabled_ = true;
  return true;
}

bool ElevationMapping::initializeElevationMap() {
  if (initializeElevationMap_) {
    if (static_cast<elevation_mapping::InitializationMethods>(initializationMethod_) ==
        elevation_mapping::InitializationMethods::PlanarFloorInitializer) {
      geometry_msgs::msg::TransformStamped transform_msg;
      tf2::Stamped<tf2::Transform> transform;

      // Listen to transform between mapFrameId_ and targetFrameInitSubmap_ and use z value for initialization
      try {
        transform_msg = transformBuffer_->.lookupTransform(mapFrameId_, targetFrameInitSubmap_, rclcpp::Time(0), rclcpp::Duration(5.0));
        tf2::fromMsg(transform_msg, transform);

        RCLCPP_DEBUG_STREAM(this->get_logger(), "Initializing with x: " << transform.getOrigin().x() << " y: " << transform.getOrigin().y()
                                                 << " z: " << transform.getOrigin().z());

        const grid_map::Position positionRobot(transform.getOrigin().x(), transform.getOrigin().y());

        // Move map before we apply the height values. This prevents unwanted behavior from intermediate move() calls in
        // updateMapLocation().
        map_->move(positionRobot);

        map_->setRawSubmapHeight(positionRobot, transform.getOrigin().z() + initSubmapHeightOffset_, lengthInXInitSubmap_,
                                lengthInYInitSubmap_, marginInitSubmap_);
        return true;
      } catch (tf2::TransformException& ex) {
        RCLCPP_DEBUG(this->get_logger(), "%s", ex.what());
        RCLCPP_WARN(this->get_logger(), "Could not initialize elevation map with constant height. (This warning can be ignored if TF tree is not available.)");
        return false;
      }
    }
  }
  return true;
}

bool ElevationMapping::clearMapServiceCallback(std_srvs::srv::Empty::Request& /*request*/, std_srvs::srv::Empty::Response& /*response*/) {
  RCLCPP_INFO(this->get_logger(), "Clearing map...");
  bool success = map_->clear();
  success &= initializeElevationMap();
  RCLCPP_INFO(this->get_logger(), "Map cleared.");

  return success;
}

bool ElevationMapping::maskedReplaceServiceCallback(grid_map_msgs::srv::SetGridMap::Request& request,
                                                    grid_map_msgs::srv::SetGridMap::Response& /*response*/) {
  RCLCPP_INFO(this->get_logger(), "Masked replacing of map.");
  grid_map::GridMap sourceMap;
  grid_map::GridMapRosConverter::fromMessage(request.map, sourceMap);

  // Use the supplied mask or do not use a mask
  grid_map::Matrix mask;
  if (sourceMap.exists(maskedReplaceServiceMaskLayerName_)) {
    mask = sourceMap[maskedReplaceServiceMaskLayerName_];
  } else {
    mask = Eigen::MatrixXf::Ones(sourceMap.getSize()(0), sourceMap.getSize()(1));
  }

  boost::recursive_mutex::scoped_lock scopedLockRawData(map_->getRawDataMutex());

  // Loop over all layers that should be set
  for (auto sourceLayerIterator = sourceMap.getLayers().begin(); sourceLayerIterator != sourceMap.getLayers().end();
       sourceLayerIterator++) {
    // skip "mask" layer
    if (*sourceLayerIterator == maskedReplaceServiceMaskLayerName_) {
      continue;
    }
    grid_map::Matrix& sourceLayer = sourceMap[*sourceLayerIterator];
    // Check if the layer exists in the elevation map
    if (map_->getRawGridMap().exists(*sourceLayerIterator)) {
      grid_map::Matrix& destinationLayer = map_->getRawGridMap()[*sourceLayerIterator];
      for (grid_map::GridMapIterator destinationIterator(map_->getRawGridMap()); !destinationIterator.isPastEnd(); ++destinationIterator) {
        // Use the position to find corresponding indices in source and destination
        const grid_map::Index destinationIndex(*destinationIterator);
        grid_map::Position position;
        map_->getRawGridMap().getPosition(*destinationIterator, position);

        if (!sourceMap.isInside(position)) {
          continue;
        }

        grid_map::Index sourceIndex;
        sourceMap.getIndex(position, sourceIndex);
        // If the mask allows it, set the value from source to destination
        if (!std::isnan(mask(sourceIndex(0), sourceIndex(1)))) {
          destinationLayer(destinationIndex(0), destinationIndex(1)) = sourceLayer(sourceIndex(0), sourceIndex(1));
        }
      }
    } else {
      RCLCPP_ERROR(this->get_logger(), "Masked replace service: Layer %s does not exist!", sourceLayerIterator->c_str());
    }
  }

  return true;
}

bool ElevationMapping::saveMapServiceCallback(grid_map_msgs::srv::ProcessFile::Request& request,
                                              grid_map_msgs::srv::ProcessFile::Response& response) {
  RCLCPP_INFO(this->get_logger(), "Saving map to file.");
  boost::recursive_mutex::scoped_lock scopedLock(map_->getFusedDataMutex());
  map_->fuseAll();
  std::string topic = node_->getNamespace() + "/elevation_map";
  if (!request.topic_name.empty()) {
    topic = node_->getNamespace() + "/" + request.topic_name;
  }
  response.success = static_cast<unsigned char>(grid_map::GridMapRosConverter::saveToBag(map_->getFusedGridMap(), request.file_path, topic));
  response.success = static_cast<unsigned char>(
      (grid_map::GridMapRosConverter::saveToBag(map_->getRawGridMap(), request.file_path + "_raw", topic + "_raw")) &&
      static_cast<bool>(response.success));
  return static_cast<bool>(response.success);
}

bool ElevationMapping::loadMapServiceCallback(grid_map_msgs::srv::ProcessFile::Request& request,
                                              grid_map_msgs::srv::ProcessFile::Response& response) {
  RCLCPP_WARN(this->get_logger(), "Loading from bag file.");
  boost::recursive_mutex::scoped_lock scopedLockFused(map_->getFusedDataMutex());
  boost::recursive_mutex::scoped_lock scopedLockRaw(map_->getRawDataMutex());

  std::string topic = node_->getNamespace();
  if (!request.topic_name.empty()) {
    topic += "/" + request.topic_name;
  } else {
    topic += "/elevation_map";
  }

  response.success =
      static_cast<unsigned char>(grid_map::GridMapRosConverter::loadFromBag(request.file_path, topic, map_->getFusedGridMap()));
  response.success = static_cast<unsigned char>(
      grid_map::GridMapRosConverter::loadFromBag(request.file_path + "_raw", topic + "_raw", map_->getRawGridMap()) &&
      static_cast<bool>(response.success));

  // Update timestamp for visualization in ROS
  map_->setTimestamp(rclcpp::Time::now());
  map_->postprocessAndPublishRawElevationMap();
  return static_cast<bool>(response.success);
}

void ElevationMapping::resetMapUpdateTimer() {
  mapUpdateTimer_.stop();
  rclcpp::Duration periodSinceLastUpdate = rclcpp::Time::now() - map_->getTimeOfLastUpdate();
  if (periodSinceLastUpdate > maxNoUpdateDuration_) {
    periodSinceLastUpdate.fromSec(0.0);
  }
  mapUpdateTimer_.setPeriod(maxNoUpdateDuration_ - periodSinceLastUpdate);
  mapUpdateTimer_.start();
}

void ElevationMapping::stopMapUpdateTimer() {
  mapUpdateTimer_.stop();
}

}  // namespace elevation_mapping
