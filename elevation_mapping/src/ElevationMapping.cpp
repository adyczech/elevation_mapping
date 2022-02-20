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
#include <mutex>

#include <grid_map_msgs/msg/grid_map.hpp>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <boost/bind.hpp>
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

  systemClock_ = std::make_shared<rclcpp::Clock>();

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
  fusionTriggerService_ = create_service<std_srvs::srv::Empty>(
    "trigger_fusion",
    std::bind(&ElevationMapping::fuseEntireMapServiceCallback, this, std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default,
    fusionCallbackGroup_);

  fusedSubmapService_ = create_service<grid_map_msgs::srv::GetGridMap>(
    "get_submap",
    std::bind(&ElevationMapping::getFusedSubmapServiceCallback, this, std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default,
    fusionCallbackGroup_);

  rawSubmapService_ = create_service<grid_map_msgs::srv::GetGridMap>(
    "get_raw_submap",
    std::bind(&ElevationMapping::getRawSubmapServiceCallback, this, std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default,
    fusionCallbackGroup_);

  clearMapService_ = create_service<std_srvs::srv::Empty>(
    "clear_map",
    std::bind(&ElevationMapping::clearMapServiceCallback, this, std::placeholders::_1, std::placeholders::_2));

  enableUpdatesService_ = create_service<std_srvs::srv::Empty>(
    "enable_updates",
    std::bind(&ElevationMapping::enableUpdatesServiceCallback, this, std::placeholders::_1, std::placeholders::_2));

  disableUpdatesService_ = create_service<std_srvs::srv::Empty>(
    "disable_updates",
    std::bind(&ElevationMapping::disableUpdatesServiceCallback, this, std::placeholders::_1, std::placeholders::_2));

  maskedReplaceService_ = create_service<grid_map_msgs::srv::SetGridMap>(
    "masked_replace",
    std::bind(&ElevationMapping::maskedReplaceServiceCallback, this, std::placeholders::_1, std::placeholders::_2));

  saveMapService_ = create_service<grid_map_msgs::srv::ProcessFile>(
    "save_map",
    std::bind(&ElevationMapping::saveMapServiceCallback, this, std::placeholders::_1, std::placeholders::_2));

  loadMapService_ = create_service<grid_map_msgs::srv::ProcessFile>(
    "load_map",
    std::bind(&ElevationMapping::loadMapServiceCallback, this, std::placeholders::_1, std::placeholders::_2));
}

void ElevationMapping::setupTimers() {
  fusionCallbackGroup_ = create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive
  );
  visibilityCleanupCallbackGroup_ = create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive
  );

  mapUpdateTimer_ = this->create_wall_timer(
    rclcpp::Duration::to_chrono<std::chrono::duration<long double, std::milli>>(maxNoUpdateDuration_),
    std::bind(&ElevationMapping::mapUpdateTimerCallback, this));

  mapUpdateTimer_->cancel();  // Do not start the timer yet

  if (fusedMapPublishTimerDuration_.nanoseconds() != 0) {
    fusedMapPublishTimer_ = create_wall_timer(
      fusedMapPublishTimerDuration_,
      std::bind(&ElevationMapping::publishFusedMapCallback, this),
      fusionCallbackGroup_
    );

    fusedMapPublishTimer_->cancel();  // Do not start the timer yet
  }

  // Multi-threading for visibility cleanup. Visibility clean-up does not help when continuous clean-up is enabled.
  if (map_->enableVisibilityCleanup_ && 
      visibilityCleanupTimerDuration_.nanoseconds() != 0  && 
    !map_->enableContinuousCleanup_) {
    visibilityCleanupTimer_ = create_wall_timer(
      visibilityCleanupTimerDuration_,
      std::bind(&ElevationMapping::visibilityCleanupCallback, this),
      visibilityCleanupCallbackGroup_
    );

    visibilityCleanupTimer_->cancel();  // Do not start the timer yet
  }
}

ElevationMapping::~ElevationMapping() {}

bool ElevationMapping::readParameters() {
  // ElevationMapping parameters.
  this->param("point_cloud_topic", pointCloudTopic_, std::string("/points"));
  this->param("robot_pose_with_covariance_topic", robotPoseTopic_, std::string("/pose"));
  this->param("track_point_frame_id", trackPointFrameId_, std::string("/robot"));
  this->param("track_point_x", trackPoint_.x(), 0.0);
  this->param("track_point_y", trackPoint_.y(), 0.0);
  this->param("track_point_z", trackPoint_.z(), 0.0);

  this->param("robot_pose_cache_size", robotPoseCacheSize_, 200);
  RCLCPP_ASSERT(robotPoseCacheSize_ >= 0);

  double minUpdateRate;
  this->param("min_update_rate", minUpdateRate, 2.0);
  if (minUpdateRate == 0.0) {
    maxNoUpdateDuration_.fromSec(0.0);
    RCLCPP_WARN(this->get_logger(), "Rate for publishing the map is zero.");
  } else {
    maxNoUpdateDuration_.fromSec(1.0 / minUpdateRate);
  }
  RCLCPP_ASSERT(!maxNoUpdateDuration_.isZero());

  double timeTolerance;
  this->param("time_tolerance", timeTolerance, 0.0);
  timeTolerance_.fromSec(timeTolerance);

  double fusedMapPublishingRate;
  this->param("fused_map_publishing_rate", fusedMapPublishingRate, 1.0);
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
  this->param("visibility_cleanup_rate", visibilityCleanupRate, 1.0);
  if (visibilityCleanupRate == 0.0) {
    visibilityCleanupTimerDuration_.fromSec(0.0);
    RCLCPP_WARN(this->get_logger(), "Rate for visibility cleanup is zero and therefore disabled.");
  } else {
    visibilityCleanupTimerDuration_.fromSec(1.0 / visibilityCleanupRate);
    map_->visibilityCleanupDuration_ = 1.0 / visibilityCleanupRate;
  }

  // ElevationMap parameters. TODO Move this to the elevation map class.
  this->param("map_frame_id", mapFrameId_, std::string("/map"));
  map_->setFrameId(mapFrameId_);

  grid_map::Length length;
  grid_map::Position position;
  double resolution;
  this->param("length_in_x", length(0), 1.5);
  this->param("length_in_y", length(1), 1.5);
  this->param("position_x", position.x(), 0.0);
  this->param("position_y", position.y(), 0.0);
  this->param("resolution", resolution, 0.01);
  map_->setGeometry(length, resolution, position);

  this->param("min_variance", map_->minVariance_, pow(0.003, 2));
  this->param("max_variance", map_->maxVariance_, pow(0.03, 2));
  this->param("mahalanobis_distance_threshold", map_->mahalanobisDistanceThreshold_, 2.5);
  this->param("multi_height_noise", map_->multiHeightNoise_, pow(0.003, 2));
  this->param("min_horizontal_variance", map_->minHorizontalVariance_, pow(resolution / 2.0, 2));  // two-sigma
  this->param("max_horizontal_variance", map_->maxHorizontalVariance_, 0.5);
  this->param("underlying_map_topic", map_->underlyingMapTopic_, std::string());
  this->param("enable_visibility_cleanup", map_->enableVisibilityCleanup_, true);
  this->param("enable_continuous_cleanup", map_->enableContinuousCleanup_, false);
  this->param("scanning_duration", map_->scanningDuration_, 1.0);
  this->param("masked_replace_service_mask_layer_name", maskedReplaceServiceMaskLayerName_, std::string("mask"));

  // Settings for initializing elevation map
  this->param("initialize_elevation_map", initializeElevationMap_, false);
  this->param("initialization_method", initializationMethod_, 0);
  this->param("length_in_x_init_submap", lengthInXInitSubmap_, 1.2);
  this->param("length_in_y_init_submap", lengthInYInitSubmap_, 1.8);
  this->param("margin_init_submap", marginInitSubmap_, 0.3);
  this->param("init_submap_height_offset", initSubmapHeightOffset_, 0.0);
  this->param("target_frame_init_submap", targetFrameInitSubmap_, std::string("/footprint"));

  if (!robotMotionMapUpdater_->readParameters()) {
    return false;
  }

  return true;
}

bool ElevationMapping::initialize() {
  RCLCPP_INFO(this->get_logger(), "Elevation mapping node initializing ... ");

  rclcpp::sleep_for(std::chrono::duration<int64_t, std::milli>(1000));  // Need this to get the TF caches fill up. FIXME?
  // resetMapUpdateTimer();  // FIXME?
  fusedMapPublishTimer_.reset();
  visibilityCleanupTimer_.reset();  // FIXME?
  initializeElevationMap();
  return true;
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
    resetMapUpdateTimer();
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
     resetMapUpdateTimer();
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

  std::scoped_lock scopedLock(map_->getRawDataMutex());

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

void ElevationMapping::mapUpdateTimerCallback() {
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

  std::scoped_lock scopedLock(map_->getRawDataMutex());

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

void ElevationMapping::publishFusedMapCallback() {
  if (!map_->hasFusedMapSubscribers()) {
    return;
  }
  RCLCPP_DEBUG(this->get_logger(), "Elevation map is fused and published from timer.");
  std::scoped_lock scopedLock(map_->getFusedDataMutex());
  map_->fuseAll();
  map_->publishFusedElevationMap();
}

void ElevationMapping::visibilityCleanupCallback() {
  RCLCPP_DEBUG(this->get_logger(), "Elevation map is running visibility cleanup.");
  // Copy constructors for thread-safety.
  map_->visibilityCleanup(rclcpp::Time(lastPointCloudUpdateTime_));
}

bool ElevationMapping::fuseEntireMapServiceCallback(std_srvs::srv::Empty::Request::SharedPtr, std_srvs::srv::Empty::Response::SharedPtr) {
  std::scoped_lock scopedLock(map_->getFusedDataMutex());
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

bool ElevationMapping::getFusedSubmapServiceCallback(grid_map_msgs::srv::GetGridMap::Request::SharedPtr request,
                                                     grid_map_msgs::srv::GetGridMap::Response::SharedPtr response) {
  grid_map::Position requestedSubmapPosition(request->position_x, request->position_y);
  grid_map::Length requestedSubmapLength(request->length_x, request->length_y);
  RCLCPP_DEBUG(this->get_logger(), "Elevation submap request: Position x=%f, y=%f, Length x=%f, y=%f.", requestedSubmapPosition.x(), requestedSubmapPosition.y(),
            requestedSubmapLength(0), requestedSubmapLength(1));

  bool isSuccess;
  grid_map::GridMap subMap;
  {
    std::scoped_lock scopedLock(map_->getFusedDataMutex());
    map_->fuseArea(requestedSubmapPosition, requestedSubmapLength);

    subMap = map_->getFusedGridMap().getSubmap(requestedSubmapPosition, requestedSubmapLength, isSuccess);
    // scopedLock.unlock();
  }

  if (request->layers.empty()) {
    grid_map::GridMapRosConverter::toMessage(subMap, response->map);
  } else {
    std::vector<std::string> layers;
    for (const std::string& layer : request->layers) {
      layers.push_back(layer);
    }
    grid_map::GridMapRosConverter::toMessage(subMap, layers, response->map);
  }

  RCLCPP_DEBUG(this->get_logger(), "Elevation submap responded with timestamp %f.", map_->getTimeOfLastFusion().toSec());
  return isSuccess;
}

bool ElevationMapping::getRawSubmapServiceCallback(grid_map_msgs::srv::GetGridMap::Request::SharedPtr request,
                                                   grid_map_msgs::srv::GetGridMap::Response::SharedPtr response) {
  grid_map::Position requestedSubmapPosition(request->position_x, request->position_y);
  grid_map::Length requestedSubmapLength(request->length_x, request->length_y);
  RCLCPP_DEBUG(this->get_logger(), "Elevation raw submap request: Position x=%f, y=%f, Length x=%f, y=%f.", requestedSubmapPosition.x(),
            requestedSubmapPosition.y(), requestedSubmapLength(0), requestedSubmapLength(1));
  bool isSuccess;
  grid_map::GridMap subMap;
  {
    std::scoped_lock scopedLock(map_->getRawDataMutex());

    subMap = map_->getRawGridMap().getSubmap(requestedSubmapPosition, requestedSubmapLength, isSuccess);
    // scopedLock.unlock();
  }

  if (request->layers.empty()) {
    grid_map::GridMapRosConverter::toMessage(subMap, response->map);
  } else {
    std::vector<std::string> layers;
    for (const std::string& layer : request->layers) {
      layers.push_back(layer);
    }
    grid_map::GridMapRosConverter::toMessage(subMap, layers, response->map);
  }
  return isSuccess;
}

bool ElevationMapping::disableUpdatesServiceCallback(std_srvs::srv::Empty::Request::SharedPtr /*request*/, std_srvs::srv::Empty::Response::SharedPtr /*response*/) {
  RCLCPP_INFO(this->get_logger(), "Disabling updates.");
  updatesEnabled_ = false;
  return true;
}

bool ElevationMapping::enableUpdatesServiceCallback(std_srvs::srv::Empty::Request::SharedPtr /*request*/, std_srvs::srv::Empty::Response::SharedPtr /*response*/) {
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

bool ElevationMapping::clearMapServiceCallback(std_srvs::srv::Empty::Request::SharedPtr /*request*/, std_srvs::srv::Empty::Response::SharedPtr /*response*/) {
  RCLCPP_INFO(this->get_logger(), "Clearing map...");
  bool success = map_->clear();
  success &= initializeElevationMap();
  RCLCPP_INFO(this->get_logger(), "Map cleared.");

  return success;
}

bool ElevationMapping::maskedReplaceServiceCallback(grid_map_msgs::srv::SetGridMap::Request::SharedPtr request,
                                                    grid_map_msgs::srv::SetGridMap::Response::SharedPtr /*response*/) {
  RCLCPP_INFO(this->get_logger(), "Masked replacing of map.");
  grid_map::GridMap sourceMap;
  grid_map::GridMapRosConverter::fromMessage(request->map, sourceMap);

  // Use the supplied mask or do not use a mask
  grid_map::Matrix mask;
  if (sourceMap.exists(maskedReplaceServiceMaskLayerName_)) {
    mask = sourceMap[maskedReplaceServiceMaskLayerName_];
  } else {
    mask = Eigen::MatrixXf::Ones(sourceMap.getSize()(0), sourceMap.getSize()(1));
  }

  std::scoped_lock scopedLockRawData(map_->getRawDataMutex());

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

bool ElevationMapping::saveMapServiceCallback(grid_map_msgs::srv::ProcessFile::Request::SharedPtr request,
                                              grid_map_msgs::srv::ProcessFile::Response::SharedPtr response) {
  RCLCPP_INFO(this->get_logger(), "Saving map to file.");
  std::scoped_lock scopedLock(map_->getFusedDataMutex());
  map_->fuseAll();
  std::string topic = this->getNamespace() + "/elevation_map";
  if (!request->topic_name.empty()) {
    topic = this->getNamespace() + "/" + request->topic_name;
  }
  response->success = static_cast<unsigned char>(grid_map::GridMapRosConverter::saveToBag(map_->getFusedGridMap(), request->file_path, topic));
  response->success = static_cast<unsigned char>(
      (grid_map::GridMapRosConverter::saveToBag(map_->getRawGridMap(), request->file_path + "_raw", topic + "_raw")) &&
      static_cast<bool>(response->success));
  return static_cast<bool>(response->success);
}

bool ElevationMapping::loadMapServiceCallback(grid_map_msgs::srv::ProcessFile::Request::SharedPtr request,
                                              grid_map_msgs::srv::ProcessFile::Response::SharedPtr response) {
  RCLCPP_WARN(this->get_logger(), "Loading from bag file.");
  std::scoped_lock scopedLockFused(map_->getFusedDataMutex());
  std::scoped_lock scopedLockRaw(map_->getRawDataMutex());

  std::string topic = this->getNamespace();
  if (!request->topic_name.empty()) {
    topic += "/" + request->topic_name;
  } else {
    topic += "/elevation_map";
  }

  response->success =
      static_cast<unsigned char>(grid_map::GridMapRosConverter::loadFromBag(request->file_path, topic, map_->getFusedGridMap()));
  response->success = static_cast<unsigned char>(
      grid_map::GridMapRosConverter::loadFromBag(request->file_path + "_raw", topic + "_raw", map_->getRawGridMap()) &&
      static_cast<bool>(response->success));

  // Update timestamp for visualization in ROS
  map_->setTimestamp(rclcpp::Time::now());
  map_->postprocessAndPublishRawElevationMap();
  return static_cast<bool>(response->success);
}

void ElevationMapping::resetMapUpdateTimer() {
  mapUpdateTimer_->cancel();
  rclcpp::Duration periodSinceLastUpdate = systemClock_->now() - map_->getTimeOfLastUpdate();
  if (periodSinceLastUpdate > maxNoUpdateDuration_) {
    periodSinceLastUpdate.from_nanoseconds(0);
  }
  auto period = (maxNoUpdateDuration_ - periodSinceLastUpdate);
  if (period.nanoseconds() < 0) {
    period = rclcpp::Duration::from_nanoseconds(0);
  }

  mapUpdateTimer_ = create_wall_timer(
    (period).to_chrono<std::chrono::duration<long double, std::milli>>(),
    std::bind(&ElevationMapping::mapUpdateTimerCallback, this)
  );
}

void ElevationMapping::stopMapUpdateTimer() {
  mapUpdateTimer_->cancel();
}

}  // namespace elevation_mapping
