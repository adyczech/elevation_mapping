/*
 * ElevationMap.hpp
 *
 *  Created on: Nov 12, 2013
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, ANYbotics
 */

#pragma once

// Grid Map
#include <grid_map_msgs/srv/get_grid_map.hpp>
#include <grid_map_msgs/srv/process_file.hpp>
#include <grid_map_msgs/srv/set_grid_map.hpp>

// ROS
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <message_filters/cache.h>
#include <message_filters/subscriber.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/empty.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

// Elevation Mapping
#include "elevation_mapping/ElevationMap.hpp"
#include "elevation_mapping/PointXYZRGBConfidenceRatio.hpp"
#include "elevation_mapping/RobotMotionMapUpdater.hpp"
#include "elevation_mapping/WeightedEmpiricalCumulativeDistributionFunction.hpp"
#include "elevation_mapping/input_sources/InputSourceManager.hpp"

namespace elevation_mapping {

enum class InitializationMethods { PlanarFloorInitializer };

/*!
 * The elevation mapping main class. Coordinates the ROS interfaces, the timing,
 * and the data handling between the other classes.
 */
class ElevationMapping : public rclcpp::Node {
 public:
  /*!
   * Constructor.
   *
   * @param node the shared pointer to the ROS node node handle.
   */
  explicit ElevationMapping(const rclcpp::NodeOptions & node);

  /*!
   * Destructor.
   */
  virtual ~ElevationMapping();

  /*!
   * Callback function for new data to be added to the elevation map.
   *
   * @param pointCloudMsg    The point cloud to be fused with the existing data.
   * @param publishPointCloud If true, publishes the pointcloud after updating the map.
   * @param sensorProcessor The sensorProcessor to use in this callback.
   */
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::UniquePtr pointCloudMsg, bool publishPointCloud,
                          const SensorProcessorBase::UniquePtr& sensorProcessor);

  /*!
   * Callback function for the update timer. Forces an update of the map from
   * the robot's motion if no new measurements are received for a certain time
   * period.
   */
  void mapUpdateTimerCallback();

  /*!
   * Callback function for the fused map publish timer. Publishes the fused map
   * based on configurable duration.
   */
  void publishFusedMapCallback();

  /*!
   * Callback function for cleaning map based on visibility ray tracing.
   */
  void visibilityCleanupCallback();

  /*!
   * ROS service callback function to trigger the fusion of the entire
   * elevation map.
   *
   * @param request     The ROS service request.
   * @param response    The ROS service response.
   * @return true if successful.
   */
  bool fuseEntireMapServiceCallback(std_srvs::srv::Empty::Request::SharedPtr request, std_srvs::srv::Empty::Response::SharedPtr response);

  /*!
   * ROS service callback function to return a submap of the fused elevation map.
   *
   * @param request     The ROS service request defining the location and size of the fused submap.
   * @param response    The ROS service response containing the requested fused submap.
   * @return true if successful.
   */
  bool getFusedSubmapServiceCallback(grid_map_msgs::srv::GetGridMap::Request::SharedPtr request, grid_map_msgs::srv::GetGridMap::Response::SharedPtr response);

  /*!
   * ROS service callback function to return a submap of the raw elevation map.
   *
   * @param request     The ROS service request defining the location and size of the raw submap.
   * @param response    The ROS service response containing the requested raw submap.
   * @return true if successful.
   */
  bool getRawSubmapServiceCallback(grid_map_msgs::srv::GetGridMap::Request::SharedPtr request, grid_map_msgs::srv::GetGridMap::Response::SharedPtr response);

  /*!
   * ROS service callback function to enable updates of the elevation map.
   *
   * @param request     The ROS service request.
   * @param response    The ROS service response.
   * @return true if successful.
   */
  bool enableUpdatesServiceCallback(std_srvs::srv::Empty::Request::SharedPtr request, std_srvs::srv::Empty::Response::SharedPtr response);

  /*!
   * ROS service callback function to disable updates of the elevation map.
   *
   * @param request     The ROS service request.
   * @param response    The ROS service response.
   * @return true if successful.
   */
  bool disableUpdatesServiceCallback(std_srvs::srv::Empty::Request::SharedPtr request, std_srvs::srv::Empty::Response::SharedPtr response);

  /*!
   * ROS service callback function to clear all data of the elevation map.
   *
   * @param request     The ROS service request.
   * @param response    The ROS service response.
   * @return true if successful.
   */
  bool clearMapServiceCallback(std_srvs::srv::Empty::Request::SharedPtr request, std_srvs::srv::Empty::Response::SharedPtr response);

  /*!
   * ROS service callback function to allow for setting the individual layers of the elevation map through a service call.
   * The layer mask can be used to only set certain cells and not the entire map. Cells
   * containing NAN in the mask are not set, all the others are set. If the layer mask is
   * not supplied, the entire map will be set in the intersection of both maps. The
   * provided map can be of different size and position than the map that will be altered.
   *
   * @param request    The ROS service request.
   * @param response   The ROS service response.
   * @return true if successful.
   */
  bool maskedReplaceServiceCallback(grid_map_msgs::srv::SetGridMap::Request::SharedPtr request, grid_map_msgs::srv::SetGridMap::Response::SharedPtr response);

  /*!
   * ROS service callback function to save the grid map with all layers to a ROS bag file.
   *
   * @param request   The ROS service request.
   * @param response  The ROS service response.
   * @return true if successful.
   */
  bool saveMapServiceCallback(grid_map_msgs::srv::ProcessFile::Request::SharedPtr request, grid_map_msgs::srv::ProcessFile::Response::SharedPtr response);

  /*!
   * ROS service callback function to load the grid map with all layers from a ROS bag file.
   *
   * @param request     The ROS service request.
   * @param response    The ROS service response.
   * @return true if successful.
   */
  bool loadMapServiceCallback(grid_map_msgs::srv::ProcessFile::Request::SharedPtr request, grid_map_msgs::srv::ProcessFile::Response::SharedPtr response);

 private:
  /*!
   * Initializes the node.
   * 
   * Gets called by a timer after the node is constructed to allow for the use of
   * shared_from_this() when initializing member objects.
   */
  void initializeNode();

  /*!
   * Reads and verifies the ROS parameters.
   *
   * @return true if successful.
   */
  bool readParameters();

  /*!
   * Performs the initialization procedure.
   *
   * @return true if successful.
   */
  bool initialize();

  /**
   * Sets up the subscribers for both robot poses and input data.
   */
  void setupSubscribers();

  /**
   * Sets up the services.
   */
  void setupServices();

  /**
   * Sets up the timers.
   */
  void setupTimers();

  /*!
   * Update the elevation map from the robot motion up to a certain time.
   *
   * @param time    Time to which the map is updated to.
   * @return true if successful.
   */
  bool updatePrediction(const rclcpp::Time& time);

  /*!
   * Updates the location of the map to follow the tracking point. Takes care
   * of the data handling the goes along with the relocalization.
   *
   * @return true if successful.
   */
  bool updateMapLocation();

  /*!
   * Reset and start the map update timer.
   */
  void resetMapUpdateTimer();

  /*!
   * Stop the map update timer.
   */
  void stopMapUpdateTimer();

  /*!
   * Initializes a submap around the robot of the elevation map with a constant height.
   */
  bool initializeElevationMap();

  /*!
   * Returns true if fusing the map is enabled.
   */
  bool isFusingEnabled();

 protected:
  //! Input sources.
  std::shared_ptr<InputSourceManager> inputSources_;
  
  //! ROS subscribers.
  message_filters::Subscriber<geometry_msgs::msg::PoseWithCovarianceStamped> robotPoseSubscriber_;

  //! ROS service servers.
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr fusionTriggerService_;
  rclcpp::Service<grid_map_msgs::srv::GetGridMap>::SharedPtr fusedSubmapService_;
  rclcpp::Service<grid_map_msgs::srv::GetGridMap>::SharedPtr rawSubmapService_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr clearMapService_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr enableUpdatesService_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr disableUpdatesService_;
  rclcpp::Service<grid_map_msgs::srv::SetGridMap>::SharedPtr maskedReplaceService_;
  rclcpp::Service<grid_map_msgs::srv::ProcessFile>::SharedPtr saveMapService_;
  rclcpp::Service<grid_map_msgs::srv::ProcessFile>::SharedPtr loadMapService_;

  //! Callback group for fusion service.
  rclcpp::CallbackGroup::SharedPtr fusionCallbackGroup_;

  //! Cache for the robot pose messages.
  message_filters::Cache<geometry_msgs::msg::PoseWithCovarianceStamped> robotPoseCache_;

  //! Size of the cache for the robot pose messages.
  int robotPoseCacheSize_;

  //! Frame ID of the elevation map
  std::string mapFrameId_;

  //! TF listener and buffer.
  std::shared_ptr<tf2_ros::Buffer> transformBuffer_;
  std::shared_ptr<tf2_ros::TransformListener> transformListener_;

  //! Point which the elevation map follows.
  kindr::Position3D trackPoint_;
  std::string trackPointFrameId_;

  //! ROS topics for subscriptions.
  std::string robotPoseTopic_;

  //! Elevation map.
  std::shared_ptr<ElevationMap> map_;

  //! Robot motion elevation map updater.
  std::shared_ptr<RobotMotionMapUpdater> robotMotionMapUpdater_;

  //! If true, robot motion updates are ignored.
  bool ignoreRobotMotionUpdates_;

  //! If false, elevation mapping stops updating
  bool updatesEnabled_;

  //! Time of the last point cloud update.
  rclcpp::Time lastPointCloudUpdateTime_;

  //! Timer for the robot motion update. Uses RCL_ROS_TIME
  rclcpp::TimerBase::SharedPtr mapUpdateTimer_;

  //! Timer for the initialization of the node. Uses RCL_STEADY_TIME
  rclcpp::TimerBase::SharedPtr initTimer_;

  //! Maximum time that the map will not be updated.
  rclcpp::Duration maxNoUpdateDuration_;

  //! Time tolerance for updating the map with data before the last update.
  //! This is useful when having multiple sensors adding data to the map.
  rclcpp::Duration timeTolerance_;

  //! Timer for publishing the fused map. Uses RCL_STEADY_TIME
  rclcpp::TimerBase::SharedPtr fusedMapPublishTimer_;

  //! Duration for the publishing the fusing map.
  rclcpp::Duration fusedMapPublishTimerDuration_;

  //! If map is fused after every change for debugging/analysis purposes.
  bool isContinuouslyFusing_;

  //! Timer for the raytracing cleanup. Uses RCL_STEADY_TIME
  rclcpp::TimerBase::SharedPtr visibilityCleanupTimer_;

  //! Duration for the raytracing cleanup timer.
  rclcpp::Duration visibilityCleanupTimerDuration_;

  //! Callback group for raytracing cleanup.
  rclcpp::CallbackGroup::SharedPtr visibilityCleanupCallbackGroup_;

  //! Becomes true when corresponding poses and point clouds can be found
  bool receivedFirstMatchingPointcloudAndPose_;

  //! Name of the mask layer used in the masked replace service
  std::string maskedReplaceServiceMaskLayerName_;

  //! Enables initialization of the elevation map
  bool initializeElevationMap_;

  //! Enum to choose the initialization method
  int initializationMethod_;

  //! Width of submap of the elevation map with a constant height
  double lengthInXInitSubmap_;

  //! Height of submap of the elevation map with a constant height
  double lengthInYInitSubmap_;

  //! Margin of submap of the elevation map with a constant height
  double marginInitSubmap_;

  //! Target frame to get the init height of the elevation map
  std::string targetFrameInitSubmap_;

  //! Additional offset of the height value
  double initSubmapHeightOffset_;
};

}  // namespace elevation_mapping
