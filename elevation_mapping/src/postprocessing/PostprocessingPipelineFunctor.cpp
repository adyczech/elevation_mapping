/*
 * PostprocessingPipelineFunctor.cpp
 *
 *  Created on: Sep. 14, 2020
 *      Author: Magnus Gärtner
 *   Institute: ETH Zurich, ANYbotics
 *
 *  Note. Large parts are adopted from grid_map_demos/FiltersDemo.cpp.
 */

#include <grid_map_ros/grid_map_ros.hpp>

#include "elevation_mapping/postprocessing/PostprocessingPipelineFunctor.hpp"
#include "elevation_mapping/QoS.hpp"

namespace elevation_mapping {

using namespace std::literals::chrono_literals;

PostprocessingPipelineFunctor::PostprocessingPipelineFunctor(rclcpp::Node::SharedPtr node)
    : node_(node),
    maxNoUpdateDuration_(0ns),
    filterChain_("grid_map::GridMap"), 
    filterChainConfigured_(false) {
  // TODO (magnus) Add logic when setting up failed. What happens actually if it is not configured?
  readParameters();

  publisher_ = node_->create_publisher<grid_map_msgs::msg::GridMap>(
    outputTopic_, default_qos(1, maxNoUpdateDuration_));

  // Setup filter chain.
  if (!node_->has_parameter(filterChainParametersName_) || !filterChain_.configure(
    filterChainParametersName_,
    node_->get_node_logging_interface(),
    node_->get_node_parameters_interface())) {
    RCLCPP_WARN(node_->get_logger(), "Could not configure the filter chain. Will publish the raw elevation map without postprocessing!");
    return;
  }

  filterChainConfigured_ = true;
}

PostprocessingPipelineFunctor::~PostprocessingPipelineFunctor() = default;

void PostprocessingPipelineFunctor::readParameters() {
  node_->declare_parameter("output_topic", std::string("elevation_map_raw"));
  node_->declare_parameter("postprocessor_pipeline_name", std::string("postprocessor_pipeline"));
  node_->get_parameter("output_topic", outputTopic_);
  node_->get_parameter("postprocessor_pipeline_name", filterChainParametersName_);

  double minUpdateRate;
  if (!node_->get_parameter("min_update_rate", minUpdateRate)) {
    RCLCPP_ERROR(node_->get_logger(), 
      "Failed to read the 'min_update_rate parameter'. Has it been declared (ElevationMapping::readParameters)?");
  } else if (minUpdateRate == 0.0) {
    maxNoUpdateDuration_ = rclcpp::Duration::from_nanoseconds(0);
    RCLCPP_WARN(node_->get_logger(), "Rate for publishing the map is zero.");
  } else {
    maxNoUpdateDuration_ = rclcpp::Duration::from_seconds(1.0 / minUpdateRate);
  }
}

grid_map::GridMap PostprocessingPipelineFunctor::operator()(GridMap& inputMap) {
  if (not filterChainConfigured_) {
    RCLCPP_WARN_ONCE(node_->get_logger(), "No postprocessing pipeline was configured. Forwarding the raw elevation map!");
    return inputMap;
  }

  grid_map::GridMap outputMap;
  if (not filterChain_.update(inputMap, outputMap)) {
    RCLCPP_ERROR(node_->get_logger(), "Could not perform the grid map filter chain! Forwarding the raw elevation map!");
    return inputMap;
  }

  return outputMap;
}

void PostprocessingPipelineFunctor::publish(const GridMap& gridMap) const {
  // Publish filtered output grid map.
  grid_map_msgs::msg::GridMap::UniquePtr outputMessage;
  outputMessage = grid_map::GridMapRosConverter::toMessage(gridMap);
  publisher_->publish(std::move(outputMessage));
  RCLCPP_DEBUG(node_->get_logger(), "Elevation map raw has been published.");
}

bool PostprocessingPipelineFunctor::hasSubscribers() const {
  return publisher_->get_subscription_count() > 0;
}

}  // namespace elevation_mapping
