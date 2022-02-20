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

PostprocessingPipelineFunctor::PostprocessingPipelineFunctor(rclcpp::Node::SharedPtr node)
    : node_(node), filterChain_("grid_map::GridMap"), filterChainConfigured_(false) {
  // TODO (magnus) Add logic when setting up failed. What happens actually if it is not configured?
  readParameters();

  publisher_ = node_->create_publisher<grid_map_msgs::msg::GridMap>(outputTopic_, rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(qos_grid_map)));

  // Setup filter chain.
  if (!node.hasParam(filterChainParametersName_) || !filterChain_.configure(filterChainParametersName_, node)) {
    RCLCPP_WARN(node_->get_logger(), "Could not configure the filter chain. Will publish the raw elevation map without postprocessing!");
    return;
  }

  filterChainConfigured_ = true;
}

PostprocessingPipelineFunctor::~PostprocessingPipelineFunctor() = default;

void PostprocessingPipelineFunctor::readParameters() {
  node_->param("output_topic", outputTopic_, std::string("elevation_map_raw"));
  node_->param("postprocessor_pipeline_name", filterChainParametersName_, std::string("postprocessor_pipeline"));
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
