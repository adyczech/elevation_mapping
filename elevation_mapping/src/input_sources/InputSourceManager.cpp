/*
 *  InputSourceManager.cpp
 *
 *  Created on: Oct 02, 2020
 *  Author: Magnus Gärtner
 *  Institute: ETH Zurich, ANYbotics
 */

#include "elevation_mapping/input_sources/InputSourceManager.hpp"
#include "elevation_mapping/ElevationMapping.hpp"

namespace elevation_mapping {

InputSourceManager::InputSourceManager(const rclcpp::Node::SharedPtr node) : node_(node) {}

bool InputSourceManager::configureFromRos(const std::string& inputSourcesNamespace) {
  std::vector<std::string> sensor_names;

    std::stringstream ss;
  ss << inputSourcesNamespace << "." << "sensors";
  std::string full_sensor_parameter_name = ss.str();

  node_->declare_parameter(full_sensor_parameter_name, std::vector<std::string>());

  if (!node_->get_parameter(full_sensor_parameter_name, sensor_names)) {
    RCLCPP_ERROR(node_->get_logger(), "No sensor plugin(s) defined for elevation mapping. Not subscribing to any inputs.");
  } else if (sensor_names.empty()) {
    RCLCPP_ERROR(node_->get_logger(), "List of sensor plugin(s) is empty. Not subscribing to any inputs.");
  }

  bool successfulConfiguration = true;
  std::set<std::string> subscribedTopics;
  SensorProcessorBase::GeneralParameters generalSensorProcessorConfig(node_->get_parameter("robot_base_frame_id").as_string(),
                                                                      node_->get_parameter("map_frame_id").as_string());

  for (const auto& sensor_name : sensor_names) {
    // TODO(SivertHavso): Confirm whether a new node for each input should be made.
    Input source = Input(node_);

    if (!source.configure(std::string("input_sources." + sensor_name), generalSensorProcessorConfig)) {
      successfulConfiguration = false;
    }

    std::string subscribedTopic = source.getSubscribedTopic();
    bool topicIsUnique = subscribedTopics.insert(subscribedTopic).second;

    if (topicIsUnique) {
      sources_.push_back(std::move(source));
    } else {
      RCLCPP_WARN(
          node_->get_logger(),
          "The input sources specification tried to subscribe to %s "
          "multiple times. Only subscribing once.",
          subscribedTopic.c_str());
      successfulConfiguration = false;
    }
  }

  return successfulConfiguration;
}

int InputSourceManager::getNumberOfSources() {
  return static_cast<int>(sources_.size());
}

}  // namespace elevation_mapping