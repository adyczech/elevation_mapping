/*
 *  Input.cpp
 *
 *  Created on: Oct 06, 2020
 *  Author: Magnus Gärtner
 *  Institute: ETH Zurich, ANYbotics
 */

#include "elevation_mapping/input_sources/Input.hpp"

#include "elevation_mapping/sensor_processors/LaserSensorProcessor.hpp"
#include "elevation_mapping/sensor_processors/PerfectSensorProcessor.hpp"
#include "elevation_mapping/sensor_processors/StereoSensorProcessor.hpp"
#include "elevation_mapping/sensor_processors/StructuredLightSensorProcessor.hpp"

namespace elevation_mapping {

Input::Input(rclcpp::Node::SharedPtr node) : node_(node), queueSize_(0), publishOnUpdate_(true) {}

bool Input::configure(std::string inputNamespace,
                      const SensorProcessorBase::GeneralParameters& generalSensorProcessorParameters) {  
  node_->declare_parameter(std::string(inputNamespace + ".type"), std::string());
  node_->declare_parameter(std::string(inputNamespace + ".topic"), std::string());
  node_->declare_parameter(std::string(inputNamespace + ".queue_size"), 1);
  node_->declare_parameter(std::string(inputNamespace + ".publish_on_update"), false);

  if (!node_->get_parameter(std::string(inputNamespace + ".type"), dataType_) || dataType_.empty()) {
    RCLCPP_ERROR(node_->get_logger(), "No sensor type specified for elevation mapping sensor %s; ignoring.", inputNamespace.c_str());
    return false;
  }

  if (!node_->get_parameter(std::string(inputNamespace + ".topic"), topic_) || 
      topic_.empty() ||
      topic_[0] == '~') {
    RCLCPP_ERROR(node_->get_logger(), "No sensor topic specified for elevation mapping sensor %s; ignoring.", inputNamespace.c_str());
    return false;
  }

  if (!node_->get_parameter(std::string(inputNamespace + ".queue_size"), queueSize_)) {
    RCLCPP_ERROR(node_->get_logger(), "No sensor queue size specified for elevation mapping sensor %s; ignoring.", inputNamespace.c_str());
    return false;
  } if (queueSize_ <= 0) {
    RCLCPP_INFO_STREAM(node_->get_logger(), "Skipping elevation mapping sensor " << inputNamespace << " because queue size '" << queueSize_ << "' is <1");
    return false;
  }

  if (!node_->get_parameter(std::string(inputNamespace + ".publish_on_update"), publishOnUpdate_)) {
    RCLCPP_ERROR(node_->get_logger(), "No sensor publish on update specified for elevation mapping sensor %s; ignoring.", inputNamespace.c_str());
    return false;
  }

  // SensorProcessor
  if (!configureSensorProcessor(std::string(inputNamespace + ".sensor_processor"), generalSensorProcessorParameters)) {
    return false;
  }

  RCLCPP_DEBUG(node_->get_logger(), "Configured %s:%s @ %s (publishing_on_update: %s), using %s to process data.\n", dataType_.c_str(), inputNamespace.c_str(),
            node_->get_node_base_interface()->resolve_topic_or_service_name(topic_, false, true).c_str(),
            publishOnUpdate_ ? "true" : "false",
            dataType_.c_str());
  return true;
}

std::string Input::getSubscribedTopic() const {
  return topic_;  // TODO(SivertHavso): need to resolve?
}

bool Input::configureSensorProcessor(std::string processorNamespace,
                                     const SensorProcessorBase::GeneralParameters& generalSensorProcessorParameters) {
  std::string sensorType;

  node_->declare_parameter(std::string(processorNamespace + ".sensor_processor.type"), std::string());

  if (!node_->get_parameter(std::string(processorNamespace + ".sensor_processor.type"), sensorType)) {
    RCLCPP_ERROR(node_->get_logger(), "No sensor processor type specified for elevation mapping sensor %s; ignoring.", processorNamespace.c_str());
    return false;
  } if (topic_.empty() || topic_[0] == '~') {
    RCLCPP_INFO_STREAM(node_->get_logger(), "Skipping elevation mapping sensor processor type '" << sensorType << "'");
    return false;
  }

  if (sensorType == "structured_light") {
    sensorProcessor_.reset(new StructuredLightSensorProcessor(node_, generalSensorProcessorParameters));
  } else if (sensorType == "stereo") {
    sensorProcessor_.reset(new StereoSensorProcessor(node_, generalSensorProcessorParameters));
  } else if (sensorType == "laser") {
    sensorProcessor_.reset(new LaserSensorProcessor(node_, generalSensorProcessorParameters));
  } else if (sensorType == "perfect") {
    sensorProcessor_.reset(new PerfectSensorProcessor(node_, generalSensorProcessorParameters));
  } else {
    RCLCPP_ERROR(node_->get_logger(), "The sensor type %s is not available.", sensorType.c_str());
    return false;
  }

  return sensorProcessor_->readParameters(std::string(processorNamespace + ".sensor_processor"));
}

}  // namespace elevation_mapping
