/*
 *  Input.hpp
 *
 *  Created on: Oct 06, 2020
 *  Author: Magnus Gärtner
 *  Institute: ETH Zurich, ANYbotics
 */

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <string>

#include "elevation_mapping/sensor_processors/SensorProcessorBase.hpp"
#include "elevation_mapping/QoS.hpp"

namespace elevation_mapping {
class ElevationMapping;  // Forward declare to avoid cyclic import dependency.

/**
 * @brief An Input feeds data to ElevationMapping callbacks. E.g it holds a subscription to an sensor source and registered an appropriate
 * ElevationMapping callback.
 */
class Input {
 public:
  template <typename MsgT>
  using CallbackT = void (ElevationMapping::*)(const std::shared_ptr<const MsgT>&, bool, const SensorProcessorBase::UniquePtr&);

  /**
   * @brief Constructor.
   * @param node Shared pointer to the manager Node. Used to subscribe
   * to inputs.
   */
  explicit Input(rclcpp::Node::SharedPtr node);

  /**
   * @brief Configure the input source.
   * @param inputNamespace The parameter namespace of the input parameters to load.
   * @param generalSensorProcessorParameters Parameters shared by all sensor processors.
   * @return True if configuring was successful.
   */
  bool configure(std::string inputNamespace,
                 const SensorProcessorBase::GeneralParameters& generalSensorProcessorParameters);

  /**
   * @brief Registers the corresponding callback in the elevationMap.
   * @param map The map we want to link this input source to.
   * @param callback The callback to use for incoming data.
   * @tparam MsgT The message types of the callback.
   */
  template <typename MsgT>
  void registerCallback(ElevationMapping& map, CallbackT<MsgT> callback);

  /**
   * @return The topic (as absolute path, with renames) that this input
   * subscribes to.
   */
  std::string getSubscribedTopic() const;

  /**
   * @return The type of this input source.
   */
  std::string getType() { return dataType_; }

 private:
  /**
   * @brief Configures the used sensor processor from the given parameters.
   * @param processorName The parameter namespace of the sensor processor
   * @param generalSensorProcessorParameters  General parameters needed for the sensor processor that are not specific to this sensor
   * processor.
   * @return True if successful.
   */
  bool configureSensorProcessor(std::string processorNamespace,
                                const SensorProcessorBase::GeneralParameters& generalSensorProcessorParameters);

  // ROS connection.
  rclcpp::SubscriptionBase::SharedPtr subscriber_;
  rclcpp::Node::SharedPtr node_;

  //! Sensor processor
  SensorProcessorBase::UniquePtr sensorProcessor_;

  // Parameters.
  std::string name_;
  std::string dataType_;
  uint32_t queueSize_;
  std::string topic_;
  bool publishOnUpdate_;
};

template <typename MsgT>
void Input::registerCallback(ElevationMapping& map, CallbackT<MsgT> callback) {
  subscriber_ = node_->create_subscription<MsgT>(
      topic_, default_qos(queueSize_), std::bind(callback, std::ref(map), std::placeholders::_1, publishOnUpdate_, std::ref(sensorProcessor_)));
  RCLCPP_INFO(node_->get_logger(), "Subscribing to %s: %s, queue_size: %i.", dataType_.c_str(), topic_.c_str(), queueSize_);
}

}  // namespace elevation_mapping