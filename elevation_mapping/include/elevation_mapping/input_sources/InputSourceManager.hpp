/*
 *  InputSourceManager.hpp
 *
 *  Created on: Oct 02, 2020
 *  Author: Magnus Gärtner
 *  Institute: ETH Zurich, ANYbotics
 */

#pragma once

#include "elevation_mapping/input_sources/Input.hpp"

#include <rclcpp/rclcpp.hpp>

namespace elevation_mapping {
class ElevationMapping;  // Forward declare to avoid cyclic import dependency.

/**
 * @brief An input source manager reads a list of input sources from the configuration and connects them to the appropriate callback of
 * elevation mapping.
 */
class InputSourceManager {
 public:
  using SharedPtr = std::shared_ptr<InputSourceManager>;
  /**
   * @brief Constructor.
   * @param node Used to resolve the namespace and setup the subscribers.
   */
  explicit InputSourceManager(const rclcpp::Node::SharedPtr node);

  /**
   * @brief Configure the input sources.
   * This will configure all managed input sources.
   * @param inputSourcesNamespace The namespace of the subscribers list to load.
   * @return True if configuring was successful.
   */
  bool configureFromRos(const std::string& inputSourcesNamespace);

  /**
   * @brief Get all the input sources managed by the InputSourceManager
   * 
   * @return vector of input sources
   */
  std::vector<Input> getSources() { return sources_; };

  /**
   * @return The number of successfully configured input sources.
   */
  int getNumberOfSources();

 protected:
  //! A list of input sources.
  std::vector<Input> sources_;

  //! Node handle to load.
  rclcpp::Node::SharedPtr node_;
};

}  // namespace elevation_mapping
